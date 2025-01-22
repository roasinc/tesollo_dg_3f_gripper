#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "angles/angles.h"
#include "delto_3f_interfaces/srv/set_data.hpp"
#include "control_msgs/action/gripper_command.hpp"

#include <modbus/modbus.h>

class Delto3FGripperDriver : public rclcpp::Node
{
    public:
        Delto3FGripperDriver(): Node("delto_3f_gripper_driver")
        {
            this->declare_parameter<std::string>("port_name", "/dev/ttyS0");
            this->declare_parameter<int>("baudrate", 115200);
            this->declare_parameter<double>("rate", 10.0);

            auto port_name = this->get_parameter("port_name").get_parameter_value().get<std::string>();
            auto baudrate = this->get_parameter("baudrate").get_parameter_value().get<uint32_t>();
            RCLCPP_INFO(this->get_logger(), "Port: [%s] and baudrate %ld", port_name.c_str(), baudrate);

            modbus_ = modbus_new_rtu(port_name.c_str(), baudrate, 'N', 8, 1);
            if(modbus_connect(modbus_) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to connect MODBUS server...");
                assert(false);
            }
            modbus_set_slave(modbus_, 1);

            uint16_t registers[2] = {0, };
            if(modbus_read_input_registers(modbus_, 0x0, 2, registers) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to read input registers...");
                assert(false);
            }
            RCLCPP_INFO(this->get_logger(), "Gripper Product ID: [0x%X], Firmware Version: [%d]", registers[0], registers[1]);

            // ROS Related
            pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            srv_set_grasp_mode_ = this->create_service<delto_3f_interfaces::srv::SetData>("set_grasp_mode",
                                std::bind(&Delto3FGripperDriver::set_grasp_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
            srv_set_grasp_torque_ = this->create_service<delto_3f_interfaces::srv::SetData>("set_grasp_torque",
                                std::bind(&Delto3FGripperDriver::set_grasp_torque_callback, this, std::placeholders::_1, std::placeholders::_2));

            as_grasp_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(
                this,
                "gripper_command",
                std::bind(&Delto3FGripperDriver::handle_grasp_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Delto3FGripperDriver::handle_grasp_cancel, this, std::placeholders::_1),
                std::bind(&Delto3FGripperDriver::handle_grasp_accepted, this, std::placeholders::_1)
            );

            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&Delto3FGripperDriver::timer_callback, this));
            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }
        ~Delto3FGripperDriver() {}

    private:
        void timer_callback()
        {
            m_.lock();
            uint16_t registers[24] = {0, };
            if(modbus_read_input_registers(modbus_, 0x02, 24, registers) == -1)
            {
                m_.unlock();
                return;
            }

            m_.unlock();

            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = this->now();
            msg.name = {"gripper_f1m1_joint", "gripper_f1m2_joint", "gripper_f1m3_joint", "gripper_f1m4_joint",
                        "gripper_f2m1_joint", "gripper_f2m2_joint", "gripper_f2m3_joint", "gripper_f2m4_joint",
                        "gripper_f3m1_joint", "gripper_f3m2_joint", "gripper_f3m3_joint", "gripper_f3m4_joint"};

            msg.position = {
                angles::from_degrees(int16_t(registers[0]) / 10.0),
                angles::from_degrees(int16_t(registers[1]) / 10.0),
                angles::from_degrees(int16_t(registers[2]) / 10.0),
                angles::from_degrees(int16_t(registers[3]) / 10.0),
                angles::from_degrees(int16_t(registers[4]) / 10.0),
                angles::from_degrees(int16_t(registers[5]) / 10.0),
                angles::from_degrees(int16_t(registers[6]) / 10.0),
                angles::from_degrees(int16_t(registers[7]) / 10.0),
                angles::from_degrees(int16_t(registers[8]) / 10.0),
                angles::from_degrees(int16_t(registers[9]) / 10.0),
                angles::from_degrees(int16_t(registers[10]) / 10.0),
                angles::from_degrees(int16_t(registers[11]) / 10.0)
            };

            msg.effort = {
                int16_t(registers[12]) / 1000.0,
                int16_t(registers[13]) / 1000.0,
                int16_t(registers[14]) / 1000.0,
                int16_t(registers[15]) / 1000.0,
                int16_t(registers[16]) / 1000.0,
                int16_t(registers[17]) / 1000.0,
                int16_t(registers[18]) / 1000.0,
                int16_t(registers[19]) / 1000.0,
                int16_t(registers[20]) / 1000.0,
                int16_t(registers[21]) / 1000.0,
                int16_t(registers[22]) / 1000.0,
                int16_t(registers[23]) / 1000.0
            };

            pub_joint_states_->publish(msg);
        }

        void set_grasp_mode_callback(const std::shared_ptr<delto_3f_interfaces::srv::SetData::Request> request,
                                        std::shared_ptr<delto_3f_interfaces::srv::SetData::Response> response)
        {
            if(request->set_data < 1 || request->set_data > 3)
            {
                response->result = false;
                response->message = "Invalid grasp mode...";
                return;
            }

            m_.lock();
            if(modbus_write_register(modbus_, 0x43, request->set_data) == -1)
            {
                m_.unlock();
                response->result = false;
                response->message = "Write register failed...";
                return;
            }

            m_.unlock();
            RCLCPP_INFO(this->get_logger(), "Gripper Set Grasp Mode: [%d]...", request->set_data);
            response->result = true;
        }

        void set_grasp_torque_callback(const std::shared_ptr<delto_3f_interfaces::srv::SetData::Request> request,
                                        std::shared_ptr<delto_3f_interfaces::srv::SetData::Response> response)
        {
            if(request->set_data < 0 || request->set_data > 200)
            {
                response->result = false;
                response->message = "Invalid grasp torque...";
                return;
            }

            m_.lock();
            if(modbus_write_register(modbus_, 0x44, request->set_data) == -1)
            {
                m_.unlock();
                response->result = false;
                response->message = "Write register failed...";
                return;
            }

            m_.unlock();
            RCLCPP_INFO(this->get_logger(), "Gripper Set Grasp Torque: [%d]...", request->set_data);
            response->result = true;
        }

        void handle_grasp_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
        {
            using namespace std::placeholders;
            std::thread{ std::bind(&Delto3FGripperDriver::execute_grasp, this, _1), goal_handle }.detach();
        }

        rclcpp_action::GoalResponse handle_grasp_goal(const rclcpp_action::GoalUUID &uuid,
                                std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received request Grasp.");
            (void)uuid;
            (void)goal;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_grasp_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void execute_grasp(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
        {
            auto feedback = std::make_shared<control_msgs::action::GripperCommand::Feedback>();
            auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
            auto goal = goal_handle->get_goal();

            RCLCPP_INFO(this->get_logger(), "Executing Grasp...");

            if(goal->command.max_effort < 0 || goal->command.max_effort > 200)
            {
                m_.unlock();
                RCLCPP_ERROR(this->get_logger(), "Invalid Grasp Torque...");
                result->position = 0;
                goal_handle->abort(result);
                return;
            }

            m_.lock();
            if(modbus_write_register(modbus_, 0x44, goal->command.max_effort) == -1)
            {
                m_.unlock();
                result->position = 0;
                goal_handle->abort(result);
                return;
            }
            m_.unlock();

            if(goal->command.position == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Opening Gripper...");
                m_.lock();
                if(modbus_write_bit(modbus_, 0x1, 0) == -1)
                {
                    m_.unlock();
                    RCLCPP_ERROR(this->get_logger(), "Failed to write single bit...");
                    result->position = 0;
                    goal_handle->abort(result);
                    return;
                }
                m_.unlock();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Closing Gripper...");
                m_.lock();
                if(modbus_write_bit(modbus_, 0x1, 1) == -1)
                {
                    m_.unlock();
                    RCLCPP_ERROR(this->get_logger(), "Failed to write single bit...");
                    result->position = 0;
                    goal_handle->abort(result);
                    return;
                }
                m_.unlock();
            }

            rclcpp::sleep_for(std::chrono::milliseconds(500));
            result->position = goal->command.position;
            result->reached_goal = true;
            goal_handle->succeed(result);

            RCLCPP_INFO(this->get_logger(), "Grasp Succeeded.");
        }

    private:
        modbus_t *modbus_;
        std::mutex m_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
        rclcpp::Service<delto_3f_interfaces::srv::SetData>::SharedPtr srv_set_grasp_mode_;
        rclcpp::Service<delto_3f_interfaces::srv::SetData>::SharedPtr srv_set_grasp_torque_;
        rclcpp_action::Server<control_msgs::action::GripperCommand>::SharedPtr as_grasp_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Delto3FGripperDriver>());

    rclcpp::shutdown();
    return 0;
}