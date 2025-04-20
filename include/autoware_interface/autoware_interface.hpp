#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/int32.hpp"

#define KPH2MPS 1/3.6
#define DEG2RAD 0.0174533
#define WHEEL_BASE 2.57048

class AutowareInterface : public rclcpp::Node
{
    public:
        explicit AutowareInterface();

    private:
        rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr AW_speed_angle_command_sub_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr AW_gear_command_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_speed_status_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_angle_status_sub_;
        
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_speed_command_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_angle_command_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr CAN_gear_command_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr AW_speed_status_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr AW_angle_status_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr AW_control_mode_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        double AW_speed_command_ = 0.0;
        double AW_angle_command_ = 0.0;
        int AW_gear_command_ = 0;
        double steering_angle_ = 0.0;
        double vehicle_speed_ = 0.0;

        void VelocityStatusCallback(const std_msgs::msg::Float64::SharedPtr msg);
        void SteeringStatusCallback(const std_msgs::msg::Float64::SharedPtr msg);
        void AWControlCommandCallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
        void AWGearCommandCallback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg);
        void TimerCallback();
};