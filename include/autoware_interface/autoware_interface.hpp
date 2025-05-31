#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#define KPH2MPS 1/3.6
#define DEG2RAD 0.0174533
#define WHEEL_BASE 2.57048

class AutowareInterface : public rclcpp::Node
{
    public:
        explicit AutowareInterface();

    private:
        rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr speed_angle_command_sub_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_command_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_status_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_status_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gear_status_sub_;
        
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_command_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_command_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gear_command_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_brake_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_tire_status_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
        
        rclcpp::TimerBase::SharedPtr timer_;

        double speed_command_ = 0.0;
        double angle_command_ = 0.0;
        int gear_command_ = 0;
        double force_brake_ = 0.0;
        int gear_status_ = 1;
        double steering_angle_ = 0.0;
        double vehicle_speed_ = 0.0;
        geometry_msgs::msg::Pose goal_pose_;
        double dist_to_goal_ = 0.0;

        void VelocityStatusCallback(const std_msgs::msg::Float64::SharedPtr msg);
        void SteeringStatusCallback(const std_msgs::msg::Float64::SharedPtr msg);
        void GearStatusCallback(const std_msgs::msg::Int32::SharedPtr msg);

        void AWControlCommandCallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
        void AWGearCommandCallback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg);
        void GoalPoseCallback(const geometry_msgs::msg::PoseStamped msg);
        void EgoPoseCallback(const geometry_msgs::msg::PoseStamped msg);
        void TimerCallback();
};