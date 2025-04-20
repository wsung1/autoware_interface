#include "autoware_interface/autoware_interface.hpp"

using namespace std::chrono_literals;

AutowareInterface::AutowareInterface() : Node("autoware_interface")
{   
    // Subscribers
    AW_speed_angle_command_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::AWControlCommandCallback, this, std::placeholders::_1));
    AW_gear_command_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
        "/control/command/gear_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::AWGearCommandCallback, this, std::placeholders::_1));
    TC_speed_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/input/velocity_status", rclcpp::QoS(1), std::bind(&AutowareInterface::VelocityStatusCallback, this, std::placeholders::_1));
    TC_angle_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/input/steering_status", rclcpp::QoS(1), std::bind(&AutowareInterface::SteeringStatusCallback, this, std::placeholders::_1));

    // Publishers
    TC_speed_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_cmd", rclcpp::QoS(1)); 
    TC_angle_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steering_cmd", rclcpp::QoS(1));
    CAN_gear_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("/can_message_handler/intput/gear_cmd", rclcpp::QoS(1));
    
    AW_speed_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1));
    AW_angle_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1));
    AW_control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));

    timer_ = this->create_wall_timer(10ms, std::bind(&AutowareInterface::TimerCallback, this));
}

void AutowareInterface::VelocityStatusCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    vehicle_speed_ = msg->data * KPH2MPS;
}

void AutowareInterface::SteeringStatusCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    steering_angle_ = msg->data * DEG2RAD;
}

void AutowareInterface::AWControlCommandCallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
    AW_speed_command_ = msg->longitudinal.speed;
    AW_angle_command_ = msg->lateral.steering_tire_angle;
}

void AutowareInterface::AWGearCommandCallback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{
    AW_gear_command_ = msg->command;
}

void AutowareInterface::TimerCallback()
{    
    /* To Autoware */
    autoware_auto_vehicle_msgs::msg::VelocityReport AW_velocity_status_msg;
    autoware_auto_vehicle_msgs::msg::SteeringReport AW_steering_tire_status_msg;
    autoware_auto_vehicle_msgs::msg::ControlModeReport AW_control_mode_msg;

    AW_velocity_status_msg.header.stamp = this->now();
    AW_velocity_status_msg.header.frame_id = "base_link";
    AW_velocity_status_msg.longitudinal_velocity = vehicle_speed_;
    AW_velocity_status_msg.heading_rate = (vehicle_speed_ * std::tan(steering_angle_)) / WHEEL_BASE;

    AW_steering_tire_status_msg.stamp = this->now();
    AW_steering_tire_status_msg.steering_tire_angle = steering_angle_; // / 15.7;
    
    AW_speed_status_pub_->publish(AW_velocity_status_msg);
    AW_angle_status_pub_->publish(AW_steering_tire_status_msg);

    AW_control_mode_msg.mode = 1;
    AW_control_mode_pub_->publish(AW_control_mode_msg);

    /* To TwistController */
    std_msgs::msg::Float64 TC_speed_command_msg;
    std_msgs::msg::Float64 TC_angle_command_msg;
    
    TC_speed_command_msg.data = AW_speed_command_;
    TC_angle_command_msg.data = AW_angle_command_; // * 15.7; // No steering wheel in KAMO, thus no steering wheel angle, only tire angle
    
    TC_speed_command_pub_->publish(TC_speed_command_msg);
    TC_angle_command_pub_->publish(TC_angle_command_msg);

    /* To CANMessageHandler */
    std_msgs::msg::Int32 CAN_gear_command_msg;
    CAN_gear_command_msg.data = static_cast<int32_t>(AW_gear_command_);
    CAN_gear_command_pub_->publish(CAN_gear_command_msg);

}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutowareInterface>());
    rclcpp::shutdown();
    return 0;
}
