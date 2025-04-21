#include "autoware_interface/autoware_interface.hpp"

using namespace std::chrono_literals;

AutowareInterface::AutowareInterface() : Node("autoware_interface")
{   
    /* From Autoware */
    speed_angle_command_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::AWControlCommandCallback, this, std::placeholders::_1));
    gear_command_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
        "/control/command/gear_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::AWGearCommandCallback, this, std::placeholders::_1));
    
    /* From CANMessageHandler */
    speed_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/can_message_handler/velocity_status", rclcpp::QoS(1), std::bind(&AutowareInterface::VelocityStatusCallback, this, std::placeholders::_1));
    angle_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/can_message_handler/steering_status", rclcpp::QoS(1), std::bind(&AutowareInterface::SteeringStatusCallback, this, std::placeholders::_1));


    /* To TwistController */
    speed_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("/autoware_interface/velocity_cmd", rclcpp::QoS(1)); 
    angle_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("/autoware_interface/steering_cmd", rclcpp::QoS(1));

    /* To CANMessageHandler */
    gear_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("/autoware_interface/gear_cmd", rclcpp::QoS(1));
    
    /* To Autoware */
    velocity_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1));
    steering_tire_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1));
    control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));

    timer_ = this->create_wall_timer(10ms, std::bind(&AutowareInterface::TimerCallback, this));
}

void AutowareInterface::VelocityStatusCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    vehicle_speed_ = msg->data;
}

void AutowareInterface::SteeringStatusCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    steering_angle_ = msg->data;
}

void AutowareInterface::AWControlCommandCallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
    speed_command_ = msg->longitudinal.speed;
    angle_command_ = msg->lateral.steering_tire_angle;
}

void AutowareInterface::AWGearCommandCallback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{
    gear_command_ = msg->command;
}

void AutowareInterface::TimerCallback()
{    
    /* To Autoware */
    autoware_auto_vehicle_msgs::msg::VelocityReport velocity_status_msg;
    velocity_status_msg.header.stamp = this->now();
    velocity_status_msg.header.frame_id = "base_link";
    velocity_status_msg.longitudinal_velocity = vehicle_speed_;
    velocity_status_msg.heading_rate = (vehicle_speed_ * std::tan(steering_angle_)) / WHEEL_BASE;
    velocity_status_pub_->publish(velocity_status_msg);

    autoware_auto_vehicle_msgs::msg::SteeringReport steering_tire_status_msg;
    steering_tire_status_msg.stamp = this->now();
    steering_tire_status_msg.steering_tire_angle = steering_angle_; // / 15.7;
    steering_tire_status_pub_->publish(steering_tire_status_msg);

    autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.mode = 1;
    control_mode_pub_->publish(control_mode_msg);
    

    /* To TwistController */
    std_msgs::msg::Float64 speed_command_msg;
    speed_command_msg.data = speed_command_;
    speed_command_pub_->publish(speed_command_msg);

    std_msgs::msg::Float64 angle_command_msg;
    angle_command_msg.data = angle_command_; // * 15.7; // No steering wheel in KAMO, thus no steering wheel angle, only tire angle
    angle_command_pub_->publish(angle_command_msg);


    /* To CANMessageHandler */
    std_msgs::msg::Int32 gear_command_msg;
    gear_command_msg.data = static_cast<int32_t>(gear_command_);
    gear_command_pub_->publish(gear_command_msg);

}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutowareInterface>());
    rclcpp::shutdown();
    return 0;
}
