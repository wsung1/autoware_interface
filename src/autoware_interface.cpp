#include "autoware_interface/autoware_interface.hpp"

using namespace std::chrono_literals;

AutowareInterface::AutowareInterface() : Node("autoware_interface")
{   
    /* From Autoware */
    speed_angle_command_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::AWControlCommandCallback, this, std::placeholders::_1));
    gear_command_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
        "/control/command/gear_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::AWGearCommandCallback, this, std::placeholders::_1));
    goal_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/planning/mission_planning/echo_back_goal_pose", rclcpp::QoS{1}.transient_local(), std::bind(&AutowareInterface::GoalPoseCallback, this, std::placeholders::_1));
    ego_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization/pose_twist_fusion_filter/pose", rclcpp::QoS(1), std::bind(&AutowareInterface::EgoPoseCallback, this, std::placeholders::_1));

    /* From CANMessageHandler */
    speed_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/can_message_receiver/velocity_status", rclcpp::QoS(1), std::bind(&AutowareInterface::VelocityStatusCallback, this, std::placeholders::_1));
    angle_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/can_message_receiver/steering_status", rclcpp::QoS(1), std::bind(&AutowareInterface::SteeringStatusCallback, this, std::placeholders::_1));
    gear_status_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/can_message_receiver/gear_state", rclcpp::QoS(1), std::bind(&AutowareInterface::GearStatusCallback, this, std::placeholders::_1));

    /* To TwistController */
    speed_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("/autoware_interface/velocity_cmd", rclcpp::QoS(1)); 
    angle_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("/autoware_interface/steering_cmd", rclcpp::QoS(1));
    gear_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("/autoware_interface/gear_cmd", rclcpp::QoS(1));
    force_brake_pub_ = this->create_publisher<std_msgs::msg::Float64>("/autoware_interface/force_brake", rclcpp::QoS(1));

    /* To Autoware */
    velocity_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1));
    steering_tire_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1));
    control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));
    gear_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
        "/vehicle/status/gear_status", rclcpp::QoS(1));

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

void AutowareInterface::GoalPoseCallback(const geometry_msgs::msg::PoseStamped msg)
{
    goal_pose_ = msg.pose;
}

void AutowareInterface::EgoPoseCallback(const geometry_msgs::msg::PoseStamped msg)
{
    geometry_msgs::msg::Pose ego_pose = msg.pose;
    dist_to_goal_ = std::hypot(ego_pose.position.x - goal_pose_.position.x, ego_pose.position.y - goal_pose_.position.y);
    RCLCPP_INFO(this->get_logger(), "distance : %f", dist_to_goal_);
}

void AutowareInterface::GearStatusCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    gear_status_ = msg->data;
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

    autoware_auto_vehicle_msgs::msg::GearReport gear_status_msg;
    if(gear_status_ == 1) gear_status_ = 22; //P
    else if(gear_status_ == 2) gear_status_ = 20; //R
    else if(gear_status_ == 3) gear_status_ = 1; //N
    else if(gear_status_ == 4) gear_status_ = 2; //D
    else gear_status_ = 22; //P
    gear_status_msg.report = gear_status_;
    gear_status_pub_->publish(gear_status_msg);

    /* To TwistController */
    std_msgs::msg::Float64 speed_command_msg;
    // float custom_mps_cmd = static_cast<int>(speed_command_ * 3.6) / 3.6; //HJK_250423
    speed_command_msg.data = speed_command_;
    // speed_command_msg.data = custom_mps_cmd;
    speed_command_pub_->publish(speed_command_msg);

    std_msgs::msg::Float64 angle_command_msg;
    angle_command_msg.data = angle_command_; // * 15.7; // No steering wheel in KAMO, thus no steering wheel angle, only tire angle
    angle_command_pub_->publish(angle_command_msg);

    std_msgs::msg::Int32 gear_command_msg;
    gear_command_msg.data = static_cast<int32_t>(gear_command_);
    
    std_msgs::msg::Float64 force_brake_msg;


    // if(gear_command_ == 22)
    // {
    //     if(force_brake_ == -1000.0)
    //     {
    //         gear_command_pub_->publish(gear_command_msg);
    //     }
    //     force_brake_msg.data = force_brake_;
    //     force_brake_pub_->publish(force_brake_msg);
    // }
    // else
    // {
    //     force_brake_ = 0.0;
    //     force_brake_msg.data = force_brake_;
    //     force_brake_pub_->publish(force_brake_msg);
    //     gear_command_pub_->publish(gear_command_msg);
    // }

    if(dist_to_goal_ < 3.0) //start force brake distance [m]
    {
        force_brake_ -= 1.5; //increment gain
        if(force_brake_ <= -1000.0)
        {
            if(gear_command_ == 22)
            {
                gear_command_pub_->publish(gear_command_msg);
            }
            force_brake_ = -1000.0;
        }
        force_brake_msg.data = force_brake_;
        force_brake_pub_->publish(force_brake_msg); 
    }
    else
    {
        force_brake_ = 0.0;
        force_brake_msg.data = force_brake_;
        force_brake_pub_->publish(force_brake_msg);
        gear_command_pub_->publish(gear_command_msg);
    }
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutowareInterface>());
    rclcpp::shutdown();
    return 0;
}
