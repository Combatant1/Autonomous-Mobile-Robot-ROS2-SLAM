#ifndef NOISY_CONTROLLER_HPP
#define NOISY_CONTROLLER_HPP

#include <random>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class NoisyController : public rclcpp::Node
{
public:
    NoisyController(const std::string & node_name);

private:
    void jointCallback(const sensor_msgs::msg::JointState & msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double wheel_radius_;
    double wheel_separation_;

    double front_left_wheel_prev_pos_;
    double front_right_wheel_prev_pos_;
    double rear_left_wheel_prev_pos_;
    double rear_right_wheel_prev_pos_;
    rclcpp::Time prev_time_;

    double x_, y_, theta_;

    nav_msgs::msg::Odometry odom_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;

    // Random noise members — initialized once, kept alive across callbacks
    std::default_random_engine noise_generator_;
    std::normal_distribution<double> front_left_encoder_noise_;
    std::normal_distribution<double> front_right_encoder_noise_;
    std::normal_distribution<double> rear_left_encoder_noise_;
    std::normal_distribution<double> rear_right_encoder_noise_;
};

#endif