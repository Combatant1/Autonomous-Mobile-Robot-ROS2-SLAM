#include "amr_controller/controller_wnoise.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>

#include <random>

NoisyController::NoisyController(const std::string & node_name)
    : Node(node_name)
    , front_left_wheel_prev_pos_(0.0)
    , front_right_wheel_prev_pos_(0.0)
    , rear_left_wheel_prev_pos_(0.0)
    , rear_right_wheel_prev_pos_(0.0)
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
{
    declare_parameter("wheel_radius", 0.33);
    declare_parameter("wheel_separation", 0.239);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Wheel radius: " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Wheel separation: " << wheel_separation_);

    prev_time_ = get_clock()->now();


    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&NoisyController::jointCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/amr_controller/odom_noisy", 10);

    // Create the speed conversion matrix


    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_noisy";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_noisy";

}


void NoisyController::jointCallback(const sensor_msgs::msg::JointState & msg)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> front_left_encoder_noise(0.0, 0.005);
    std::normal_distribution<double> front_right_encoder_noise(0.0, 0.005);
    double front_left_wheel_encoder = msg.position.at(1) + front_left_encoder_noise(noise_generator);
    double front_right_wheel_encoder = msg.position.at(0) + front_right_encoder_noise(noise_generator);
    double dp_front_left = front_left_wheel_encoder - front_left_wheel_prev_pos_;
    double dp_front_right = front_right_wheel_encoder - front_right_wheel_prev_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    front_left_wheel_prev_pos_ = msg.position.at(1);
    front_right_wheel_prev_pos_ = msg.position.at(0);
    rear_left_wheel_prev_pos_ = msg.position.at(3);
    rear_right_wheel_prev_pos_ = msg.position.at(2);
    prev_time_ = msg_time;

    double front_left_wheel_speed = dp_front_left / dt.seconds();
    double front_right_wheel_speed = dp_front_right / dt.seconds();

    double linear_velocity = (wheel_radius_ * front_right_wheel_speed + wheel_radius_ * front_left_wheel_speed) / 2.0;
    double angular_velocity = (wheel_radius_ * front_right_wheel_speed - wheel_radius_ * front_left_wheel_speed) / wheel_separation_;

    double d_s = (wheel_radius_ * dp_front_right + wheel_radius_ * dp_front_left) / 2.0;
    double d_theta = (wheel_radius_ * dp_front_right - wheel_radius_ * dp_front_left) / wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * std::cos(theta_);
    y_ += d_s * std::sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.twist.twist.linear.x = linear_velocity;
    odom_msg_.twist.twist.angular.z = angular_velocity;

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();
    transform_stamped_.header.stamp = get_clock()->now();

    odom_pub_->publish(odom_msg_);
    transform_broadcaster_->sendTransform(transform_stamped_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}