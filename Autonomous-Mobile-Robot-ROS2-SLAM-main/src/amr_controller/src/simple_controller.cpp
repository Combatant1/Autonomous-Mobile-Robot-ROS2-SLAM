#include "amr_controller/simple_controller.hpp"
#include <Eigen/Geometry>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>

SimpleController::SimpleController(const std::string &node_name)
    : Node(node_name), front_left_wheel_prev_pos_(0.0),
      front_right_wheel_prev_pos_(0.0), rear_left_wheel_prev_pos_(0.0),
      rear_right_wheel_prev_pos_(0.0), x_(0.0), y_(0.0), theta_(0.0) {
  declare_parameter("wheel_radius", 0.033);
  declare_parameter("wheel_separation", 0.239);

  wheel_radius_ = get_parameter("wheel_radius").as_double();
  wheel_separation_ = get_parameter("wheel_separation").as_double();

  RCLCPP_INFO_STREAM(get_logger(), "Wheel radius: " << wheel_radius_);
  RCLCPP_INFO_STREAM(get_logger(), "Wheel separation: " << wheel_separation_);

  transform_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/simple_velocity_controller/commands", 10);
  vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/amr_controller/cmd_vel", 10,
      std::bind(&SimpleController::velCallback, this, std::placeholders::_1));
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&SimpleController::jointCallback, this, std::placeholders::_1));

  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>("/amr_controller/odom", 10);
  prev_time_ = get_clock()->now();

  // Create the speed conversion matrix
  speed_conversion_matrix_ << wheel_radius_ / 2, wheel_radius_ / 2,
      wheel_radius_ / wheel_separation_, -wheel_radius_ / wheel_separation_;

  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_footprint";
  odom_msg_.pose.pose.orientation.x = 0.0;
  odom_msg_.pose.pose.orientation.y = 0.0;
  odom_msg_.pose.pose.orientation.z = 0.0;
  odom_msg_.pose.pose.orientation.w = 1.0;

  transform_stamped_.header.frame_id = "odom";
  transform_stamped_.child_frame_id = "base_footprint";

  RCLCPP_INFO_STREAM(get_logger(), "Speed conversion matrix:\n"
                                       << speed_conversion_matrix_);
}

void SimpleController::velCallback(
    const geometry_msgs::msg::TwistStamped &msg) {
  Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);

  Eigen::Vector2d wheel_speed =
      speed_conversion_matrix_.inverse() * robot_speed;
  std_msgs::msg::Float64MultiArray wheel_speed_msg;
  wheel_speed_msg.data.push_back(
      wheel_speed.coeff(0)); // Right wheel speed -> front_right_joint
  wheel_speed_msg.data.push_back(
      wheel_speed.coeff(1)); // Left wheel speed  -> front_left_joint
  wheel_speed_msg.data.push_back(
      wheel_speed.coeff(0)); // Right wheel speed -> rear_right_joint
  wheel_speed_msg.data.push_back(
      wheel_speed.coeff(1)); // Left wheel speed  -> rear_left_joint

  wheel_cmd_pub_->publish(wheel_speed_msg);
}

void SimpleController::jointCallback(const sensor_msgs::msg::JointState &msg) {
  // Front wheel deltas
  double dp_front_left = msg.position.at(1) - front_left_wheel_prev_pos_;
  double dp_front_right = msg.position.at(0) - front_right_wheel_prev_pos_;

  // Rear wheel deltas
  double dp_rear_left = msg.position.at(3) - rear_left_wheel_prev_pos_;
  double dp_rear_right = msg.position.at(2) - rear_right_wheel_prev_pos_;

  rclcpp::Time msg_time = msg.header.stamp;
  rclcpp::Duration dt = msg_time - prev_time_;

  // Store all previous positions
  front_left_wheel_prev_pos_ = msg.position.at(1);
  front_right_wheel_prev_pos_ = msg.position.at(0);
  rear_left_wheel_prev_pos_ = msg.position.at(3);
  rear_right_wheel_prev_pos_ = msg.position.at(2);
  prev_time_ = msg_time;

  // Average front and rear deltas per side for better odometry accuracy
  double dp_left = (dp_front_left + dp_rear_left) / 2.0;
  double dp_right = (dp_front_right + dp_rear_right) / 2.0;

  double left_wheel_speed = dp_left / dt.seconds();
  double right_wheel_speed = dp_right / dt.seconds();

  double linear_velocity =
      wheel_radius_ * (right_wheel_speed + left_wheel_speed) / 2.0;
  double angular_velocity = wheel_radius_ *
                            (right_wheel_speed - left_wheel_speed) /
                            wheel_separation_;

  double d_s = wheel_radius_ * (dp_right + dp_left) / 2.0;
  double d_theta = wheel_radius_ * (dp_right - dp_left) / wheel_separation_;
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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleController>("simple_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}