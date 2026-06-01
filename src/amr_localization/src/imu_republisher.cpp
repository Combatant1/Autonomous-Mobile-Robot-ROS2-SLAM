#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuRepublisher : public rclcpp::Node
{
public:
  ImuRepublisher() : Node("imu_republisher_node")
  {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_ekf", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/out", 10,
      std::bind(&ImuRepublisher::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "IMU republisher started: imu/out -> imu_ekf");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu & imu)
  {
    sensor_msgs::msg::Imu new_imu = imu;
    new_imu.header.frame_id = "base_footprint_ekf";
    imu_pub_->publish(new_imu);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRepublisher>());
  rclcpp::shutdown();
  return 0;
}