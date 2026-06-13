#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <libserial/SerialPort.h>

using namespace std::chrono_literals;

class SimpleSerialReceiver : public rclcpp::Node
{
public:
    SimpleSerialReceiver() : Node("simple_serial_receiver")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        port_ = get_parameter("port").as_string();
        esp_.Open(port_);
        esp_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        pub_ = create_publisher<std_msgs::msg::String>("serial_receiver", 10);
        timer_ = create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::timerCallback, this));

        RCLCPP_INFO(get_logger(), "publishing at 100 Hz");
    }

    ~SimpleSerialReceiver()
    {
        esp_.Close();
    }

private:
    unsigned int counter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ ;
    rclcpp::TimerBase::SharedPtr timer_;
    LibSerial::SerialPort esp_;
    std::string port_;

    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        if(rclcpp::ok() && esp_.IsDataAvailable())
        {
            esp_.ReadLine(message.data);
        }
        pub_->publish(message);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSerialReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}