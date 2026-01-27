#include <memory>
#include <functional>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MyPublisher : public rclcpp::Node
{
public:
    MyPublisher() : Node("my_publisher"), count_{0}
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("coordinates", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MyPublisher::callback, this));
    }

private:
    void callback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "23.455, 39.394 -> " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "[COORDINATES]: '%s'", msg.data.c_str());
        publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisher>());
    rclcpp::shutdown();
    return 0;
}