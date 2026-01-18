#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");

    // Declare and get parameters
    node->declare_parameter<int64_t>("a", 0);
    node->declare_parameter<int64_t>("b", 0);
    
    int64_t a = node->get_parameter("a").as_int();
    int64_t b = node->get_parameter("b").as_int();

    if (a == 0 && b == 0 && argc >= 3)
    {
        a = atoll(argv[1]);
        b = atoll(argv[2]);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requesting to add: %ld + %ld", a, b);

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
        node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[INTERRUPTED, EXITING...]");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[SERVICE NOT AVAILABLE, WAITING...]");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[SUM]: %ld", result.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[FAILED TO CALL SERVICE]");
    }

    rclcpp::shutdown();
    return 0;
}