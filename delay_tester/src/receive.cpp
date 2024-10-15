#include "rclcpp/rclcpp.hpp"
#include <vector>

#include "std_msgs/msg/float64.hpp"

class Receive : public rclcpp::Node
{
public:
    Receive() : Node("receive")
    {
        sub_ = this->create_subscription<std_msgs::msg::Float64>("forward", 10,
            std::bind(&Receive::sub_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<std_msgs::msg::Float64>("backward", 10);
        RCLCPP_INFO(this->get_logger(), "Receive Initialized");
    }

    ~Receive()
    {
        double sum = 0;
        for (auto& delay : delays)
            sum += delay;
        RCLCPP_INFO(this->get_logger(), "\033[32mAverage Single Delay: %f\033[0m", sum / delays.size());
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    std::vector<double> delays;

    void sub_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pub_->publish(*msg);
        auto now = rclcpp::Clock().now().seconds();
        RCLCPP_INFO(this->get_logger(), "Single Delay: %f", now - msg->data);
        delays.push_back(now - msg->data);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Receive>());
    rclcpp::shutdown();
    return 0;
}