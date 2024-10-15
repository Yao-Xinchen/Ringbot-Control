#include "rclcpp/rclcpp.hpp"
#include <vector>

#include "std_msgs/msg/float64.hpp"

class Send : public rclcpp::Node
{
public:
    Send() : Node("send")
    {
        pub_ = this->create_publisher<std_msgs::msg::Float64>("forward", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&Send::timer_callback, this));
        sub_ = this->create_subscription<std_msgs::msg::Float64>("backward", 10,
            std::bind(&Send::sub_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Send Initialized");
    }

    ~Send()
    {
        double sum = 0;
        for (auto& delay : delays)
            sum += delay;
        RCLCPP_INFO(this->get_logger(), "\033[32mAverage Double Delay: %f\033[0m", sum / delays.size());
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    std::vector<double> delays;

    void timer_callback()
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = rclcpp::Clock().now().seconds();
        pub_->publish(msg);
    }

    void sub_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        auto now = rclcpp::Clock().now().seconds();
        RCLCPP_INFO(this->get_logger(), "Double Delay: %f", now - msg->data);
        delays.push_back(now - msg->data);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Send>());
    rclcpp::shutdown();
    return 0;
}