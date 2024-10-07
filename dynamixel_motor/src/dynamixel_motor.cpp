#include "rclcpp/rclcpp.hpp"

#include "ringbot_interface/msg/motor_data.hpp"

class DynamixelMotor : public rclcpp::Node
{
public:
    DynamixelMotor() : Node("dynamixel_motor")
    {
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelMotor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}