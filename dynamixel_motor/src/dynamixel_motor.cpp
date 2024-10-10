#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "dynamixel_motor/motor_object.h"

#include "ringbot_interface/msg/motor_data.hpp"

#define umap std::unordered_map
#define PUB_RATE 5 // ms

using ringbot_interface::msg::MotorData;
using std::vector;
using std::string;
using std::unique_ptr;

class DynamixelMotor : public rclcpp::Node
{
public:
    DynamixelMotor() : Node("dynamixel_motor")
    {
        motor_init();

        command_sub_ = this->create_subscription<MotorData>("motor_command", 10,
            std::bind(&DynamixelMotor::command_callback, this, std::placeholders::_1));
        state_pub_ = this->create_publisher<MotorData>("motor_state", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_RATE),
            std::bind(&DynamixelMotor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Dynamixel Motor Initialized");
    }

private:
    rclcpp::Subscription<MotorData>::SharedPtr command_sub_;
    rclcpp::Publisher<MotorData>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    umap<std::string, unique_ptr<MotorObject>> motors_;

    void command_callback(const MotorData::SharedPtr msg)
    {
        int count = msg->id.size();
        for (auto i = 0; i < count; i++)
        {
            std::string id = msg->id[i];
            auto iter = motors_.find(id);
            if (iter == motors_.end()) continue;

            auto& motor = iter->second;
            motor->set_goal(msg->pos[i], msg->vel[i]);
        }
    }

    void timer_callback()
    {
        MotorData msg{};
        for (auto& [id, motor] : motors_)
        {
            auto [pos, vel] = motor->get_state();
            msg.id.push_back(id);
            msg.pos.push_back(pos);
            msg.vel.push_back(vel);
        }
        state_pub_->publish(msg);
    }

    void motor_init()
    {
        vector<bool> motor_enables{};
        motor_enables = this->declare_parameter("motor.enables", motor_enables);
        int motor_count = motor_enables.size();

        vector<string> motor_rids{};
        motor_rids = this->declare_parameter("motor.rids", motor_rids);
        vector<int64_t> motor_hids{};
        motor_hids = this->declare_parameter("motor.hids", motor_hids);

        for (int i = 0; i < motor_count; i++)
        {
            if (!motor_enables[i]) continue;

            string rid = motor_rids[i];
            int hid = motor_hids[i];
            motors_[rid] = std::make_unique<MotorObject>(rid, hid);
        }

        for (auto& [id, motor] : motors_)
        {
            motor->print_info();
        }
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