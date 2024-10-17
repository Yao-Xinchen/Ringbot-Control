#include "dynamixel_motor/motor_object.h"

#include "rclcpp/rclcpp.hpp"
#include <cmath>

std::unique_ptr<DynamixelWorkbench> MotorObject::dxl_wb = nullptr;

MotorObject::MotorObject(std::string rid, int hid, std::string mode)
{
    this->rid = rid;
    this->hid = hid;

    this->mode = mode;

    if (mode == "POS") dxl_wb->setPositionControlMode(hid);
    else if (mode == "VEL") dxl_wb->setVelocityControlMode(hid);
    else {
        RCLCPP_ERROR(rclcpp::get_logger("MotorObject"), "\033[31mInvalid mode %s\033[0m", mode.c_str()); // red
        rclcpp::shutdown();
    }

    dxl_wb->ledOn(hid);
    dxl_wb->torqueOn(hid);

    tx_thread = std::thread(&MotorObject::tx_loop, this);
}

void MotorObject::init(std::string port_name, uint32_t baud_rate)
{
    dxl_wb = std::make_unique<DynamixelWorkbench>();

    auto success = dxl_wb->begin(port_name.c_str(), baud_rate);

    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger("MotorObject"),
            "\033[32mSucceeded to open Workbench on port %s\033[0m", port_name.c_str()); // green
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotorObject"),
            "\033[31mFailed to open Workbench on port %s\033[0m", port_name.c_str()); // red
        rclcpp::shutdown();
    }
}

void MotorObject::set_goal(double pos, double vel)
{
    goal_pos = pos;
    goal_vel = vel;

    if (mode == "POS" && !std::isnan(goal_vel))
    {
        RCLCPP_WARN(rclcpp::get_logger("MotorObject"),
            "\033[33mMotor %s is in POS mode, but velocity is set\033[0m", rid.c_str()); // yellow
    }
    if (mode == "VEL" && !std::isnan(goal_pos))
    {
        RCLCPP_WARN(rclcpp::get_logger("MotorObject"),
            "\033[33mMotor %s is in VEL mode, but position is set\033[0m", rid.c_str());
    }
}

std::tuple<double, double> MotorObject::get_state() const
{
    return {fb_pos, fb_vel};
}

void MotorObject::print_info() const
{
    std::cout << "MotorObject with RID " << rid << ", HID " << hid << ", mode " << mode << std::endl;
    // set output="screen" in launch file to see this message
}

void MotorObject::tx_loop()
{
    while (rclcpp::ok())
    {
        bool success = true;
        if (mode == "POS") success = dxl_wb->goalPosition(hid, goal_vel);
        if (mode == "VEL") success = dxl_wb->goalVelocity(hid, goal_vel);

        if (!success)
        {
            RCLCPP_WARN(rclcpp::get_logger("MotorObject"), "Failed to set goal data");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(TX_RATE));
    }
}

void MotorObject::rx_loop()
{
    while (rclcpp::ok())
    {
        bool success = true;
        success &= dxl_wb->getRadian(hid, &fb_pos);
        success &= dxl_wb->getVelocity(hid, &fb_vel);

        if (!success)
        {
            RCLCPP_WARN(rclcpp::get_logger("MotorObject"), "Failed to get feedback data");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(RX_RATE));
    }
}