#include "dynamixel_motor/motor_object.h"

#include "rclcpp/rclcpp.hpp"
#include <cmath>

std::unique_ptr<DynamixelWorkbench> MotorObject::dxl_wb = nullptr;

MotorObject::MotorObject(std::string rid, int hid)
{
    this->rid = rid;
    this->hid = hid;

    if (dxl_wb == nullptr)
    {
        dxl_wb = std::make_unique<DynamixelWorkbench>();
        // TODO: replace "device_name" with the actual device name
        dxl_wb->init("device_name", 1000000, nullptr);
    }

    dxl_wb->ledOn(hid);
    dxl_wb->torqueOn(hid);

    tx_thread = std::thread(&MotorObject::tx_loop, this);
}

void MotorObject::set_goal(double pos, double vel)
{
    goal_pos = pos;
    goal_vel = vel;
}

std::tuple<double, double> MotorObject::get_state() const
{
    return {fb_pos, fb_vel};
}

void MotorObject::print_info() const
{
    std::cout << "MotorObject with RID: " << rid << ", HID: " << hid << std::endl;
    // set output="screen" in launch file to see this message
}

void MotorObject::tx_loop()
{
    while (rclcpp::ok())
    {
        if (!std::isnan(goal_pos))
        {
            dxl_wb->goalPosition(hid, goal_pos);
        }

        if (!std::isnan(goal_vel))
        {
            dxl_wb->goalVelocity(hid, goal_vel);
        }
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
            RCLCPP_ERROR(rclcpp::get_logger("MotorObject"), "Failed to get feedback data");
        }
    }
}