#include "dynamixel_motor/motor_object.h"

#include "rclcpp/rclcpp.hpp"

std::unique_ptr<DynamixelWorkbench> MotorObject::dxl_wb = std::make_unique<DynamixelWorkbench>();

MotorObject::MotorObject(std::string rid, int hid)
{
    this->rid = rid;
    this->hid = hid;

    tx_thread = std::thread(&MotorObject::tx_loop, this);

    dxl_wb->ledOn(hid);
    dxl_wb->torqueOn(hid);
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
    // TODO: Implement this function
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