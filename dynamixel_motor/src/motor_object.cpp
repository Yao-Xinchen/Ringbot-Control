#include "dynamixel_motor/motor_object.h"

#include "rclcpp/rclcpp.hpp"
#include <cmath>

std::unique_ptr<DynamixelWorkbench> MotorObject::dxl_wb = nullptr;

MotorObject::MotorObject(std::string rid, int hid, std::string mode)
{
    this->rid = rid;
    this->hid = hid;

    if (mode == "POS") this->mode = POS;
    else if (mode == "VEL") this->mode = VEL;
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotorObject"), "\033[31mInvalid mode %s\033[0m", mode.c_str()); // red
        rclcpp::shutdown();
    }

    // log
    const char* log;

    // set mode
    switch (this->mode)
    {
    case POS:
        dxl_wb->setPositionControlMode(hid, &log); break;
    case VEL:
        dxl_wb->setVelocityControlMode(hid, &log); break;
    default:
        break;
    }

    dxl_wb->ledOn(hid, &log);
    // RCLCPP_INFO(rclcpp::get_logger("MotorObject"), "ledOn: %s", log);

    dxl_wb->torqueOn(hid, &log);
    // RCLCPP_INFO(rclcpp::get_logger("MotorObject"), "torqueOn: %s", log);

    tx_thread = std::thread(&MotorObject::tx_loop, this);
    rx_thread = std::thread(&MotorObject::rx_loop, this);
}

MotorObject::~MotorObject()
{
    dxl_wb->ledOff(hid);
    dxl_wb->torqueOff(hid);
    tx_thread.join();
    rx_thread.join();
}

void MotorObject::init(std::string port_name, uint32_t baud_rate)
{
    dxl_wb = std::make_unique<DynamixelWorkbench>();

    auto success = dxl_wb->init(port_name.c_str(), baud_rate);

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

    switch (mode)
    {
    case POS:
        if (std::isnan(goal_vel)) break;
        RCLCPP_WARN(rclcpp::get_logger("MotorObject"),
            "\033[33mMotor %s is in POS mode, but velocity is set\033[0m", rid.c_str()); // yellow
        break;
    case VEL:
        if (std::isnan(goal_pos)) break;
        RCLCPP_WARN(rclcpp::get_logger("MotorObject"),
            "\033[33mMotor %s is in VEL mode, but position is set\033[0m", rid.c_str());
        break;
    default:
        break;
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
        const char* log;
        bool success = true;

        switch (mode)
        {
        case POS:
            success = dxl_wb->goalPosition(hid, goal_pos, &log); break;
        case VEL:
            success = dxl_wb->goalVelocity(hid, goal_vel, &log); break;
        default:
            break;
        }

        // switch (mode)
        // {
        // case POS: {
        //     auto value = dxl_wb->convertRadian2Value(hid, goal_pos);
        //     std::cout << "pos value: " << value << std::endl;
        //     success &= dxl_wb->goalPosition(hid, value, &log);
        //     break;
        // }
        // case VEL: {
        //     auto value = dxl_wb->convertVelocity2Value(hid, goal_vel);
        //     std::cout << "vel value: " << value << std::endl;
        //     success &= dxl_wb->goalVelocity(hid, value, &log);
        //     break;
        // }
        // default:
        //     break;
        // }

        if (!success)
        {
            RCLCPP_WARN(rclcpp::get_logger("MotorObject"), "Error: %s", log);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(TX_RATE));
    }
}

void MotorObject::rx_loop()
{
    while (rclcpp::ok())
    {
        const char* log;
        bool success = true;
        success &= dxl_wb->getRadian(hid, &fb_pos, &log);
        success &= dxl_wb->getVelocity(hid, &fb_vel, &log);

        if (!success)
        {
            RCLCPP_WARN(rclcpp::get_logger("MotorObject"), "Error: %s", log);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(RX_RATE));
    }
}

void MotorObject::ping()
{
    auto success = dxl_wb->ping(hid);
    if (!success)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotorObject"), "\033[31mFailed to ping motor %s\033[0m", rid.c_str()); // red
        rclcpp::shutdown();
    }
}