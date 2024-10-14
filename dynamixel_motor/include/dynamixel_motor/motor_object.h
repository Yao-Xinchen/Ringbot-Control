#ifndef MOTOR_OBJECT_H
#define MOTOR_OBJECT_H

#include <string>
#include <tuple>
#include <thread>
#include <iostream>
#include <memory>

#include "dynamixel_workbench/dynamixel_workbench.h"

class MotorObject
{
public:
    MotorObject(std::string rid, int hid, std::string mode);

    static void init(std::string port_name, uint32_t baud_rate);

    void set_goal(double pos, double vel);

    [[nodiscard]] std::tuple<double, double> get_state() const;

    void print_info() const;

private:
    static std::unique_ptr<DynamixelWorkbench> dxl_wb;

    std::string rid; // ROS ID
    int hid; // Hardware ID

    std::string mode;
    float goal_pos;
    float goal_vel;

    float fb_pos;
    float fb_vel;

    std::thread tx_thread;

    void tx_loop();
    
    void rx_loop();
};

#endif // MOTOR_OBJECT_H