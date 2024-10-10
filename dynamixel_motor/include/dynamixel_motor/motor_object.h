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
    MotorObject(std::string rid, int hid);

    void set_goal(double pos, double vel, double tor);

    [[nodiscard]] std::tuple<double, double, double> get_state() const;

    void print_info() const;

private:
    static std::unique_ptr<DynamixelWorkbench> dxl_wb;

    std::string rid; // ROS ID
    int hid; // Hardware ID

    std::thread tx_thread;

    void tx_loop();
};

#endif // MOTOR_OBJECT_H