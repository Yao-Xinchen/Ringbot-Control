#include "dynamixel_motor/motor_object.h"

std::unique_ptr<DynamixelWorkbench> MotorObject::dxl_wb = std::make_unique<DynamixelWorkbench>();

MotorObject::MotorObject(std::string rid, int hid)
{
    this->rid = rid;
    this->hid = hid;

    tx_thread = std::thread(&MotorObject::tx_loop, this);

    dxl_wb->ledOn(hid);
}

void MotorObject::set_goal(double pos, double vel, double tor)
{
    // TODO: Set goal position, velocity, and torque
}

std::tuple<double, double, double> MotorObject::get_state() const
{
    return {0, 0, 0};
}

void MotorObject::print_info() const
{
    std::cout << "MotorObject with RID: " << rid << ", HID: " << hid << std::endl;
    // set output="screen" in launch file to see this message
}

void MotorObject::tx_loop()
{
}