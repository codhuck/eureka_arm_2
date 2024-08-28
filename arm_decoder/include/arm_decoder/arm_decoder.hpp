#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <array>
#include <vector>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string> 
#include <cstring>

namespace arm_decoder 
{
    class Arm_decoder: public rclcpp::Node
    {
        public:

            Arm_decoder();
            ~Arm_decoder();

private:

    void callback(const std_msgs::msg::UInt8MultiArray::SharedPtr arr);
    void publisher();
    float convertFloat16ToFloat32(uint8_t byte1, uint8_t byte2);

private:

    std::array<double, 7> velocities;
    std::array<double, 7> efforts;
    std::array<double, 7> positions;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub;
    rclcpp::TimerBase::SharedPtr timer;
    };

}