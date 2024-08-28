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
#include "half.hpp"  

namespace arm_encoder 
{
    class Arm_encoder: public rclcpp::Node
    {
        public:

            Arm_encoder();
            ~Arm_encoder();

private:

    void callback(const sensor_msgs::msg::JointState::SharedPtr message);
    void send();
    void publisher();
    float convertFloat16ToFloat32(uint8_t byte1, uint8_t byte2);

private:

    std::array<float, 7> velocities;
    int heartbeat;

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;
    rclcpp::TimerBase::SharedPtr timer;
    };

}