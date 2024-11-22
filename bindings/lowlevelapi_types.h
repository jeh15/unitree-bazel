#pragma once

#include <cstdint>
#include <string>
#include <array>
#include <map>

namespace lowleveltypes {
    constexpr uint8_t num_motors = 12;

    // Motor Map:
    const std::map<std::string, uint8_t> MotorID = {
        {"Front_Right_Abduction", 0},
        {"Front_Right_Hip", 1},
        {"Front_Right_Knee", 2},
        {"Front_Left_Abduction", 3},
        {"Front_Left_Hip", 4},
        {"Front_Left_Knee", 5},
        {"Hind_Right_Abduction", 6},
        {"Hind_Right_Hip", 7},
        {"Hind_Right_Knee", 8},
        {"Hind_Left_Abduction", 9},
        {"Hind_Left_Hip", 10},
        {"Hind_Left_Knee", 11}
    };

    // Data Structs from Robot:
    struct LowState {
        std::array<short, 4> foot_force = { 0 };
    };

    struct IMUState {
        std::array<float, 4> quaternion = { 0 };
        std::array<float, 3> gyroscope = { 0 };
        std::array<float, 3> accelerometer = { 0 };
        std::array<float, 3> rpy = { 0 };
    };

    struct MotorState {
        std::array<float, num_motors> q = { 0 };
        std::array<float, num_motors> qd = { 0 };
        std::array<float, num_motors> qdd = { 0 };
        std::array<float, num_motors> torque_estimate = { 0 };
    };

    // Data Struct to Command Robot:
    struct MotorCommand {
        std::array<float, num_motors> q_setpoint = {
            0.0, 0.9, -1.8,
            0.0, 0.9, -1.8,
            0.0, 0.9, -1.8,
            0.0, 0.9, -1.8
        };
        std::array<float, num_motors> qd_setpoint = { 0 };
        std::array<float, num_motors> torque_feedforward = { 0 };
        std::array<float, num_motors> stiffness = { 0 };
        std::array<float, num_motors> damping = { 0 };
        std::array<float, num_motors> kp = { 0 };
        std::array<float, num_motors> kd = { 0 };
    };
}