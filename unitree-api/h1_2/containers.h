#pragma once

#include <cstdint>
#include <string>
#include <array>
#include <map>

namespace unitree::containers {
    constexpr uint8_t num_motors = 27;

    // Motor Map:
    enum MotorID {
        Left_Hip_Yaw = 0,
        Left_Hip_Pitch = 1,
        Left_Hip_Roll = 2,
        Left_Knee = 3,
        Left_Ankle_Pitch = 4,
        Left_Ankle_A = 4,
        Left_Ankle_Roll = 5,
        Left_Ankle_B = 5,
        Right_Hip_Yaw = 6,
        Right_Hip_Pitch = 7,
        Right_Hip_Roll = 8,
        Right_Knee_Pitch = 9,
        Right_Ankle_Pitch = 10,
        Right_Ankle_A = 10,
        Right_Ankle_Roll = 11,
        Right_Ankle_B = 11,
        Waist_Yaw = 12,
        Left_Shoulder_Pitch = 13,
        Left_Shoulder_Roll = 14,
        Left_Shoulder_Yaw = 15,
        Left_Elbow = 16,
        Left_Wrist_Roll = 17,
        Left_Wrist_Pitch = 18,
        Left_Wrist_Yaw = 19,
        Right_Shoulder_Pitch = 20,
        Right_Shoulder_Roll = 21,
        Right_Shoulder_Yaw = 22,
        Right_Elbow = 23,
        Right_Wrist_Roll = 24,
        Right_Wrist_Pitch = 25,
        Right_Wrist_Yaw = 26,
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
        std::array<float, num_motors> q_setpoint = { 0 };
        std::array<float, num_motors> qd_setpoint = { 0 };
        std::array<float, num_motors> torque_feedforward = { 0 };
        std::array<float, num_motors> stiffness = { 0 };
        std::array<float, num_motors> damping = { 0 };
    };

    // Motor Limit Constants:
    struct MotorLimits {
        // According to the official H1-2 XML:
        std::array<float, num_motors> q_lb = {
            -0.43, -0.43, -1.57, -0.26, -0.89, -0.26,
            -0.43, -3.14, -3.14, -0.26, -0.89, -0.26,
            -2.35,
            -3.14, -0.38, -2.66, -0.95, -3.01, -0.47, -1.27,
            -3.14, -3.4, -3.01, -0.95, -2.75, -0.47, -1.27
        };
        std::array<float, num_motors> q_ub = {
            0.43, 0.43, 1.57, 2.05, 0.52, 0.26,
            0.43, 2.5, 0.43, 2.05, 0.52, 0.26,
            2.35,
            1.57, 3.4, 3.01, 3.18, 2.75, 0.47, 1.27,
            1.57, 0.38, 2.66, 3.18, 3.01, 0.47, 1.27
        };

        std::array<float, num_motors> qd_lb = {
            -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14,
            -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14,
            -2.0 * 3.14,
            -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14,
            -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14, -2.0 * 3.14
        };
        std::array<float, num_motors> qd_ub = {
            2.0 * 3.14, 2.0 * 3.14,  2.0 * 3.14,  2.0 * 3.14,  2.0 * 3.14, 2.0 * 3.14, 
            2.0 * 3.14, 2.0 * 3.14,  2.0 * 3.14,  2.0 * 3.14,  2.0 * 3.14, 2.0 * 3.14, 
            2.0 * 3.14,
            2.0 * 3.14, 2.0 * 3.14,  2.0 * 3.14,  2.0 * 3.14,  2.0 * 3.14, 2.0 * 3.14, 2.0 * 3.14, 
            2.0 * 3.14, 2.0 * 3.14,  2.0 * 3.14,  2.0 * 3.14,  2.0 * 3.14, 2.0 * 3.14, 2.0 * 3.14
        };
        std::array<float, num_motors> tau_lb = {
            -200.0, -200.0, -200.0, 300.0, -60.0, -40.0,
            -200.0, -200.0, -200.0, 300.0, -60.0, -40.0,
            -200.0,
            -40, -40, -18, -18, -19, -19, -19,
            -40, -40, -18, -18, -19, -19, -19
        };
        std::array<float, num_motors> tau_ub = {
            200.0, 200.0, 200.0, -300.0, 60.0, 40.0,
            200.0, 200.0, 200.0, -300.0, 60.0, 40.0,
            200.0,
            40, 40, 18, 18, 19, 19, 19,
            40, 40, 18, 18, 19, 19, 19
        };
    };

}