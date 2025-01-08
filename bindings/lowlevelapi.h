#pragma once

#include <iostream>
#include <stdint.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <algorithm>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include "lowlevelapi_types.h"

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"


class MotorController {
public:
    explicit MotorController() {}
    ~MotorController() {}
    void init();
    void update_command(lowleveltypes::MotorCommand& motor_cmd);
    /* Add back for debug purposes? */
    // void get_state(const std::string& motor_name);
    lowleveltypes::LowState get_low_state();
    lowleveltypes::IMUState get_imu_state();
    lowleveltypes::MotorState get_motor_state();
 
private:
    void init_cmd_msg();
    void robot_state_msg_handler(const void* messages);
    void control_loop();
    const double PosStopF = (2.146E+9f);
    const double VelStopF = (16000.0f);
    lowleveltypes::MotorCommand motor_commands;

private:
    const struct {
        std::array<float, lowleveltypes::num_motors> q_lb = {
            -1.0472, -1.5708, -2.7227,
            -1.0472, -1.5708, -2.7227,
            -1.0472, -0.5236, -2.7227,
            -1.0472, -0.5236, -2.7227
        };
        std::array<float, lowleveltypes::num_motors> q_ub = {
            1.0472, 3.4907, -0.83776,
            1.0472, 3.4907, -0.83776,
            1.0472, 4.5379, -0.83776,
            1.0472, 4.5379, -0.83776,
        };
        std::array<float, lowleveltypes::num_motors> qd_lb = {
            -10.0, -10.0, -10.0,
            -10.0, -10.0, -10.0,
            -10.0, -10.0, -10.0,
            -10.0, -10.0, -10.0
        };
        std::array<float, lowleveltypes::num_motors> qd_ub = {
            10.0, 10.0, 10.0,
            10.0, 10.0, 10.0,
            10.0, 10.0, 10.0,
            10.0, 10.0, 10.0
        };
        std::array<float, lowleveltypes::num_motors> tau_lb = {
            -23.7, -23.7, -45.3,
            -23.7, -23.7, -45.3,
            -23.7, -23.7, -45.3,
            -23.7, -23.7, -45.3
        };
        std::array<float, lowleveltypes::num_motors> tau_ub = {
            23.7, 23.7, 45.3,
            23.7, 23.7, 45.3,
            23.7, 23.7, 45.3,
            23.7, 23.7, 45.3
        };
    } motor_bounds;

    unitree_go::msg::dds_::LowCmd_ motor_cmd{};
    unitree_go::msg::dds_::LowState_ robot_state{};

    // Publisher:
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> motor_cmd_publisher;
    
    // Subscriber:
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> robot_state_subscriber;

    // LowCmd write Thread:
    ThreadPtr lcmd_thread_ptr;
};

// Checksum:
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void MotorController::init() {
    // Initialization Command Message:
    init_cmd_msg();

    // Initialize Publisher:
    motor_cmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    motor_cmd_publisher->InitChannel();

    /*create subscriber*/
    robot_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    robot_state_subscriber->InitChannel(std::bind(&MotorController::robot_state_msg_handler, this, std::placeholders::_1), 1);

    // Low Level Motor Control Loop:
    lcmd_thread_ptr = CreateRecurrentThreadEx("control_loop", UT_CPU_ID_NONE, 2000, &MotorController::control_loop, this);
}

void MotorController::init_cmd_msg() {
    motor_cmd.head()[0] = 0xFE;
    motor_cmd.head()[1] = 0xEF;
    motor_cmd.level_flag() = 0xFF;
    motor_cmd.gpio() = 0;

    for(int i=0; i<20; i++) {
        motor_cmd.motor_cmd()[i].mode() = (0x01);
        motor_cmd.motor_cmd()[i].q() = (PosStopF);
        motor_cmd.motor_cmd()[i].kp() = (0);
        motor_cmd.motor_cmd()[i].dq() = (VelStopF);
        motor_cmd.motor_cmd()[i].kd() = (0);
        motor_cmd.motor_cmd()[i].tau() = (0);
    }
}

void MotorController::robot_state_msg_handler(const void* message) {
    robot_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void MotorController::control_loop() {
    // Iterate over motors:
    for(const auto& [key, value] : lowleveltypes::MotorID) {
        float q_error = motor_commands.q_setpoint[value] - robot_state.motor_state()[value].q();
        float qd_error = motor_commands.qd_setpoint[value] - robot_state.motor_state()[value].dq();
        float torque_feedforward = std::clamp(motor_commands.torque_feedforward[value], motor_bounds.tau_lb[value], motor_bounds.tau_ub[value]);
        float torque_input = torque_feedforward + motor_commands.kp[value] * (q_error) + motor_commands.kd[value] * (qd_error);
        float torque_cmd = std::clamp(torque_input,  motor_bounds.tau_lb[value], motor_bounds.tau_ub[value]);
        motor_cmd.motor_cmd()[value].q() = motor_commands.q_setpoint[value];
        motor_cmd.motor_cmd()[value].dq() = motor_commands.qd_setpoint[value];
        motor_cmd.motor_cmd()[value].kp() = motor_commands.stiffness[value];
        motor_cmd.motor_cmd()[value].kd() = motor_commands.damping[value];
        motor_cmd.motor_cmd()[value].tau() = torque_cmd;
    }

    // Checksum:
    motor_cmd.crc() = crc32_core((uint32_t *)&motor_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);

    // Publish Command:
    motor_cmd_publisher->Write(motor_cmd);
}

void MotorController::update_command(lowleveltypes::MotorCommand& motor_cmd) {
    // Iterate over motors and update motor command:
    for(const auto& [key, value] : lowleveltypes::MotorID) {
        float q_setpoint = std::clamp(motor_cmd.q_setpoint[value], motor_bounds.q_lb[value], motor_bounds.q_ub[value]);
        float qd_setpoint = std::clamp(motor_cmd.qd_setpoint[value], motor_bounds.qd_lb[value], motor_bounds.qd_ub[value]);
        float torque_feedforward = std::clamp(motor_cmd.torque_feedforward[value], motor_bounds.tau_lb[value], motor_bounds.tau_ub[value]);
        float stiffness = std::clamp(motor_cmd.stiffness[value], 0.0f, 100.0f);
        float damping = std::clamp(motor_cmd.damping[value], 0.0f, 100.0f);
        float kp = std::clamp(motor_cmd.kp[value], 0.0f, 100.0f);
        float kd = std::clamp(motor_cmd.kd[value], 0.0f, 100.0f);
        motor_commands.q_setpoint[value] = q_setpoint;
        motor_commands.qd_setpoint[value] = qd_setpoint; 
        motor_commands.torque_feedforward[value] = torque_feedforward;
        motor_commands.stiffness[value] = stiffness;
        motor_commands.damping[value] = damping;
        motor_commands.kp[value] = kp;
        motor_commands.kd[value] = kd;
    }
}

lowleveltypes::LowState MotorController::get_low_state() {
    lowleveltypes::LowState low_state;
    for (int i = 0; i < 4; i++) {
        low_state.foot_force[i] = robot_state.foot_force()[i];
    }

    /* C++20 Required */
    // low_state.foot_force = std::to_array(robot_state.foot_force());

    return low_state;
}

lowleveltypes::IMUState MotorController::get_imu_state() {
    lowleveltypes::IMUState imu_state;

    for (int i = 0; i < 4; i++) {
        imu_state.quaternion[i] = robot_state.imu_state().quaternion()[i];
    }

    for (int i = 0; i < 3; i++) {
        imu_state.gyroscope[i] = robot_state.imu_state().gyroscope()[i];
        imu_state.accelerometer[i] = robot_state.imu_state().accelerometer()[i];
        imu_state.rpy[i] = robot_state.imu_state().rpy()[i];
    }

    /* C++20 Required */
    // imu_state.quaternion = std::to_array(robot_state.imu_state().quaternion());
    // imu_state.gyroscope = std::to_array(robot_state.imu_state().gyroscope());
    // imu_state.accelerometer = std::to_array(robot_state.imu_state().accelerometer());
    // imu_state.rpy = std::to_array(robot_state.imu_state().rpy());

    return imu_state;
}

lowleveltypes::MotorState MotorController::get_motor_state() {
    lowleveltypes::MotorState motor_state;
    for(const auto& [key, value] : lowleveltypes::MotorID) {
        motor_state.q[value] = robot_state.motor_state()[value].q();
        motor_state.qd[value] = robot_state.motor_state()[value].dq();
        motor_state.qdd[value] = robot_state.motor_state()[value].ddq();
        motor_state.torque_estimate[value] = robot_state.motor_state()[value].tau_est();
    }
    return motor_state;
}
