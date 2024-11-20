#include <iostream>
#include <stdio.h>
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

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

std::map<std::string, uint8_t> MotorID = {
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

struct MotorCommand {
    float q_setpoint[12] = {
        0.0, 0.9, -1.8,
        0.0, 0.9, -1.8,
        0.0, 0.9, -1.8,
        0.0, 0.9, -1.8
    };
    float qd_setpoint[12] = { 0 };
    float torque_feedforward[12] = { 0 };
};

struct MotorState {
    float q[12] = { 0 };
    float qd[12] = { 0 };
};

class MotorController {
public:
    explicit MotorController() {}
    ~MotorController() {}
    void init();
    void update_command(MotorCommand& motor_cmd);

private:
    void init_cmd_msg();
    void robot_state_msg_handler(const void* messages);
    void control_loop();
    MotorCommand motor_commands;
    MotorState motor_states;

private:
    const struct {
        float q_lb[12] = {
            -1.0472, -1.5708, -2.7227,
            -1.0472, -1.5708, -2.7227,
            -1.0472, -0.5236, -2.7227,
            -1.0472, -0.5236, -2.7227
        };
        float q_ub[12] = {
            1.0472, 3.4907, -0.83776,
            1.0472, 3.4907, -0.83776,
            1.0472, 4.5379, -0.83776,
            1.0472, 4.5379, -0.83776,
        };
        float qd_lb[12] = {
            -0.5, -0.5, -0.5,
            -0.5, -0.5, -0.5,
            -0.5, -0.5, -0.5,
            -0.5, -0.5, -0.5
        };
        float qd_ub[12] = {
            0.5, 0.5, 0.5,
            0.5, 0.5, 0.5,
            0.5, 0.5, 0.5,
            0.5, 0.5, 0.5
        };
        float tau_lb[12] = {
            -5.0, -5.0, -5.0,
            -5.0, -5.0, -5.0,
            -5.0, -5.0, -5.0,
            -5.0, -5.0, -5.0
        };
        float tau_ub[12] = {
            5.0, 5.0, 5.0,
            5.0, 5.0, 5.0,
            5.0, 5.0, 5.0,
            5.0, 5.0, 5.0
        };
    } motor_bounds;
    std::string target = "Front_Right_Abduction";
    float kp[12] = {
        5.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };
    float kd[12] = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };

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

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
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
        motor_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        motor_cmd.motor_cmd()[i].q() = (PosStopF);
        motor_cmd.motor_cmd()[i].kp() = (0);
        motor_cmd.motor_cmd()[i].dq() = (VelStopF);
        motor_cmd.motor_cmd()[i].kd() = (0);
        motor_cmd.motor_cmd()[i].tau() = (0);
    }
}

void MotorController::robot_state_msg_handler(const void* message) {
    robot_state = *(unitree_go::msg::dds_::LowState_*)message;

    // // Iterate over motors:
    // for(const auto& [key, value] : MotorID) {
    //     motor_states.q[value] = low_state_msg->motor_state()[value].q();
    //     motor_states.qd[value] = low_state_msg->motor_state()[value].dq();
    // }

    // Look at target motor:
    // auto motor_position = low_state_msg->motor_state()[MotorID[target]].q();
    // std::cout << target << " Motor Position: " << motor_position << std::endl;
}

void MotorController::control_loop() {
    // Iterate over motors:
    for(const auto& [key, value] : MotorID) {
        float q = robot_state.motor_state()[value].q();
        float qd = robot_state.motor_state()[value].dq();
        float q_error = motor_commands.q_setpoint[value] - q;
        float qd_error = motor_commands.qd_setpoint[value] - qd;
        float torque_cmd = motor_commands.torque_feedforward[value] + kp[value] * (q_error) + kd[value] * (qd_error);
        float saturated_cmd = std::clamp(torque_cmd, motor_bounds.tau_lb[value], motor_bounds.tau_ub[value]);
        if (value == MotorID[target])
            std::cout << qd << std::endl;
        motor_cmd.motor_cmd()[value].q() = 0.0;
        motor_cmd.motor_cmd()[value].dq() = 0.0;
        motor_cmd.motor_cmd()[value].kp() = 0.0;
        motor_cmd.motor_cmd()[value].kd() = 0.0;
        motor_cmd.motor_cmd()[value].tau() = saturated_cmd ;
    }

    // Checksum:
    motor_cmd.crc() = crc32_core((uint32_t *)&motor_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
    // Publish Command:
    motor_cmd_publisher->Write(motor_cmd);
}

void MotorController::update_command(MotorCommand& motor_cmd) {
    // Iterate over motors and update motor command:
    for(const auto& [key, value] : MotorID) {
        float q_setpoint = std::clamp(motor_cmd.q_setpoint[value], motor_bounds.q_lb[value], motor_bounds.q_ub[value]);
        float qd_setpoint = std::clamp(motor_cmd.qd_setpoint[value], motor_bounds.qd_lb[value], motor_bounds.qd_ub[value]);
        float torque_feedforward = std::clamp(motor_cmd.torque_feedforward[value], motor_bounds.tau_lb[value], motor_bounds.tau_ub[value]);
        motor_commands.q_setpoint[value] = q_setpoint;
        motor_commands.qd_setpoint[value] = qd_setpoint; 
        motor_commands.torque_feedforward[value] = torque_feedforward; 
    }
}

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    MotorController controller;
    controller.init();

    auto start = std::chrono::high_resolution_clock::now();

    // Control Rate:
    const uint8_t dt = 20;
    MotorCommand motor_command;

    while (true) {
        auto elapsed_time = std::chrono::high_resolution_clock::now() - start;
        // Send MotorController Command:
        // controller.update_command(motor_command);

        // Sleep for dt:
        std::this_thread::sleep_for(std::chrono::milliseconds(dt));

        // Run for 10 Seconds:
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed_time).count() > 10.0) {
            break;
        }
    }

    return 0;
}
