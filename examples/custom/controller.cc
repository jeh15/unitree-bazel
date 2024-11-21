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
constexpr uint8_t num_motors = 12;

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

struct MotorState {
    std::array<float, num_motors> q = { 0 };
    std::array<float, num_motors> qd = { 0 };
};

class MotorController {
public:
    explicit MotorController() {}
    ~MotorController() {}
    void init();
    void update_command(MotorCommand& motor_cmd);
    void get_state(const std::string& motor_name);

private:
    void init_cmd_msg();
    void robot_state_msg_handler(const void* messages);
    void control_loop();
    MotorCommand motor_commands;
    MotorState motor_states;

private:
    const struct {
        std::array<float, num_motors> q_lb = {
            -1.0472, -1.5708, -2.7227,
            -1.0472, -1.5708, -2.7227,
            -1.0472, -0.5236, -2.7227,
            -1.0472, -0.5236, -2.7227
        };
        std::array<float, num_motors> q_ub = {
            1.0472, 3.4907, -0.83776,
            1.0472, 3.4907, -0.83776,
            1.0472, 4.5379, -0.83776,
            1.0472, 4.5379, -0.83776,
        };
        std::array<float, num_motors> qd_lb = {
            -0.5, -0.5, -0.5,
            -0.5, -0.5, -0.5,
            -0.5, -0.5, -0.5,
            -0.5, -0.5, -0.5
        };
        std::array<float, num_motors> qd_ub = {
            0.5, 0.5, 0.5,
            0.5, 0.5, 0.5,
            0.5, 0.5, 0.5,
            0.5, 0.5, 0.5
        };
        std::array<float, num_motors> tau_lb = {
            -5.0, -5.0, -5.0,
            -5.0, -5.0, -5.0,
            -5.0, -5.0, -5.0,
            -5.0, -5.0, -5.0
        };
        std::array<float, num_motors> tau_ub = {
            5.0, 5.0, 5.0,
            5.0, 5.0, 5.0,
            5.0, 5.0, 5.0,
            5.0, 5.0, 5.0
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
}

void MotorController::control_loop() {
    // Iterate over motors:
    for(const auto& [key, value] : MotorID) {
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

void MotorController::update_command(MotorCommand& motor_cmd) {
    // Iterate over motors and update motor command:
    for(const auto& [key, value] : MotorID) {
        float q_setpoint = std::clamp(motor_cmd.q_setpoint[value], motor_bounds.q_lb[value], motor_bounds.q_ub[value]);
        float qd_setpoint = std::clamp(motor_cmd.qd_setpoint[value], motor_bounds.qd_lb[value], motor_bounds.qd_ub[value]);
        float torque_feedforward = std::clamp(motor_cmd.torque_feedforward[value], motor_bounds.tau_lb[value], motor_bounds.tau_ub[value]);
        float stiffness = std::clamp(motor_cmd.stiffness[value], 0.0f, 20.0f);
        float damping = std::clamp(motor_cmd.damping[value], 0.0f, 10.0f);
        float kp = std::clamp(motor_cmd.kp[value], 0.0f, 10.0f);
        float kd = std::clamp(motor_cmd.kd[value], 0.0f, 10.0f);
        motor_commands.q_setpoint[value] = q_setpoint;
        motor_commands.qd_setpoint[value] = qd_setpoint; 
        motor_commands.torque_feedforward[value] = torque_feedforward;
        motor_commands.stiffness[value] = stiffness;
        motor_commands.damping[value] = damping;
        motor_commands.kp[value] = kp;
        motor_commands.kd[value] = kd;
    }
}

void MotorController::get_state(const std::string& motor_name) {
    auto motor_position = robot_state.motor_state()[MotorID[motor_name]].q();
    std::cout << motor_name << " : " << motor_position << std::endl;
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

    // Set Motor Command:
    motor_command.stiffness = {
        60.0, 60.0, 60.0,
        60.0, 60.0, 60.0,
        60.0, 60.0, 60.0,
        60.0, 60.0, 60.0
    };
    motor_command.damping = {
        5.0, 5.0, 5.0,
        5.0, 5.0, 5.0,
        5.0, 5.0, 5.0,
        5.0, 5.0, 5.0
    };
    motor_command.kp = {
        10.0, 10.0, 10.0,
        10.0, 10.0, 10.0,
        10.0, 10.0, 10.0,
        10.0, 10.0, 10.0
    };
    motor_command.kd = {
        2.0, 2.0, 2.0,
        2.0, 2.0, 2.0,
        2.0, 2.0, 2.0,
        2.0, 2.0, 2.0
    };

    // Update Controller Setpoints and Values:
    controller.update_command(motor_command);

    while (true) {
        auto elapsed_time = std::chrono::high_resolution_clock::now() - start;
        // Send MotorController Command:

        // Get state:
        std::string key = "Front_Right_Abduction";
        controller.get_state(key);

        // Sleep for dt:
        std::this_thread::sleep_for(std::chrono::milliseconds(dt));

        // Run for 10 Seconds:
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed_time).count() > 60.0)
            break;
        
    }

    return 0;
}
