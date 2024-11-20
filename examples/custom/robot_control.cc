#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <chrono>
#include <thread>

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

class Robot {
public:
    explicit Robot() {}
    ~Robot() {}
    void Init();
    void Command();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);

private:
    float dt = 0.002;
    float q_desired[12] = {
        0.0, 1.1, -1.8,
        0.0, 0.9, -1.8,
        0.0, 0.9, -1.8,
        0.0, 0.9, -1.8
    };
    float qd_desired[12] = {0};
    float kp[12] = {
        0.0, 15.0, 0.0,
        0.0, 5.0, 0.0,
        0.0, 5.0, 0.0,
        0.0, 5.0, 0.0
    };
    float kd[12] = {
        0.0, 1.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 1.0, 0.0
    };

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    // Publisher:
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    
    // Subscriber:
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    // LowCmd write Thread:
    ThreadPtr lowCmdWriteThreadPtr;
};

// Required Checksum:
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

void Robot::Init() {
    // Send Initialization Command:
    InitLowCmd();

    // Initialize Publisher:
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Robot::LowStateMessageHandler, this, std::placeholders::_1), 1);
}

void Robot::InitLowCmd() {
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++) {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Robot::LowStateMessageHandler(const void* message) {
    unitree_go::msg::dds_::LowState_* low_state_msg = (unitree_go::msg::dds_::LowState_*)message;

    // Look at left front hip motor:
    std::string key = "Front_Right_Hip";
    auto motor_position = low_state_msg->motor_state()[MotorID[key]].q();
    std::cout << key << " Motor Position: " << motor_position << std::endl;   
}

void Robot::Command() {
    std::string motor_ids[3] = {
        "Front_Right_Abduction",
        "Front_Right_Hip",
        "Front_Right_Knee"
    };

    for(const auto &motor_id: motor_ids) {
        auto id = MotorID[motor_id];
        low_cmd.motor_cmd()[id].q() = q_desired[id];
        low_cmd.motor_cmd()[id].dq() = qd_desired[id];
        low_cmd.motor_cmd()[id].kp() = kp[id];
        low_cmd.motor_cmd()[id].kd() = kd[id];
        low_cmd.motor_cmd()[id].tau() = 0;
        std::cout << q_desired[id] << std::endl;
    }

    // Checksum:
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
    // Publish Command:
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    Robot control_handler;
    control_handler.Init();

    auto start = std::chrono::high_resolution_clock::now();

    // Control Rate:
    const uint8_t dt = 20;

    while (true) {
        // Send Robot Command:
        control_handler.Command();

        // Sleep for dt:
        std::this_thread::sleep_for(std::chrono::milliseconds(dt));

        // Run for 10 Seconds:
        auto elapsed_time = std::chrono::high_resolution_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed_time).count() > 5.0) {
            break;
        }
    }

    return 0;
}
