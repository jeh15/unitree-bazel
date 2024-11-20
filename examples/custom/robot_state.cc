#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <chrono>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

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

class RobotState
{
public:
    explicit RobotState()
    {}

    ~RobotState()
    {}

    void Init();

private:
    void LowStateMessageHandler(const void* messages);

private:
    float dt = 0.002; // 0.001~0.01
    unitree_go::msg::dds_::LowState_ low_state{};  // default init

    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
};

void RobotState::Init() {
    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&RobotState::LowStateMessageHandler, this, std::placeholders::_1), 1);
}

void RobotState::LowStateMessageHandler(const void* message) {
    unitree_go::msg::dds_::LowState_* low_state_msg = (unitree_go::msg::dds_::LowState_*)message;
    
    // Iterate over map:
    // for(const auto& [key, value] : MotorID) {
    //     auto motor_position = low_state_msg->motor_state()[value].q();
    //     std::cout << key << ":" << motor_position << std::endl;
    // }

    // Look at left front hip motor:
    std::string key = "Front_Right_Hip";
    auto motor_position = low_state_msg->motor_state()[MotorID[key]].q();
    std::cout << key << " Motor Position: " << motor_position << std::endl;
    
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    RobotState state_handler;
    state_handler.Init();

    auto start = std::chrono::high_resolution_clock::now();

    while (true) {
        // Read Robot State and Print to Console:

        // Run for 10 Seconds:
        auto elapsed_time = std::chrono::high_resolution_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed_time).count() > 10.0) {
            break;
        }
    }

    return 0;
}
