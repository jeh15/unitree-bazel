#pragma once

#include <iostream>
#include <stdint.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>

#include "absl/status/status.h"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

#include "unitree-api/containers.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::containers;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"


class UnitreeDriver {
    public:
        explicit UnitreeDriver(const std::string network_name, int control_rate = 1000): network_name(network_name), control_rate_us(control_rate) {}
        ~UnitreeDriver() {}

        absl::Status initialize() {
            // Initialize Channel:
            ChannelFactory::Instance()->Init(0, network_name);

            // Initialization Command Message:
            init_cmd_msg();

            // Initialize Publisher:
            motor_cmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
            motor_cmd_publisher->InitChannel();

            /*create subscriber*/
            robot_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
            robot_state_subscriber->InitChannel(std::bind(&UnitreeDriver::robot_state_msg_handler, this, std::placeholders::_1), 1);

            initialized = true;
            return absl::OkStatus();
        }

        absl::Status initialize_thread() {
            if(!initialized)
                return absl::FailedPreconditionError("Motor Controller not initialized");

            thread = std::thread(&UnitreeDriver::control_loop, this);
            thread_initialized = true;
            return absl::OkStatus();
        }

        absl::Status stop_thread() {
            if(!initialized || !thread_initialized)
                return absl::FailedPreconditionError("Motor Controller or Control Thread not initialized");

            running = false;
            thread.join();
            return absl::OkStatus();
        }

        void update_command(MotorCommand& motor_cmd) {
            std::lock_guard<std::mutex> lock(mutex);
            // Iterate over motors and update motor command:
            for(const auto& [key, value] : MotorID) {
                float q_setpoint = std::clamp(motor_cmd.q_setpoint[value], motor_limits.q_lb[value], motor_limits.q_ub[value]);
                float qd_setpoint = std::clamp(motor_cmd.qd_setpoint[value], motor_limits.qd_lb[value], motor_limits.qd_ub[value]);
                float torque_feedforward = std::clamp(motor_cmd.torque_feedforward[value], motor_limits.tau_lb[value], motor_limits.tau_ub[value]);
                float stiffness = std::clamp(motor_cmd.stiffness[value], 0.0f, 100.0f);
                float damping = std::clamp(motor_cmd.damping[value], 0.0f, 100.0f);
                motor_commands.q_setpoint[value] = q_setpoint;
                motor_commands.qd_setpoint[value] = qd_setpoint; 
                motor_commands.torque_feedforward[value] = torque_feedforward;
                motor_commands.stiffness[value] = stiffness;
                motor_commands.damping[value] = damping;
            }
        }

        LowState get_low_state() {
            LowState low_state;
            for (int i = 0; i < 4; i++) {
                low_state.foot_force[i] = robot_state.foot_force()[i];
            }
            /* C++20 Required */
            // low_state.foot_force = std::to_array(robot_state.foot_force());
            return low_state;
        }

        IMUState get_imu_state() {
            IMUState imu_state;
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

        MotorState get_motor_state() {
            MotorState motor_state;
            for(const auto& [key, value] : MotorID) {
                motor_state.q[value] = robot_state.motor_state()[value].q();
                motor_state.qd[value] = robot_state.motor_state()[value].dq();
                motor_state.qdd[value] = robot_state.motor_state()[value].ddq();
                motor_state.torque_estimate[value] = robot_state.motor_state()[value].tau_est();
            }
            return motor_state;
        }

        int get_control_rate() {
            return control_rate_us;
        }

        bool is_initialized() {
            return initialized;
        }

        bool is_thread_initialized() {
            return thread_initialized;
        }
    
    private:
        // Motor Structs:
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
                -10.0, -10.0, -10.0,
                -10.0, -10.0, -10.0,
                -10.0, -10.0, -10.0,
                -10.0, -10.0, -10.0
            };
            std::array<float, num_motors> qd_ub = {
                10.0, 10.0, 10.0,
                10.0, 10.0, 10.0,
                10.0, 10.0, 10.0,
                10.0, 10.0, 10.0
            };
            std::array<float, num_motors> tau_lb = {
                -23.7, -23.7, -45.3,
                -23.7, -23.7, -45.3,
                -23.7, -23.7, -45.3,
                -23.7, -23.7, -45.3
            };
            std::array<float, num_motors> tau_ub = {
                23.7, 23.7, 45.3,
                23.7, 23.7, 45.3,
                23.7, 23.7, 45.3,
                23.7, 23.7, 45.3
            };
        } motor_limits;
        MotorCommand motor_commands;
        // Initialization Flag:
        bool initialized = false;
        bool thread_initialized = false;
        // Unitree Constants:
        const double PosStopF = (2.146E+9f);
        const double VelStopF = (16000.0f);
        // Communication and Messages:
        std::string network_name;
        unitree_go::msg::dds_::LowCmd_ motor_cmd{};
        unitree_go::msg::dds_::LowState_ robot_state{};
        ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> motor_cmd_publisher;
        ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> robot_state_subscriber;
        // Control Thread:
        uint8_t control_rate_us;
        std::atomic<bool> running{true};
        std::mutex mutex;
        std::thread thread;

        uint32_t crc32_core(uint32_t* ptr, uint32_t len) {
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

        void init_cmd_msg() {
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

        void robot_state_msg_handler(const void* message) {
            robot_state = *(unitree_go::msg::dds_::LowState_*)message;
        }

        void control_loop() {
            using Clock = std::chrono::steady_clock;
            auto next_time = Clock::now();
            // Thread Loop:
            while(running) {
                // Calculate next execution time first
                next_time += std::chrono::microseconds(control_rate_us);

                /* Lock Guard Scope */
                {   
                    std::lock_guard<std::mutex> lock(mutex);
                    // Iterate over motors:
                    for(const auto& [key, value] : MotorID) {
                        float torque_feedforward = std::clamp(motor_commands.torque_feedforward[value], motor_limits.tau_lb[value], motor_limits.tau_ub[value]);
                        motor_cmd.motor_cmd()[value].q() = motor_commands.q_setpoint[value];
                        motor_cmd.motor_cmd()[value].dq() = motor_commands.qd_setpoint[value];
                        motor_cmd.motor_cmd()[value].kp() = motor_commands.stiffness[value];
                        motor_cmd.motor_cmd()[value].kd() = motor_commands.damping[value];
                        motor_cmd.motor_cmd()[value].tau() = torque_feedforward ;
                    }

                    // Checksum:
                    motor_cmd.crc() = crc32_core((uint32_t *)&motor_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);

                    // Publish Command:
                    motor_cmd_publisher->Write(motor_cmd);
                }
                // Check for overrun and sleep until next execution time
                auto now = Clock::now();
                if (now < next_time) {
                    std::this_thread::sleep_until(next_time);
                } 
                else {
                    // Log overrun
                    auto overrun = std::chrono::duration_cast<std::chrono::microseconds>(now - next_time);
                    std::cout << "Motor Control Loop Execution Time Exceeded Control Rate: " 
                            << overrun.count() << "us" << std::endl;
                    // Reset next execution time to prevent cascading delays
                    next_time = now;
                }
            }
        }
};
