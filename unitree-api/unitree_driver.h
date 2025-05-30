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
        explicit UnitreeDriver(const std::string network_name, int control_rate_us = 2000): network_name(network_name), control_rate_us(control_rate_us) {}
        ~UnitreeDriver() {}

        absl::Status initialize() {
            // Initialize Channel:
            ChannelFactory::Instance()->Init(0, network_name);

            // Initialization Command Message:
            init_cmd_msg();

            // Initialize Publisher:
            motor_cmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
            motor_cmd_publisher->InitChannel();

            /*create subscriber: 2nd Arg of InitChannel is queue. Make sure to set to 0 or there will be a delay.*/ 
            robot_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
            robot_state_subscriber->InitChannel(std::bind(&UnitreeDriver::robot_state_msg_handler, this, std::placeholders::_1), 0);

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

        void update_command(MotorCommand& new_command) {
            std::lock_guard<std::mutex> lock(mutex);
            // Iterate over motors and update motor command:
            for(size_t i = 0; i < num_motors; ++i) {
                motor_commands.q_setpoint[i] = std::clamp(new_command.q_setpoint[i], motor_limits.q_lb[i], motor_limits.q_ub[i]);
                motor_commands.qd_setpoint[i] = std::clamp(new_command.qd_setpoint[i], motor_limits.qd_lb[i], motor_limits.qd_ub[i]);
                motor_commands.torque_feedforward[i] = std::clamp(new_command.torque_feedforward[i], motor_limits.tau_lb[i], motor_limits.tau_ub[i]);
                motor_commands.stiffness[i] = std::clamp(new_command.stiffness[i], 0.0f, 100.0f);
                motor_commands.damping[i] = std::clamp(new_command.damping[i], 0.0f, 100.0f);
            }
        }

        LowState get_low_state() {
            LowState low_state;
            for (size_t i = 0; i < 4; ++i) {
                low_state.foot_force[i] = robot_state.foot_force()[i];
            }
            /* C++20 Required */
            // low_state.foot_force = std::to_array(robot_state.foot_force());
            return low_state;
        }

        IMUState get_imu_state() {
            IMUState imu_state;
            for (size_t i = 0; i < 4; ++i) {
                imu_state.quaternion[i] = robot_state.imu_state().quaternion()[i];
            }
            for (size_t i = 0; i < 3; ++i) {
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
            for(size_t i = 0; i < num_motors; ++i) {
                motor_state.q[i] = robot_state.motor_state()[i].q();
                motor_state.qd[i] = robot_state.motor_state()[i].dq();
                motor_state.qdd[i] = robot_state.motor_state()[i].ddq();
                motor_state.torque_estimate[i] = robot_state.motor_state()[i].tau_est();
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
        uint32_t previous_crc = 0;
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
        
        // Table Look Up CRC32 Calculation:
        uint32_t crc32_core(const uint32_t* ptr, uint32_t len) {
            // Pre-computed CRC32 table for polynomial 0x04c11db7
            static const uint32_t crc_table[256] = {
                0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005, 
                0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61, 0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 
                0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75, 
                0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd, 
                0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039, 0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 
                0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d, 
                0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95, 
                0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1, 0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 
                0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072, 
                0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca, 
                0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde, 0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 
                0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba, 
                0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692, 
                0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6, 0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 
                0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2, 
                0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a, 
                0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637, 0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb, 
                0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53, 
                0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b, 
                0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff, 0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 
                0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b, 
                0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3, 
                0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7, 0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 
                0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3, 
                0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c, 
                0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8, 0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24, 
                0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec, 
                0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654, 
                0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0, 0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 
                0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4, 
                0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c, 
                0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668, 0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
            };
            
            uint32_t crc = 0xFFFFFFFF;

            for (uint32_t i = 0; i < len; ++i) {
                uint32_t data = ptr[i];
                for (int j = 0; j < 4; ++j) {
                    uint8_t byte = (data >> (24 - j * 8)) & 0xFF;
                    crc = (crc << 8) ^ crc_table[(crc >> 24) ^ byte];
                }
            }
 
            return crc;
        }

        void init_cmd_msg() {
            motor_cmd.head()[0] = 0xFE;
            motor_cmd.head()[1] = 0xEF;
            motor_cmd.level_flag() = 0xFF;
            motor_cmd.gpio() = 0;
            for(int i=0; i<20; ++i) {
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
            size_t consecutive_overruns = 0;

            // Thread Loop:
            while(running) {
                // Calculate next execution time first
                next_time += std::chrono::microseconds(control_rate_us);

                /* Lock Guard Scope */
                {   
                    std::lock_guard<std::mutex> lock(mutex);
                    // Iterate over motors:
                    for(size_t i = 0; i < num_motors; ++i) {
                        motor_cmd.motor_cmd()[i].q() = motor_commands.q_setpoint[i];
                        motor_cmd.motor_cmd()[i].dq() = motor_commands.qd_setpoint[i];
                        motor_cmd.motor_cmd()[i].kp() = motor_commands.stiffness[i];
                        motor_cmd.motor_cmd()[i].kd() = motor_commands.damping[i];
                        motor_cmd.motor_cmd()[i].tau() = motor_commands.torque_feedforward[i];
                    }
                
                }

                // Checksum:
                 uint32_t crc = crc32_core((uint32_t *)&motor_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);

                // Publish Command if CRC has changed:
                if (crc != previous_crc) {Add commentMore actions
                    motor_cmd.crc() = crc;
                    motor_cmd_publisher->Write(motor_cmd);

                }
                previous_crc = crc;

                // Check for overrun and sleep until next execution time
                auto now = Clock::now();
                if (now < next_time) {
                    std::this_thread::sleep_until(next_time);
                    consecutive_overruns = 0;
                } 
                else {
                    // Log overrun after 10 consecutive overruns
                    consecutive_overruns++;
                    if (consecutive_overruns >= 10) {
                        auto overrun = std::chrono::duration_cast<std::chrono::microseconds>(now - next_time);
                        std::cout << "Motor Control Loop Execution Time Exceeded Control Rate: " 
                                << overrun.count() << "us" << std::endl;
                    }
                    next_time = now;
                }
            }
        }
};
