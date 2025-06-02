#include <iostream>
#include <string>
#include <chrono>

#include "absl/status/status.h"
#include "absl/log/absl_check.h"
#include "rules_cc/cc/runfiles/runfiles.h"

#include "unitree-api/unitree_driver.h"
#include "unitree-api/containers.h"

using namespace unitree::containers;
using rules_cc::cc::runfiles::Runfiles;

int main(int argc, char** argv) {
    // Abseil Status:
    absl::Status result;

    // Initialize Unitree Driver:
    std::string network_name = "enx7cc2c647de4f";
    int control_rate_us = 2000;   // Control rate of inner control loop in microseconds.
    UnitreeDriver unitree_driver(network_name, control_rate_us);
    result.Update(unitree_driver.initialize());
    ABSL_CHECK(result.ok()) << result.message();

    // Sleep for 1 second to allow the robot to initialize:
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Test reading low, imu, and motor states:
    LowState low_state = unitree_driver.get_low_state();
    IMUState imu_state = unitree_driver.get_imu_state();
    MotorState motor_state = unitree_driver.get_motor_state();

    std::cout << "Low State: " << std::endl;
    std::cout << "Foot Force [FR FL HR HL]: " << low_state.foot_force[0] << ", " << low_state.foot_force[1] << ", " << low_state.foot_force[2] << ", " << low_state.foot_force[3] << std::endl;

    std::cout << "IMU State: " << std::endl;
    std::cout << "Quaternion [w x y z]: " << imu_state.quaternion[0] << ", " << imu_state.quaternion[1] << ", " << imu_state.quaternion[2] << ", " << imu_state.quaternion[3] << std::endl;
    std::cout << "Gyroscope [x y z]: " << imu_state.gyroscope[0] << ", " << imu_state.gyroscope[1] << ", " << imu_state.gyroscope[2] << std::endl;
    std::cout << "Accelerometer [x y z]: " << imu_state.accelerometer[0] << ", " << imu_state.accelerometer[1] << ", " << imu_state.accelerometer[2] << std::endl;
    std::cout << "RPY [roll pitch yaw]: " << imu_state.rpy[0] << ", " << imu_state.rpy[1] << ", " << imu_state.rpy[2] << std::endl;

    // Note: the acceleration reading for the Go2 is not available.
    std::cout << "Motor State: " << std::endl;
    for (const auto& [key, value] : MotorID) {
        std::cout << key << " [q qd qdd torque_estimate]: " << motor_state.q[value] << ", " << motor_state.qd[value] << ", " << motor_state.qdd[value] << ", " << motor_state.torque_estimate[value] << std::endl;
    }

    // Initialize MotorCommand for the inner control thread:
    MotorCommand motor_commands;
    motor_commands.q_setpoint = {
        0.0, 0.9, -1.8,
        0.0, 0.9, -1.8,
        0.0, 0.9, -1.8,
        0.0, 0.9, -1.8
    };
    motor_commands.qd_setpoint = { 0 };
    motor_commands.torque_feedforward = { 0 };
    motor_commands.stiffness = { 0 };
    motor_commands.damping = { 0 };

    // Update motor commands: (Make sure to update the motor commands before starting the thread)
    unitree_driver.update_command(motor_commands);

    // Initialize Control Thread:
    result.Update(unitree_driver.initialize_thread());
    ABSL_CHECK(result.ok()) << result.message();

    // Ramp stiffness and damping over 10 seconds to hold the robot positions:
    int num_steps = 1000;
    int sleep_time_ms = 10;
    for (int i = 0; i <= num_steps; i++) {
        float ratio = static_cast<float>(i) / static_cast<float>(num_steps);
        for (const auto& [key, value] : MotorID) {
            motor_commands.stiffness[value] = 60.0 * ratio;
            motor_commands.damping[value] = 5.0 * ratio;
        }
        unitree_driver.update_command(motor_commands);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
    }

    bool is_running = true;
    while (is_running) {
        unitree_driver.update_command(motor_commands);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
    }

    // Clean up and stop the thread: (Becareful as this will stop control!)
    result.Update(unitree_driver.stop_thread());
    ABSL_CHECK(result.ok()) << result.message();

    return 0;
}