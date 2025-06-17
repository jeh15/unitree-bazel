import time

from unitree_api_bindings import unitree_api


def main(argv=None):
    # Initialize Unitree-Api:
    network_name = "enx7cc2c647de4f"
    control_rate_ns = 2e7
    inner_control_rate_us = 2000
    control_rate_limit_us = 10000
    unitree_driver = unitree_api.UnitreeDriver(
        network_name,
        inner_control_rate_us,
        control_rate_limit_us,
    )
    unitree_driver.initialize()

    # Sleep for 1 second to allow the robot to initialize:
    time.sleep(1.0)

    # Test reading low, imu, and motor states:
    low_state = unitree_driver.get_low_state()
    imu_state = unitree_driver.get_imu_state()
    motor_state = unitree_driver.get_motor_state()

    print(f'Low State: \n Foot Force [FR FL HR HL]: {low_state.foot_force}')
    print('IMU State: ')
    print(f'Quaternion    [w x y z]: {imu_state.quaternion}')
    print(f'Gyroscope     [x y z]: {imu_state.gyroscope}')
    print(f'Accelerometer [x y z]: {imu_state.accelerometer}')
    print(f'RPY           [roll pitch yaw]: {imu_state.rpy}')

    for i in range(12):
        print(f'Motor {i} [q qd torque_estimate]: {motor_state.q[i]}, {motor_state.qd[i]}, {motor_state.torque_estimate[i]}')

    motor_commands = unitree_api.MotorCommand()
    motor_commands.q_setpoint = [0.0, 0.9, -1.8] * 4
    motor_commands.qd_setpoint = [ 0.0 ] * 12
    motor_commands.torque_feedforward = [ 0.0 ] * 12
    motor_commands.stiffness = [ 0.0 ] * 12
    motor_commands.damping = [ 0.5 ] * 12

    # Update motor commands: (Make sure to update the motor commands before starting the thread)
    unitree_driver.update_command(motor_commands)

    # Initialize Control Thread to Track Motor Commands:
    unitree_driver.initialize_thread()

    # Ramp to q_setpoint:
    num_steps = 500
    for i in range(num_steps):
        start_time = time.clock_gettime_ns(time.CLOCK_MONOTONIC)

        # Ramp Stiffness:
        motor_commands.stiffness = [ 35.0 * float(i / num_steps) ] * 12
        unitree_driver.update_command(motor_commands)

        elapsed_time = time.clock_gettime_ns(time.CLOCK_MONOTONIC) - start_time
        if elapsed_time < control_rate_ns:
            sleep_time_ns = control_rate_ns - elapsed_time
            time.sleep(sleep_time_ns / 1.0e9)
        else:
            print('Warning: Control rate exceeded.')

    # Stop Thread:
    unitree_driver.stop_thread()


if __name__ == "__main__":
    main()
