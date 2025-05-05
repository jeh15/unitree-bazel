import time

import unitree_api



def main(argv=None):
    # Initialize Unitree Driver:
    network_name = "enx7cc2c647de4f"
    inner_control_rate = 2000
    unitree_driver = unitree_api.UnitreeDriver(
        network_name,
        inner_control_rate,
    )
    unitree_driver.initialize()

    # Sleep to ensure the initialization is complete.
    time.sleep(1.0)

    # Test reading low, imu, and motor states:
    low_state = unitree_driver.get_low_state()
    imu_state = unitree_driver.get_imu_state()
    motor_state = unitree_driver.get_motor_state()

    print('Low State: ')
    print(f'Foot Force [FR FL HR FL]: {low_state.foot_force}')

    print('IMU State: ')
    print(f'Quaternion [w x y z]: {imu_state.quaternion}')
    print(f'Gyroscope [wx wy wz]: {imu_state.gyroscope}')
    print(f'Accelerometer [x y z]: {imu_state.accelerometer}')
    print(f'RPY [roll pitch yaw]: {imu_state.rpy}')

    print('Motor State: ')
    print(f'Joint Positions: {motor_state.q}')
    print(f'Joint Velocity: {motor_state.qd}')
    print(f'Torque Estimate: {motor_state.torque_estimate}')

    # Update Motor Command to Default Standing Position:
    motor_commands = unitree_api.MotorCommand()
    motor_commands.q_setpoint = [0.0, 0.9, -1.8] * 4
    motor_commands.qd_setpoint = [0.0, 0.0, 0.0] * 4
    motor_commands.torque_feedforward = [0.0, 0.0, 0.0] * 4
    motor_commands.stiffness = [0.0, 0.0, 0.0] * 4
    motor_commands.damping = [0.0, 0.0, 0.0] * 4
    unitree_driver.update_command(motor_commands)

    # Initialize Control Thread:
    unitree_driver.initialize_thread()

    num_steps = 1000
    sleep_time = 0.01
    for i in range(num_steps):
        ratio = i / num_steps
        motor_commands.stiffness = [60.0 * ratio] * 12
        motor_commands.damping = [5.0 * ratio] * 12

        unitree_driver.update_command(motor_commands)
        time.sleep(sleep_time)

    unitree_driver.stop_thread()


if __name__ == "__main__":
    main()
