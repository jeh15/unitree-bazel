import unitree_api

import time
import pdb

def main(argv=None):
    # Test Bindings by reading Robot Values:
    network_name = "eno2"
    unitree = unitree_api.MotorController()
    unitree.init(network_name)

    time.sleep(1)

    # Get Low State:
    low_state = unitree.get_low_state()
    print(f"Low State - Foot Force: {low_state.foot_force}")

    # Get IMU State:
    imu_state = unitree.get_imu_state()
    print(f"IMU State - Quaternion: {imu_state.quaternion}")
    print(f"IMU State - Gyroscope: {imu_state.gyroscope}")
    print(f"IMU State - Accelerometer: {imu_state.accelerometer}")
    print(f"IMU State - RPY: {imu_state.rpy}")

    # Get Motor State:
    motor_state = unitree.get_motor_state()
    print(f"Motor State - Motor Angles: {motor_state.q}")
    print(f"Motor State - Motor Velocities: {motor_state.qd}")
    print(f"Motor State - Motor Accelertations: {motor_state.qdd}")
    print(f"Motor State - Motor Torques: {motor_state.torque_estimate}")
    

if __name__ == '__main__':
    main()
