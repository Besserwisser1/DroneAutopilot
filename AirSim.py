import sys
sys.path.append("..")

import AirSimMS.PythonClient.multirotor.setup_path
import airsim
import numpy as np
import pprint
import time
import random
from utils import MavLink, PID

class AirSimInitializer(airsim.MultirotorClient):
    def __init__(self):
        airsim.MultirotorClient.__init__(self)
        self.confirmConnection()
        self.enableApiControl(True)

        state = self.getMultirotorState()
        print("state: %s" % pprint.pformat(state))

        imu_data = self.getImuData()
        print("imu_data: %s" % pprint.pformat(imu_data))

        barometer_data = self.getBarometerData()
        print("barometer_data: %s" % pprint.pformat(barometer_data))

        magnetometer_data = self.getMagnetometerData()
        print("magnetometer_data: %s" % pprint.pformat(magnetometer_data))

        gps_data = self.getGpsData()
        print("gps_data: %s" % pprint.pformat(gps_data))

        airsim.wait_key('Press any key to takeoff')
        print("Taking off...")
        self.armDisarm(True)
        self.takeoffAsync().join()

class DroneController(AirSimInitializer):
    def __init__(self):
        super().__init__()
        # self.autopilot = StaticAutopilot(flight_altitude, h_error, delta_throttle, SBUS_min, SBUS_mid, SBUS_max)

    def get_imu(self):
        state = self.getMultirotorState()
        roll = state.rc_data.roll
        pitch = state.rc_data.pitch
        yaw = state.rc_data.yaw
        barometer_data = self.getBarometerData()
        altitude = barometer_data.altitude
        print(f"Altitude: {altitude}")

        position = state.kinematics_estimated.position
        x = position.x_val
        y = position.y_val
        z = position.z_val
        print(f"Drone coordinates X: {x}, Y: {y}, Z: {z}")
        return roll, pitch, yaw, altitude, x, y, z

    def moveByRollPitchYawThrottleAsync(self, roll, pitch, yaw, throttle):
        super().moveByRollPitchYawThrottleAsync(roll, pitch, yaw, throttle, 0.1).join()

    def generateWindDeviation(self):
        roll_deviation = random.uniform(-0.5, 0.5)
        pitch_deviation = random.uniform(-0.1, 0.1)
        yaw_deviation = random.uniform(-0.1, 0.1)
        return roll_deviation, pitch_deviation, yaw_deviation


if __name__ == "__main__":
    try:
        from autopilot import StaticAutopilot
    except ImportError:
        print("No autopilot class found")
        exit(1)
    autopilot = StaticAutopilot(flight_altitude=122, h_error=0.1, delta_throttle=0.1, SBUS_min=-1, SBUS_mid=0, SBUS_max=1)
    controller = DroneController()

    while True:
        roll, pitch, yaw, altitude, x, y, z = controller.get_imu()
        roll_deviation, pitch_deviation, yaw_deviation = controller.generateWindDeviation()
        sbus_command = autopilot.fly(MavLink(roll + roll_deviation, pitch + pitch_deviation, yaw + yaw_deviation, altitude), controller.get_imu)
        controller.moveByRollPitchYawThrottleAsync(sbus_command.roll, sbus_command.pitch, sbus_command.yaw, sbus_command.throttle)
        time.sleep(0.05)
