import AirSimMS.PythonClient.multirotor.setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

import time
import math
import random

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

imu_data = client.getImuData()
s = pprint.pformat(imu_data)
print("imu_data: %s" % s)

barometer_data = client.getBarometerData()
s = pprint.pformat(barometer_data)
print("barometer_data: %s" % s)

magnetometer_data = client.getMagnetometerData()
s = pprint.pformat(magnetometer_data)
print("magnetometer_data: %s" % s)

gps_data = client.getGpsData()
s = pprint.pformat(gps_data)
print("gps_data: %s" % s)

airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

def moveByRollPitchYawThrottleAsync(roll, pitch, yaw, throttle):
    client.moveByRollPitchYawThrottleAsync(roll, pitch, yaw, throttle, 0.1).join()
    
def get_imu():
    state = client.getMultirotorState()
    roll = state.rc_data.roll
    pitch = state.rc_data.pitch
    yaw = state.rc_data.yaw
    barometer_data = client.getBarometerData()
    altitude = barometer_data.altitude
    print(f"Altitude: {altitude}")

    position = state.kinematics_estimated.position
    x = position.x_val
    y = position.y_val
    z = position.z_val
    print(f"Drone coordinates X: {x}, Y: {y}, Z: {z}")
    return roll, pitch, yaw, altitude, x, y, z


class BB:
    '''bounding box'''
    def __init__(self, x, y, w, h, class_index, confidence):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.class_index = class_index
        self.confidence = confidence

    @staticmethod
    def from_x1x2y1y2_array(arr):
        return BB((arr[0][0]+arr[0][2])/2, (arr[0][1]+arr[0][3])/2, abs(arr[0][0]-arr[0][2]), abs(arr[0][1]-arr[0][3]), arr[1], arr[2])

    @staticmethod
    def from_x1x2y1y2_array_of_arrays(arr):
        res = []
        for inner_arr in arr:
            res.append(BB.from_x1x2y1y2_array(inner_arr))
        return res

class MavLink:
    '''              MavLink'''
    def __init__(self, roll, pitch, yaw, h):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.h = h

    @staticmethod
    def from_array(arr):
        return MavLink(arr[0], arr[1], arr[2], arr[3])

class SBUS:
    '''              SBUS'''
    def __init__(self, throttle, roll, pitch, yaw):
        self.throttle = throttle
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    @staticmethod
    def from_array(arr):
        return SBUS(arr[0], arr[1], arr[2], arr[3])

class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

class StaticAutopilot:
    def __init__(self, flight_altitude = 123, h_error = 1.0, delta_throttle = 0.5, SBUS_min = 173, SBUS_mid = 1110, SBUS_max = 2047, max_angular_velocity = 360):
        self.flight_altitude = flight_altitude
        self.h_error = abs(h_error)
        self.min_h = self.flight_altitude - self.h_error
        self.max_h = self.flight_altitude + self.h_error
        self.delta_throttle = delta_throttle
        self.SBUS_min = SBUS_min
        self.SBUS_mid = SBUS_mid
        self.SBUS_max = SBUS_max
        self.SBUS_range = min(self.SBUS_max - self.SBUS_mid, self.SBUS_mid - self.SBUS_min)
        self.max_angular_velocity = max_angular_velocity
        self.throttle = 0.595
        self.pid_altitude = PID(Kp=0.5, Ki=0.01, Kd=0.05)
        self.pid_roll = PID(Kp=0.05, Ki=0.01, Kd=0.08)
        self.pid_pitch = PID(Kp=0.05, Ki=0.01, Kd=0.05)
        self.pid_yaw = PID(Kp=0.05, Ki=0.01, Kd=0.05)
        self.start_time = time.time()
        self.start_x = None
        self.start_y = None
        self.start_z = None

    def fly(self, telemetry, get_imu):
        error_h = self.flight_altitude - telemetry.h
        self.throttle += self.pid_altitude.update(error_h, 0.1)
        self.throttle = min(max(self.throttle, 0.59), 0.595)  

        if self.start_x is None:
            _, _, _, _, self.start_x, self.start_y, self.start_z = get_imu()

        _, _, _, _, x, y, z = get_imu()

        error_x = self.start_x - x
        error_y = self.start_y - y
        error_z = self.start_z - z

        throttle = self.throttle

        roll = self.SBUS_mid - self.pid_roll.update(error_y, 0.1) / math.pi * 180 / self.max_angular_velocity * self.SBUS_range
        pitch = self.SBUS_mid - self.pid_pitch.update(error_x, 0.1) / math.pi * 180 / self.max_angular_velocity * self.SBUS_range
        yaw = self.SBUS_mid - self.pid_yaw.update(error_z, 0.1) / math.pi * 180 / self.max_angular_velocity * self.SBUS_range
        print(f"throttle: {throttle}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")
        return SBUS(throttle, roll, pitch, yaw)

def generateWindDeviation():
    roll_deviation = random.uniform(-0.5, 0.5)
    pitch_deviation = random.uniform(-0.1, 0.1)
    yaw_deviation = random.uniform(-0.1, 0.1)
    return roll_deviation, pitch_deviation, yaw_deviation

autopilot = StaticAutopilot(flight_altitude = 122, h_error = 0.1, delta_throttle = 0.1, SBUS_min = -1, SBUS_mid = 0, SBUS_max = 1)

while True:
    roll, pitch, yaw, altitude, x, y, z = get_imu()

    roll_deviation, pitch_deviation, yaw_deviation = generateWindDeviation()

    sbus_command = autopilot.fly(MavLink(roll + roll_deviation, pitch + pitch_deviation, yaw + yaw_deviation, altitude), get_imu)

    moveByRollPitchYawThrottleAsync(sbus_command.roll, sbus_command.pitch, sbus_command.yaw, sbus_command.throttle)

    time.sleep(0.05)