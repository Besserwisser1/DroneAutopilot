import time
import math
from utils import MavLink, SBUS, PID

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