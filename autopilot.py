import time
import math

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
    def __init__(self, flight_altitude = 123, h_error = 1.0, delta_throttle = 0.5, SBUS_min = -1, SBUS_mid = 0, SBUS_max = 1, max_angular_velocity = 360):
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
        self.pid = PID(Kp=0.1, Ki=0.09, Kd=0.2)

    def fly(self, telemetry):
        error_h = self.flight_altitude - telemetry.h
        self.throttle += self.pid.update(error_h, 0.1)
        self.throttle = min(max(self.throttle, 0.585), 0.605)  

        throttle = self.throttle
        roll = self.SBUS_mid - telemetry.roll/math.pi*180/self.max_angular_velocity*self.SBUS_range
        pitch = self.SBUS_mid - telemetry.pitch/math.pi*180/self.max_angular_velocity*self.SBUS_range
        yaw = self.SBUS_mid - telemetry.yaw/math.pi*180/self.max_angular_velocity*self.SBUS_range
        print(f"throttle: {throttle}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")
        return SBUS(throttle, roll, pitch, yaw)