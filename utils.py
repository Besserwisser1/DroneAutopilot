from dataclasses import dataclass


@dataclass
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
        return BB((arr[0][0] + arr[0][2]) / 2, (arr[0][1] + arr[0][3]) / 2, abs(arr[0][0] - arr[0][2]),
                  abs(arr[0][1] - arr[0][3]), arr[1], arr[2])

    @staticmethod
    def from_x1x2y1y2_array_of_arrays(arr):
        res = []
        for inner_arr in arr:
            res.append(BB.from_x1x2y1y2_array(inner_arr))
        return res


@dataclass
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


@dataclass
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
