import AirSimMS.PythonClient.multirotor.setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

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
    print(altitude)
    return roll, pitch, yaw, altitude

airsim.wait_key('Press any key to start')
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

class StaticAutopilot:
    """                (flight_altitude)                           (                               .                     )
    SBUS_min, SBUS_mid, SBUS_max, max_angular_velocity                            
              throttle                                    
    """
    def __init__(self, flight_altitude = 123, h_error = 1.0, delta_throttle = 0.5, SBUS_min = 173, SBUS_mid = 993, SBUS_max = 1810, max_angular_velocity = 360):
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
        self.throttle = SBUS_min

    def fly(self, telemetry):
        if telemetry.h < self.min_h:
            #self.throttle += self.delta_throttle
            self.throttle = min(1.0, self.throttle + self.delta_throttle)
        elif telemetry.h > self.max_h:
            #self.throttle -= self.delta_throttle
            self.throttle = max(0.0, self.throttle - self.delta_throttle)

        throttle = self.throttle
        roll = self.SBUS_mid - telemetry.roll/math.pi*180/self.max_angular_velocity*self.SBUS_range
        pitch = self.SBUS_mid - telemetry.pitch/math.pi*180/self.max_angular_velocity*self.SBUS_range
        yaw = self.SBUS_mid - telemetry.yaw/math.pi*180/self.max_angular_velocity*self.SBUS_range
        return SBUS(throttle, roll, pitch, yaw)
    
autopilot = StaticAutopilot()
while True:
    roll, pitch, yaw, altitude = get_imu()
        
    sbus_command = autopilot.fly(MavLink(roll, pitch, yaw, altitude))
        
    moveByRollPitchYawThrottleAsync(sbus_command.roll, sbus_command.pitch, sbus_command.yaw, sbus_command.throttle)
        
    time.sleep(0.1)


'''
airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
client.moveToPositionAsync(-10, 10, -10, 5).join()

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to take images')
# get camera images from the car
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
    airsim.ImageRequest("1", airsim.ImageType.Scene), #scene vision image in png format
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
print('Retrieved images: %d' % len(responses))

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses):

    filename = os.path.join(tmp_dir, str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

airsim.wait_key('Press any key to reset to original state')

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
'''