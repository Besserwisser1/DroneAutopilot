from autopilot import StaticAutopilot
from AirSim import main_class, MavLink, SBUS



if __name__ == "__main__":
    autopilot = StaticAutopilot()
    while True:
        roll, pitch, yaw, altitude = main_class.get_imu()
        sbus_command = autopilot.fly(MavLink(roll, pitch, yaw, altitude))
        main_class.moveByRollPitchYawThrottleAsync(sbus_command.roll, sbus_command.pitch, sbus_command.yaw, sbus_command.throttle)
        # time.sleep(0.1)