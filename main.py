import time

from AirSim import DroneController
from autopilot import StaticAutopilot
from joystick import JoystickData
from utils import MavLink, PID


if __name__ == "__main__":
    autopilot = StaticAutopilot(flight_altitude=122, h_error=0.1, delta_throttle=0.1, SBUS_min=-1, SBUS_mid=0, SBUS_max=1)
    controller = DroneController()
    joystick = JoystickData()

    while True:
        roll_joy, pitch_joy, yaw_joy, throttle_joy, toggles_joy = joystick.run()
        throttle_joy = (throttle_joy + 0.75) / 1.75
        if toggles_joy == 1:
            roll, pitch, yaw, altitude, x, y, z = controller.get_imu()
            # roll_deviation, pitch_deviation, yaw_deviation = controller.generateWindDeviation()
            sbus_command = autopilot.fly(MavLink(roll, pitch, yaw, altitude), controller.get_imu)
            controller.moveByRollPitchYawThrottleAsync(sbus_command.roll, sbus_command.pitch, sbus_command.yaw, sbus_command.throttle)
        else:
            controller.moveByRollPitchYawThrottleAsync(roll_joy, pitch_joy, -yaw_joy, throttle_joy)
        # time.sleep(0.05)