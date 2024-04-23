import pygame
import sys
from pygame.locals import *


class joystick_handler(object):
    def __init__(self, id):
        self.id = id
        self.joy = pygame.joystick.Joystick(id)
        self.name = self.joy.get_name()
        self.joy.init()
        self.numaxes = self.joy.get_numaxes()
        self.numhats = self.joy.get_numhats()

        self.axes = [0.0] * self.numaxes
        self.hats = [0] * self.numhats

    def update(self):
        pygame.event.pump()
        for i in range(self.numaxes):
            self.axes[i] = round(self.joy.get_axis(i), 3)
        for i in range(self.numhats):
            self.hats[i] = self.joy.get_hat(i)


class input_test(object):
    def __init__(self):
        pygame.init()
        self.joycount = pygame.joystick.get_count()
        if self.joycount == 0:
            print("This program only works with at least one joystick plugged in. No joysticks were detected.")
            sys.exit(1)
        self.joy = []
        for i in range(self.joycount):
            self.joy.append(joystick_handler(i))

    def run(self):
        while True:
            for joy in self.joy:
                joy.update()
                self.print_joystick_data(joy)
            pygame.time.wait(1000)

    def print_joystick_data(self, joy):
        roll = joy.axes[0]
        pitch = joy.axes[1]
        yaw = joy.axes[2]
        throttle = joy.axes[3]
        toggles = joy.axes[6]

        print("Joystick:", joy.name)
        print("Roll:", roll)
        print("Pitch:", pitch)
        print("Yaw:", yaw)
        print("Throttle:", throttle)
        print("Toggle switches:", toggles)
        print()


if __name__ == "__main__":
    program = input_test()
    program.run()
