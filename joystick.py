import pygame
import sys
from pygame.locals import *


class joystick_handler(object):
    def __init__(self, id):
        self.id = id
        self.joy = pygame.joystick.Joystick(id)
        self.name = self.joy.get_name()
        self.joy.init()
        self.numaxes = min(4, self.joy.get_numaxes())

        self.axes = [0.0] * self.numaxes


    def update(self):
        pygame.event.pump()
        for i in range(self.numaxes):
            self.axes[i] = round(self.joy.get_axis(i), 3)
    


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
        print("Joystick:", joy.name)
        print("Axes:", joy.axes)
        print()


if __name__ == "__main__":
    program = input_test()
    program.run()
