import time
import argparse
import numpy as np
from collections import namedtuple
from dataclasses import dataclass

import Adafruit_PCA9685
from adafruit_servokit import ServoKit


PI = np.pi

@dataclass
class motor:
    stall: float
    step: float
    time: float

@dataclass
class limb:
    leg_pin: int
    patch_pin: int
    leg_motor: motor = motor(70, 70, 2)
    patch_motor: motor = motor(307, 60, 0.45)

class Utils():
    """
    Level 1:
    All utilities related to the servo motors
    - servo moving commands
    - servo home command
    - servo pin definitions
    """

    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
        self.pwm.set_pwm_freq(50)
        self.kit = ServoKit(channels=16)

    def toggle_patch(self, pin, direction, motor):
        pwm_val = motor.stall - direction*motor.step
        self.pwm.set_pwm(pin, 0, pwm_val)
        time.sleep(motor.time)
        self.pwm.set_pwm(pin, 0, motor.stall)
        print("patch", pin)
        time.sleep(motor.time)

    def toggle_leg(self, pin, direction, motor):
        motor_deg = motor.stall - (direction*motor.step)
        self.kit.servo[pin].set_pulse_width_range(500, 2500)
        self.kit.servo[pin].actuation_range = 270
        print("motor deg", motor_deg, pin)
        self.kit.servo[pin].angle = motor_deg
        time.sleep(motor.time)

    def home_leg(self, pin, motor):
        self.kit.servo[pin].set_pulse_width_range(500, 2500)
        self.kit.servo[pin].actuation_range = 270
        self.kit.servo[pin].angle = motor.stall + motor.step
        print("home", pin)
        time.sleep(1)

class Limb():
    """
    Level 2:
    Limb definitions and functions related to moving the limb
    """

    def __init__(self, limb):
        self.utils = Utils()
        self.limb = limb

    def engage_gecko(self):
        self.utils.toggle_patch(self.limb.patch_pin, 1, self.limb.patch_motor)

    def disengage_gecko(self):
        self.utils.toggle_patch(self.limb.patch_pin, -1, self.limb.patch_motor)

    def extend_limb(self):
        self.utils.toggle_leg(self.limb.leg_pin, 1, self.limb.leg_motor)

    def contract_limb(self):
        self.utils.toggle_leg(self.limb.leg_pin, -1, self.limb.leg_motor)

    def limb_to_home(self):
        self.utils.home_leg(self.limb.leg_pin, self.limb.leg_motor)

class GASSController():
    """
    Level 3:
    Robot controller functions
    - takes direction vector and publishes command to move
    (?) should this be for one step or run the entire loop? (currently written for one step)
    """

    def __init__(self, limbs):
        self.limbs = limbs
        self.l = 1
        self.d_ang = 0.0
        self.angles = [-2*PI/5, 0.0, 2*PI/5, 4*PI/5, 6*PI/5]
        self.neg_action_idx = None
        self.neg_action_flag = False

    def toggle_patches(self, flag):
        for limb in self.limbs:
            if flag:
                limb.engage_gecko()
            else:
                limb.disengage_gecko()

    def get_direction(self):
        """
        This function returns the direction vector for the robot.
        """
        self.d_ang = 0.0

    def get_action(self):
        """
        This function returns the action to be taken for each Limb.
        """
        action = []
        self.get_direction()

        for i, ang in enumerate(self.angles):
            act = self.l * np.cos(self.d_ang-ang)
            if not self.neg_action_flag and act < 0:
                self.neg_action_idx = i - 1
                self.neg_action_flag = True
            action.append(act)
        return action

    def run(self):
        """
        This function runs the main controller loop per control step.
        """

        action = self.get_action()
        print('Action ', action)

        for i, act in enumerate(action[self.neg_action_idx:] + action[:self.neg_action_idx]):
            if act < 0:
                self.limbs[i].engage_gecko()
            elif act > 0:
                self.limbs[i].extend_limb()

        for i, act in enumerate(action[self.neg_action_idx:] + action[:self.neg_action_idx]):
            if act < 0:
                self.limbs[i].extend_limb()
            elif act > 0:
                self.limbs[i].engage_gecko()

        for i, act in enumerate(action[self.neg_action_idx:] + action[:self.neg_action_idx]):
            if act < 0:
                self.limbs[i].disengage_gecko()
            elif act > 0:
                self.limbs[i].contract_limb()

        for i, act in enumerate(action[self.neg_action_idx:] + action[:self.neg_action_idx]):
            if act < 0:
                self.limbs[i].contract_limb()
            elif act > 0:
                self.limbs[i].disengage_gecko()

        self.neg_action_idx = None
        self.neg_action_flag = False


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--zero', action='store_true', help='Zero all limb motors')
    parser.add_argument('--home', action='store_true', help='Home all limb motors')

    args = parser.parse_args()

    # Initialize Limb Objects
    L1 = Limb(limb(leg_pin=7, patch_pin=6))
    L2 = Limb(limb(leg_pin=9, patch_pin=8))
    L3 = Limb(limb(leg_pin=11, patch_pin=10))
    L4 = Limb(limb(leg_pin=13, patch_pin=12))
    L5 = Limb(limb(leg_pin=15, patch_pin=14))

    limbs = [L1, L2, L3, L4, L5]
    robot = namedtuple('robot', 'limb1, limb2, limb3, limb4, limb5')
    GASS = robot(L1, L2, L3, L4, L5)

    ### Setting Leg Motor zeroes ###
    if args.zero:
        utils = Utils()
        utils.kit.servo[7].angle=0
        time.sleep(1)
        utils.kit.servo[9].angle=0
        time.sleep(1)
        utils.kit.servo[11].angle=0
        time.sleep(1)
        utils.kit.servo[13].angle=0
        time.sleep(1)
        utils.kit.servo[15].angle=0
        time.sleep(1)

    ### Setting Leg Motor home ###
    elif args.home:
        utils = Utils()
        utils.kit.servo[7].actuation_range = 270
        utils.kit.servo[7].angle=200
        time.sleep(1)
        utils.kit.servo[9].actuation_range = 270
        utils.kit.servo[9].angle=200
        time.sleep(1)
        utils.kit.servo[11].actuation_range = 270
        utils.kit.servo[11].angle=200
        time.sleep(1)
        utils.kit.servo[13].actuation_range = 270
        utils.kit.servo[13].angle=200
        time.sleep(1)
        utils.kit.servo[15].actuation_range = 270
        utils.kit.servo[15].angle=200
        time.sleep(1)

    else:
        control = GASSController(limbs)
        control.run()

    ### Servo test ###
#     utils = Utils()
#     patch_motor = motor(300, 150, 0.2)
#     leg_motor = motor(125, 125, 2)
#     print('toggling one motor')
#     utils.kit.servo[7].set_pulse_width_range(500, 2500)
#     utils.kit.servo[7].actuation_range = 270
#     utils.toggle_leg(pin=7, direction=-1, motor=leg_motor)
# #     utils.toggle_leg(pin=15, direction=-1, motor=leg_motor)
#     utils.kit.servo[15].angle=180
#     time.sleep(5)
#     utils.kit.servo[15].angle=0
# #     utils.toggle_leg(pin=7, direction=1, motor=leg_motor)
#     time.sleep(5)
#     utils.kit.servo[15].angle=180
#     utils.toggle_leg(pin=7, direction=-1, motor=leg_motor)

    ### Setting patch motor zeroes ###
#     utils.toggle_patch(pin=6, direction=1, motor=patch_motor)
#     utils.toggle_patch(pin=8, direction=1, motor=patch_motor)
#     utils.toggle_patch(pin=10, direction=1, motor=patch_motor)
#     utils.toggle_patch(pin=12, direction=1, motor=patch_motor)
#     utils.toggle_patch(pin=14, direction=1, motor=patch_motor)
#     utils.toggle_patch(pin=6, direction=-1, motor=patch_motor)
#     utils.toggle_patch(pin=8, direction=-1, motor=patch_motor)
#     utils.toggle_patch(pin=10, direction=-1, motor=patch_motor)
#     utils.toggle_patch(pin=12, direction=-1, motor=patch_motor)
#     utils.toggle_patch(pin=14, direction=-1, motor=patch_motor)

#     utils.toggle_patch(pin=8, direction=-1, motor=patch_motor)
#     utils.toggle_patch(pin=10, direction=-1, motor=patch_motor)
#     utils.toggle_patch(pin=12, direction=-1, motor=patch_motor)
#     utils.toggle_patch(pin=14, direction=-1, motor=patch_motor)