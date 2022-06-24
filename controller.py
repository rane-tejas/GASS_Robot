import time
import numpy as np
from collections import namedtuple

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
    leg_motor: motor(180, 100, 2)
    patch_motor: motor(307, 60, 0.25)

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
        pwm_val = motor.stall + direction*motor.step
        self.pwm.set_pwm(pin, 0, pwm_val)
        time.sleep(motor.time)
        self.pwm.set_pwm(pin, 0, motor.stall)
        time.sleep(1)

    def toggle_leg(self, pin, direction, motor):
        motor_deg = motor.stall - (direction*motor.step)
        self.kit.servo[pin].set_pulse_width_range(500, 2500)
        self.kit.servo[pin].actuation_range = 270
        self.kit.servo[pin].angle = motor_deg
        time.sleep(motor.time)

    def home_leg(self, pin):
        self.kit.servo[pin].set_pulse_width_range(500, 2500)
        self.kit.servo[pin].actuation_range = 270
        self.kit.servo[pin].angle = 0
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
        self.utils.home_leg(self.limb.leg_pin)

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
        self.angles = [0.0, 2*PI/5, 4*PI/5, 6*PI/5, 8*PI/5]

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

        for ang in self.angles:
            act = self.l * np.cos(self.d_ang-ang)
            action.append(act)
        return action

    def run(self):
        """ 
        This function runs the main controller loop per control step.
        """

        action = self.get_action()
        
        for i in range(len(action)):
            if action[i] > 0:
                self.limbs[i].extend_limb()
            else:
                self.limbs[i].contract_limb()

        self.toggle_patches(1)

        for i in range(len(action)):
            if action[i] > 0:
                self.limbs[i].contract_limb()
            else:
                self.limbs[i].extend_limb()

        self.toggle_patches(0)

if __name__ == "__main__":
    # Initialize Limb Objects
    L1 = Limb(limb(leg_pin=7, patch_pin=6))
    L2 = Limb(limb(leg_pin=9, patch_pin=8))
    L3 = Limb(limb(leg_pin=11, patch_pin=10))
    L4 = Limb(limb(leg_pin=13, patch_pin=12))
    L5 = Limb(limb(leg_pin=15, patch_pin=14))

    limbs = [L1, L2, L3, L4, L5]
    # robot = namedtuple('robot', 'limb1, limb2, limb3, limb4, limb5')
    # GASS = robot(L1, L2, L3, L4, L5)

    control = GASSController(limbs)
    control.run()