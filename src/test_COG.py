import motorDriver
import rhombusLeg
import unittest
import configLoader
import time, math
import numpy as np
from test_constants import *

UPDATE_TIME = 0.001
TIMER = 3
MED_VEL = 10.0*(UPDATE_TIME/0.001)
MED_DIST = 30
REPS = 5

def linear_path(legs,joints,x1,y1,z1,x2,y2,z2,timer):
    max = (int)(timer/UPDATE_TIME)
    # print x2,y2,z2,timer
    for indx in range(0, max, 1):
        fraction = (float)(indx)/max
        mid_place_legs(legs, joints,
                       (x2 - x1) * fraction + x1,
                       (y2 - y1) * fraction + y1,
                       (z2 - z1) * fraction + z1)
    time.sleep(0.001)

def circle_path(legs,joints,radius,x1,y1,z1,timer):
    max = (int)(2.0* math.pi*timer/UPDATE_TIME)
    # print x2,y2,z2,timer
    for indx in range(0, max, 1):
        fraction = 2.0*math.pi*(float)(indx)/max
        # print radius * math.cos(fraction) + x1, radius * math.sin(fraction) + y1, fraction
        mid_place_legs(legs, joints,
                       radius * math.cos(fraction) + x1,
                       radius * math.sin(fraction) + y1,
                       z1)
    time.sleep(0.001)

def mid_place_legs(legs,joints,dx,dy,dz):
    legs['FR'].perform_IK(
        np.array([ MID_FOOT_CENTER_X + dx, -MID_FOOT_CENTER_Y + dy, -(LOW_FOOT_CENTER_Z + 20 + dz)]),
        joints)
    legs['BR'].perform_IK(
        np.array([-MID_FOOT_CENTER_X + dx, -MID_FOOT_CENTER_Y + dy, -(LOW_FOOT_CENTER_Z + 20 + dz)]),
        joints)
    legs['FL'].perform_IK(
        np.array([ MID_FOOT_CENTER_X + dx,  MID_FOOT_CENTER_Y + dy, -(LOW_FOOT_CENTER_Z + 20 + dz)]),
        joints)
    legs['BL'].perform_IK(
        np.array([-MID_FOOT_CENTER_X + dx,  MID_FOOT_CENTER_Y + dy, -(LOW_FOOT_CENTER_Z + 20 + dz)]),
        joints)

class TestCOG(unittest.TestCase):

    def test_LegCircleTrace(self):
        # COG Results:
        #      Height = -25, fails at 35 cm radius
        #      Height = 0, fails at 35 cm radius
        #      Height = 40, fails at 39 cm radius
        #
        print "> def test_LegCircleTrace"
        radius = 30 ## Safe
        magZ = -25  
        linear_path(self.legs, self.joints, 0,   0,  0,      radius,      0,      magZ,    TIMER/2.0)
        for rep in range(50):
            print "magZ:", magZ, "mm"
            circle_path(self.legs, self.joints, radius, 0, 0, magZ, TIMER)
            magZ = magZ +1
        linear_path(self.legs, self.joints, radius,   0,  magZ,      0,      0,      0,    TIMER/2.0)

    def tearDown(self):
        print "> tearDown"
        self.dynM.stop()
        for name in self.joints.keys():
            self.dynM.write_goal(self.joints[name].ID, self.joints[name].center)
        self.dynM.command_goals()


    def setUp(self):
        print "> setUp"
        self.dynM = motorDriver.DynamixelMaster()
        self.dynM.start()
        self.joints, self.segs = configLoader.load_servos_config(self.dynM, "servos.yaml")
        self.legs = {}
        self.legs['FR'] = rhombusLeg.rhombusLeg("LegFR", self.segs, self.dynM)
        self.legs['BR'] = rhombusLeg.rhombusLeg("LegBR", self.segs, self.dynM)
        self.legs['FL'] = rhombusLeg.rhombusLeg("LegFL", self.segs, self.dynM)
        self.legs['BL'] = rhombusLeg.rhombusLeg("LegBL", self.segs, self.dynM)


if __name__ == '__main__':
    unittest.main()