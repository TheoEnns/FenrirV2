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
    print x2,y2,z2,timer
    for indx in range(0, max, 1):
        fraction = (float)(indx)/max
        mid_place_legs(legs, joints,
                       (x2 - x1) * fraction + x1,
                       (y2 - y1) * fraction + y1,
                       (z2 - z1) * fraction + z1)
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

class TestRhombusLeg(unittest.TestCase):

    def test_LegInit(self):
        print "> test_LegInit"
        legs = {}
        legs['FR'] = rhombusLeg.rhombusLeg("LegFR", self.legs, self.dynM)
        legs['BR'] = rhombusLeg.rhombusLeg("LegBR", self.legs, self.dynM)
        legs['FL'] = rhombusLeg.rhombusLeg("LegFL", self.legs, self.dynM)
        legs['BL'] = rhombusLeg.rhombusLeg("LegBL", self.legs, self.dynM)
        linear_path(legs, self.joints, 0, 0, 0, 0, 0, 0, 0.5)
        time.sleep(5)


    def test_LegSquareXZ(self):
        print "> test_LegSquareXZ"
        legs = {}
        legs['FR'] = rhombusLeg.rhombusLeg("LegFR", self.legs, self.dynM)
        legs['BR'] = rhombusLeg.rhombusLeg("LegBR", self.legs, self.dynM)
        legs['FL'] = rhombusLeg.rhombusLeg("LegFL", self.legs, self.dynM)
        legs['BL'] = rhombusLeg.rhombusLeg("LegBL", self.legs, self.dynM)
        magX = 30
        magZ = 30
        linear_path(legs, self.joints, 0,   0,  0,      0,      0,      -magZ,    TIMER/2.0)
        for rep in range(REPS):
            linear_path(legs, self.joints, 0,   0,  -magZ,    magX,     0,      -magZ,    TIMER)
            linear_path(legs, self.joints, magX,  0,  -magZ,    magX,     0,      magZ,     TIMER)
            linear_path(legs, self.joints, magX,  0,  magZ,     -magX,    0,      magZ,     TIMER)
            linear_path(legs, self.joints, -magX, 0,  magZ,     -magX,    0,      -magZ,    TIMER)
            linear_path(legs, self.joints, -magX, 0,  -magZ,    0,      0,      -magZ,    TIMER)
        linear_path(legs, self.joints, 0,   0,  -magZ,    0,      0,      0,      TIMER/2.0)

    def test_LegSquareYZ(self):
        print "> test_LegSquareYZ"
        legs = {}
        legs['FR'] = rhombusLeg.rhombusLeg("LegFR", self.legs, self.dynM)
        legs['BR'] = rhombusLeg.rhombusLeg("LegBR", self.legs, self.dynM)
        legs['FL'] = rhombusLeg.rhombusLeg("LegFL", self.legs, self.dynM)
        legs['BL'] = rhombusLeg.rhombusLeg("LegBL", self.legs, self.dynM)
        magY = 20
        magZ = 30
        linear_path(legs, self.joints, 0,   0,    0,      0,    0,      -magZ,    TIMER/2.0)
        for rep in range(REPS):
            linear_path(legs, self.joints, 0,   0,    -magZ,    0,    magY,     -magZ,    TIMER)
            linear_path(legs, self.joints, 0,   magY,   -magZ,    0,    magY,     magZ,     TIMER)
            linear_path(legs, self.joints, 0,   magY,   magZ,     0,    -magY,     magZ,    TIMER)
            linear_path(legs, self.joints, 0,   -magY,  magZ,     0,    -magY,    -magZ,    TIMER)
            linear_path(legs, self.joints, 0,   -magY,  -magZ,    0,    0,      -magZ,    TIMER)
        linear_path(legs, self.joints, 0,   0,   -magZ,    0,    0,      0,      TIMER/2.0)

    def test_LegSideToSide(self):
        print "> test_LegSideToSide"
        legs = {}
        legs['FR'] = rhombusLeg.rhombusLeg("LegFR", self.legs, self.dynM)
        legs['BR'] = rhombusLeg.rhombusLeg("LegBR", self.legs, self.dynM)
        legs['FL'] = rhombusLeg.rhombusLeg("LegFL", self.legs, self.dynM)
        legs['BL'] = rhombusLeg.rhombusLeg("LegBL", self.legs, self.dynM)
        magY = 20
        linear_path(legs, self.joints, 0,   0,    0,      0,    -magY,      0,    TIMER/2.0)
        for rep in range(REPS):
            linear_path(legs, self.joints, 0,   -magY,  0,     0,    magY,    0,    TIMER)
            linear_path(legs, self.joints, 0,    magY,  0,    0,    -magY,      0,    TIMER)
        linear_path(legs, self.joints, 0,   -magY,    0,   0,    0,      0,      TIMER/2.0)


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
        self.joints, self.legs = configLoader.load_servos_config(self.dynM, "servos.yaml")


if __name__ == '__main__':
    unittest.main()