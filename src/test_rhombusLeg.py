import motorDriver
import rhombusLeg
import unittest
import configLoader
import time, math
import numpy as np


CEN_HIPA_Z = 0.0
CEN_HIPA_Y = 67.625
CEN_HIPA_X = 70.75

CEN_HIPB_Z = 0.0
CEN_HIPB_Y = 67.625
CEN_HIPB_X = 99.25

HIG_FOOT_CENTER_Z = 162.72431069
HIG_FOOT_CENTER_Y = 67.625
HIG_FOOT_CENTER_X = 85.0
HIG_FEMURA_Z = -51.49744846
HIG_FEMURA_Y = 67.625
HIG_FEMURA_X = 39.96018347
HIG_FEMURB_Z = -51.49744846
HIG_FEMURB_Y = 67.625
HIG_FEMURB_X = 130.03961653

MID_FOOT_CENTER_Z = 94.27055479
MID_FOOT_CENTER_Y = 67.625
MID_FOOT_CENTER_X = 85.0
MID_FEMURA_Z = 0.0
MID_FEMURA_Y = 67.625
MID_FEMURA_X = 10.75
MID_FEMURB_Z = 0.0
MID_FEMURB_Y = 67.625
MID_FEMURB_X = 159.25

LOW_FOOT_CENTER_Z = 58.41330304
LOW_FOOT_CENTER_Y = 67.625
LOW_FOOT_CENTER_X = 85.0
LOW_FEMURA_Z = 56.74284096
LOW_FEMURA_Y = 67.625
LOW_FEMURA_X = 51.25
LOW_FEMURB_Z = 56.74284096
LOW_FEMURB_Y = 67.625
LOW_FEMURB_X = 118.75

UPDATE_TIME = 0.001
TIMER = 3
MED_VEL = 10.0*(UPDATE_TIME/0.001)
MED_DIST = 30
REPS = 2

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
        np.array([ MID_FOOT_CENTER_X + dx, -MID_FOOT_CENTER_Y + dy, -(MID_FOOT_CENTER_Z + 20 + dz)]),
        joints)
    legs['BR'].perform_IK(
        np.array([-MID_FOOT_CENTER_X + dx, -MID_FOOT_CENTER_Y + dy, -(MID_FOOT_CENTER_Z + 20 + dz)]),
        joints)
    legs['FL'].perform_IK(
        np.array([ MID_FOOT_CENTER_X + dx,  MID_FOOT_CENTER_Y + dy, -(MID_FOOT_CENTER_Z + 20 + dz)]),
        joints)
    legs['BL'].perform_IK(
        np.array([-MID_FOOT_CENTER_X + dx,  MID_FOOT_CENTER_Y + dy, -(MID_FOOT_CENTER_Z + 20 + dz)]),
        joints)

class TestRhombusLeg(unittest.TestCase):

    # def test_LegInit(self):
    #     print "> test_LegInit"
    #     rhombusLeg.rhombusLeg("LegFR",self.legs,self.dynM)
    #     rhombusLeg.rhombusLeg("LegBR",self.legs,self.dynM)
    #     rhombusLeg.rhombusLeg("LegFL",self.legs,self.dynM)
    #     rhombusLeg.rhombusLeg("LegBL",self.legs,self.dynM)


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
        linear_path(legs, self.joints, 0,   0,   -magY,    0,    0,      0,      TIMER/2.0)

    # def test_LegSideToSide(self):
    #     print "> test_LegSideToSide"
    #     myLeg = rhombusLeg.rhombusLeg("LegFR",self.legs,self.dynM)
    #     for perturbation in range(0,(int)(MED_DIST*MED_VEL),1):
    #         perturbation /= MED_VEL
    #         myLeg.perform_IK(
    #             np.array([ MID_FOOT_CENTER_X, -MID_FOOT_CENTER_Y + perturbation, -(MID_FOOT_CENTER_Z + 20 )]), self.joints)
    #         time.sleep(0.001)
    #     for rep in range(0,4):
    #         for perturbation in range((int)(MED_DIST*MED_VEL), -(int)(MED_DIST*MED_VEL), -1):
    #             perturbation /= MED_VEL
    #             myLeg.perform_IK(
    #                 np.array([ MID_FOOT_CENTER_X, -MID_FOOT_CENTER_Y + perturbation, -(MID_FOOT_CENTER_Z + 20 )]), self.joints)
    #             time.sleep(0.001)
    #         for perturbation in range(-(int)(MED_DIST*MED_VEL),(int)(MED_DIST*MED_VEL), 1):
    #             perturbation /= MED_VEL
    #             myLeg.perform_IK(
    #                 np.array([ MID_FOOT_CENTER_X , -MID_FOOT_CENTER_Y+ perturbation, -(MID_FOOT_CENTER_Z + 20 )]), self.joints)
    #             time.sleep(0.001)
    #     for perturbation in range( (int)(MED_DIST*MED_VEL), 0,-1):
    #         perturbation /= MED_VEL
    #         myLeg.perform_IK(
    #             np.array([ MID_FOOT_CENTER_X, -MID_FOOT_CENTER_Y + perturbation, -(MID_FOOT_CENTER_Z + 20 )]), self.joints)
    #         time.sleep(0.001)


    def tearDown(self):
        print "> tearDown"
        self.dynM.stop()
        for ID in self.dynM.IDs:
            self.dynM.write_goal(ID, 2048)
        self.dynM.command_goals()


    def setUp(self):
        print "> setUp"
        self.dynM = motorDriver.DynamixelMaster()
        self.dynM.start()
        self.joints, self.legs = configLoader.load_servos_config(self.dynM, "servos.yaml")


if __name__ == '__main__':
    unittest.main()