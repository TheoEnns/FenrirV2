import motorDriver
import rhombusLeg
import unittest
import configLoader
import time, math

class TestRhombusLeg(unittest.TestCase):

    def test_(self):
        print "> test_LegInit"
        rhombusLeg.rhombusLeg("LegFR",self.legs,self.dynM)

    def tearDown(self):
        print "> tearDown"
        self.dynM.stop()
        for ID in self.dynM.IDs:
            self.dynM.write_goal(ID, 2048)
        self.dynM.command_goals()


    def setUp(self):
        print "> setUp"
        self.dynM = motorDriver.DynamixelMaster()
        self.joints, self.legs = configLoader.load_servos_config(self.dynM, "servos.yaml")


if __name__ == '__main__':
    unittest.main()