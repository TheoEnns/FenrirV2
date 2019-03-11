import motorDriver
import unittest
import configLoader
import time, math

time_length = 3

class TestServoControl(unittest.TestCase):
    # def test_joints_sweep_fractions(self):
    #     print "> test_joints_sweep_fractions"
    #     for joint in self.joints.values():
    #         print joint.name
    #         init_time = time.time()
    #         while time.time() < (init_time + time_length):
    #             fraction = math.sin(2*math.pi*(time.time() - init_time)/(time_length))
    #             joint.set_fraction(fraction)
    #             time.sleep(0.001)

    def test_joint_sweep_degrees(self):
        print "> test_joint_sweep_degrees"
        for joint in self.joints.values():
            print joint.name
            init_time = time.time()
            while time.time() < (init_time + time_length):
                degrees = 180*math.sin(2*math.pi*(time.time() - init_time)/(time_length))
                joint.set_degrees(degrees)
                time.sleep(0.001)

    def test_femur_sweep_fractions(self):
        print "> test_femur_sweep_fractions"
        time_length = 2
        for joint in self.joints.values():
            if not joint.name.startswith("Femur"):
                continue
            print joint.name
            init_time = time.time()
            while time.time() < (init_time + time_length):
                fraction = math.sin(2 * math.pi * (time.time() - init_time) / (time_length))
                joint.set_fraction(fraction)
                time.sleep(0.001)

    def test_head_gear_sweep_fractions(self):
        print "> test_head_gear_sweep_fractions"
        time_length = 2
        for joint in self.joints.values():
            if not joint.name.startswith("HeadGear"):
                continue
            print joint.name
            init_time = time.time()
            while time.time() < (init_time + time_length):
                fraction = math.sin(2 * math.pi * (time.time() - init_time) / (time_length))
                joint.set_fraction(fraction)
                time.sleep(0.001)

    def test_head_pivot_sweep_fractions(self):
        print "> test_head_pivot_sweep_fractions"
        time_length = 2
        for joint in self.joints.values():
            if not joint.name.startswith("HeadPivot"):
                continue
            print joint.name
            init_time = time.time()
            while time.time() < (init_time + time_length):
                fraction = math.sin(2 * math.pi * (time.time() - init_time) / (time_length))
                joint.set_fraction(fraction)
                time.sleep(0.001)

    def test_hip_sweep_fractions(self):
        print "> test_hip_sweep_fractions"
        time_length = 2
        for joint in self.joints.values():
            if not joint.name.startswith("Hip"):
                continue
            print joint.name
            init_time = time.time()
            while time.time() < (init_time + time_length):
                fraction = math.sin(2 * math.pi * (time.time() - init_time) / (time_length))
                joint.set_fraction(fraction)
                time.sleep(0.001)
    
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
        self.joints, segments = configLoader.load_servos_config(self.dynM, "servos.yaml")


if __name__ == '__main__':
    unittest.main()