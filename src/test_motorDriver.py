import motorDriver
import unittest
import time, math

class TestMotorDriver(unittest.TestCase):
    def test_threading_advanced_multicommand(self):
        print "> test_threading_advanced"
        self.dynM.start()
        timeElapse = 10.0
        iter = 0.0
        endTime = time.time() + timeElapse
        while endTime > time.time():
            position = (int)(2048 + (300*math.sin(time.time())))
            for ID in self.dynM.IDs:
                self.dynM.write_goal(ID,position)
            time.sleep(.01)
            self.dynM.poll_current_positions()
            # print round(endTime -time.time(), 3), self.dynM.read_currents()
            iter = iter + 1.0
        print "Message Cycles Per Second: ", 1.0/(timeElapse/iter)
        self.dynM.stop()

    def test_threading_basic(self):
        print "> test_threading_basic"
        ID = self.dynM.IDs[0]
        self.dynM.start()
        time.sleep(0.5)
        self.dynM.write_goal(ID,2150)
        time.sleep(0.5)
        self.dynM.poll_current_position(ID)
        self.dynM.write_goal(ID,2300)
        self.dynM.poll_current_position(ID)
        time.sleep(0.5)
        self.dynM.reboot_motor(ID)
        self.dynM.command_goal(ID, 2150)
        time.sleep(0.5)
        self.dynM.stop()
        self.dynM.command_goal(ID, 2048)

    def test_motor_motion(self):
        print "> test_motor_motion"
        timeElapse = 10.0
        iter = 0.0
        indx = 0
        endTime = time.time() + timeElapse
        while endTime > time.time():
            position = (int)(2048 + (300*math.sin(time.time())))
            for ID in self.dynM.IDs:
                self.dynM.write_goal(ID,position)
            self.dynM.command_goals()
            # self.dynM.poll_current_positions()
            iter = iter + 1.0
            indx = (indx + 1)% len(self.dynM.IDs)
        print "Message Cycles Per Second: ", 1.0/(timeElapse/iter)

    def test_reboot(self):
        print "> test_reboot"
        for ID in self.dynM.IDs:
            self.dynM.reboot_motor(ID)
            self.dynM.command_goal(ID, 2048)
    
    def tearDown(self):
        print "> tearDown"
        self.dynM.stop()
        for ID in self.dynM.IDs:
                self.dynM.write_goal(ID,2048)
        self.dynM.command_goals()

    def setUp(self):
        print "> setUp"
        self.dynM = motorDriver.DynamixelMaster()

        

if __name__ == '__main__':
    unittest.main()