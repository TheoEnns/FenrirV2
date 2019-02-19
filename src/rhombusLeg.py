import numpy as np
import math


class rhombusLeg():
    ####
    # initialization
    ####

    def __init__(self, legID, configuration, driver):
        self.ID = legID
        self.config = configuration[legID]
        self.driver = driver
        self.HipSocket = np.array(self.config["HipSocket"]["X"], self.config["HipSocket"]["Y"], self.config["HipSocket"]["Z"])
        self.FemurRoll = np.array(self.config["FemurRoll"]["X"], self.config["FemurRoll"]["Y"], self.config["FemurRoll"]["Z"])
        self.FemurSwing = np.array(self.config["FemurSwing"]["X"], self.config["FemurSwing"]["Y"], self.config["FemurSwing"]["Z"])
        self.HipSocketR = self.config["HipSocket"]["R"]
        self.HipSocketRsqr = self.HipSocketR**2
        self.FemurRollR = self.config["FemurRoll"]["R"]
        self.FemurSwingR = self.config["FemurSwing"]["R"]
        self.pi_2 = math.pi/2

    # def _solve_foot(self, floorTarget):
    #
    #     return success, footCenter, footAnchor

    def _solve_Hip(self, footCenter):
        hypotuneuse = self.HipSocket - footCenter
        hypotuneuseRsqr = hypotuneuse[1]**2 + hypotuneuse[2]**2 # Y,Z plane distance
        if(hypotuneuseRsqr - self.HipSocketRsqr
            return false, 0, np.zeros(3),  np.zeros(3),  np.zeros(3)
        legYZlength = math.sqrt(hypotuneuseRsqr - self.HipSocketRsqr)
        hypotuneuseAngle = math.atan2(hypotuneuse[2], - hypotuneuse[1])
        secondAngle = math.asin(legYZlength/math.sqrt(hypotuneuseRsqr))
        hipSocket_Angle = self.pi_2 - hypotuneuseAngle - secondAngle
        return true, hipSocket_Angle  #, footCenterInHipPlane, femurA, femurB

    # def _solve_Femurs(self, footCenterInHipPlane):
    #
    #     return success, femurA_Angle, femurB_Angle, tibiaA, tibiaB