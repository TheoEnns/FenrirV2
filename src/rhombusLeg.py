import numpy as np
import math

PI_2 = math.pi / 2
TAU = math.pi * 2

class rhombusLeg():
    ####
    # initialization
    ####

    def __init__(self, legID, configuration, driver):
        self.ID = legID
        self.IDpostfix = legID[-2:]
        self.ID_hip = "Hip" + self.IDpostfix
        self.ID_femura = "FemurA" + self.IDpostfix
        self.ID_femurb = "FemurB" + self.IDpostfix
        self.config = configuration[legID]
        self.driver = driver
        self.HipSocket = np.array([self.config["HipSocket"]["X"], self.config["HipSocket"]["Y"], self.config["HipSocket"]["Z"]])
        self.HipSwingY = self.config["HipSwingA"]["Y"]
        self.HipSwingAX = self.config["HipSwingA"]["X"]
        self.HipSwingBX = self.config["HipSwingB"]["X"]
        self.HipSwingR = self.config["HipSwingA"]["R"]
        self.HipSwingRsqr = self.HipSwingR**2
        self.FemurRoll = np.array([self.config["FemurRoll"]["X"], self.config["FemurRoll"]["Y"], self.config["FemurRoll"]["Z"]])
        self.FemurSwing = np.array([self.config["FemurSwing"]["X"], self.config["FemurSwing"]["Y"], self.config["FemurSwing"]["Z"]])
        self.HipSocketR = self.config["HipSocket"]["R"]
        #self.FemurRollR = self.config["FemurRoll"]["R"]
        self.FemurSwingR = self.config["FemurSwing"]["R"]
        self.FootX = self.config["Foot"]["X"]
        self.FootY = self.config["Foot"]["Y"]
        self.Tibia = self.config["Tibia"]["Length"]

    def _solve_foot(self, floorTarget):
        hip2floor = self.HipSocket - floorTarget + np.array([ 0, self.HipSwingY, 0])
        forward_angle = math.atan2( hip2floor[0], hip2floor[2])
        left_angle = math.atan2( hip2floor[1], hip2floor[2])
        forward_roll = self.FootX*forward_angle/TAU # Approximate perturbation from the X-diameter of the foot
        left_roll = self.FootY*left_angle/TAU # Approximate perturbation from the Y-diameter of the foot
        footAnchor = floorTarget + np.array([ forward_roll, left_roll, 0])
        footCenter = footAnchor + np.array([ 0,
                                             0,
                                             0.5*self.FootX*math.cos(left_angle)])
        return True, footCenter, footAnchor

    def _solve_Hip(self, footCenter):
        hypotuneuse = self.HipSocket - footCenter
        hypotuneuseRsqr = hypotuneuse[1]**2 + hypotuneuse[2]**2 # Y,Z plane distance
        if(hypotuneuseRsqr - self.HipSwingRsqr < 0.0):
            return False, 0, np.zeros(3),  np.zeros(3),  np.zeros(3)
        legYZlength = math.sqrt(hypotuneuseRsqr - self.HipSwingRsqr)
        hypotuneuseAngle = math.atan2(hypotuneuse[2], - hypotuneuse[1])
        secondAngle = math.asin(legYZlength/math.sqrt(hypotuneuseRsqr))
        hipSocket_Angle = -(math.pi - hypotuneuseAngle - secondAngle)
        femurPivotPoint = self.HipSocket \
                         + np.array([ 0, self.HipSwingY*math.cos(hipSocket_Angle),
                                      -self.HipSwingR*math.sin(hipSocket_Angle)])
        femurA = femurPivotPoint + np.array([ self.HipSwingAX, 0, 0])
        femurB = femurPivotPoint + np.array([ self.HipSwingBX, 0, 0])
        footCenterInHipPlane = np.array([ hypotuneuse[0], -legYZlength])
        femurAInHipPlane = np.array([ self.HipSwingAX - self.HipSocket[0], 0])
        femurBInHipPlane = np.array([ self.HipSwingBX - self.HipSocket[0], 0])
        return True, hipSocket_Angle, footCenterInHipPlane, femurAInHipPlane, femurBInHipPlane

    def _solve_Femurs(self, footCenterInHipPlane, femurAInHipPlane, femurBInHipPlane):
        intersectsA = intersectionOfCircles(footCenterInHipPlane, self.Tibia, femurAInHipPlane, self.FemurSwingR)
        intersectsB = intersectionOfCircles(footCenterInHipPlane, self.Tibia, femurBInHipPlane, self.FemurSwingR)
        #print intersectsA, intersectsB
        femurA_Angle = math.pi
        femurB_Angle = math.pi
        outermostX = 60
        for item in intersectsA:
            tempVect = item - femurAInHipPlane
            tempAngle = math.atan2( tempVect[1], -tempVect[0])
            # print tempAngle
            if(outermostX > tempVect[0]):
                outermostX = tempVect[0]
                femurA_Angle = tempAngle
            #print np.linalg.norm(item - femurAInHipPlane)
        outermostX = -60
        for item in intersectsB:
            tempVect = item - femurBInHipPlane
            tempAngle = math.atan2( tempVect[1], tempVect[0])
            # print tempAngle
            if(outermostX < tempVect[0]):
                outermostX = tempVect[0]
                femurB_Angle = tempAngle
            #print np.linalg.norm(item - femurBInHipPlane)
        return True, femurA_Angle, femurB_Angle

    def calculate_IK(self, floorTarget):
        success, footCenter, footAnchor = \
            self._solve_foot( floorTarget)
        if not success:
            return False, 0, 0, 0
        success, hipSocket_Angle, footCenterInHipPlane, femurAInHipPlane, femurBInHipPlane = \
            self._solve_Hip(footCenter)
        if not success:
            return False, 0, 0, 0
        success, femurA_Angle, femurB_Angle = \
            self._solve_Femurs(footCenterInHipPlane, femurAInHipPlane, femurBInHipPlane)
        if not success:
            return False, 0, 0, 0
        return True, femurA_Angle, femurB_Angle, hipSocket_Angle

    def perform_IK(self, floorTarget, joints):
        success, femurA_Angle, femurB_Angle, hipSocket_Angle = self.calculate_IK( floorTarget)
        joints[self.ID_hip].set_radians(hipSocket_Angle)
        joints[self.ID_femura].set_radians(femurA_Angle)
        joints[self.ID_femurb].set_radians(femurB_Angle)
        return True, femurA_Angle, femurB_Angle, hipSocket_Angle


def intersectionOfCircles(pointA, radiusA, pointB, radiusB):
    seperation = math.sqrt( (pointB[0] - pointA[0])**2 + (pointB[1] - pointA[1])**2 )
    if(seperation > radiusA + radiusB):
        return []
    alpha = (radiusA**2 - radiusB**2 + seperation**2)/(2*seperation)
    hypo = math.sqrt(math.fabs(radiusA**2 - alpha**2))
    midPoint = pointA + alpha*(pointB - pointA)/seperation
    if(seperation == radiusA + radiusB):
        return [midPoint]
    point1 = np.array([midPoint[0] + hypo*(pointB[1] - pointA[1])/seperation,
                       midPoint[1] - hypo*(pointB[0] - pointA[0])/seperation])
    point2 = np.array([midPoint[0] - hypo*(pointB[1] - pointA[1])/seperation,
                       midPoint[1] + hypo*(pointB[0] - pointA[0])/seperation])
    return [point1, point2]

if __name__ == '__main__':
    import motorDriver
    import configLoader
    dynM = motorDriver.DynamixelMaster()
    joints, legs = configLoader.load_servos_config(dynM, "servos.yaml")
    myLeg = rhombusLeg("LegFR", legs, dynM)
    # success, footCenter, footAnchor =  myLeg._solve_foot(np.array([ 85.0 + 0.0, -67.625 + 0.0, -(162.72431069 + 20 - 0)]))
    success, footCenter, footAnchor =  myLeg._solve_foot(np.array([ 85.0 + 0.0, -67.625 + 0.0, -(94.27055479 + 20 + 0)]))
    success, hipSocket_Angle, footCenterInHipPlane, femurAInHipPlane, femurBInHipPlane = myLeg._solve_Hip(footCenter)
    # print footCenterInHipPlane, femurAInHipPlane, femurBInHipPlane
    success, femurA_Angle, femurB_Angle = myLeg._solve_Femurs(footCenterInHipPlane, femurAInHipPlane, femurBInHipPlane)
    print femurA_Angle, femurB_Angle, footCenter

