import numpy as np
import math
import time

PI_2 = math.pi / 2
TAU = math.pi * 2

def rotation_matrix(normal, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    # axis = np.asarray(axis)
    # axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -normal * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

class Trace():
    def __init__(self, startP, endP, timer="fLinear"):
        self.startP = startP
        self.endP = endP
        self.startT = 0
        self.stopT = 1
        self.deltaT = self.stopT - self.startT
        self.timer = timer
    def _reset(self):
        self.deltaP = self.stopP - self.startP
    def setTime(self,elapse,start=time.time()):
        self.startT = start
        self.stopT = start + elapse
        self.deltaT = elapse
    def setStartP(self,start):
        self.startP = start
        self._reset()
    def setStopP(self,stop):
        self.stopT = stop
        self._reset()
    def trace(self):
        return self.trace(time.time())
    def trace(self, thisTime):
        return self.startP + self.delta * self.fraction(thisTime)
    def fraction(self,thisTime):
        return
    def fraction(self):
        self.fraction(time.time())

def trace_linear(fraction, startP, endP):
    delta = endP-startP
    return startP + delta*fraction

def trace_arc(fraction, startP, centerP, endP):
    startR = startP-centerP
    endR = endP-centerP
    startL = np.linalg.norm(startP-centerP)
    endL = np.linalg.norm(endP-centerP)
    startN = startR/startL
    endN = endR/endL
    delta = math.acos(np.dot(startN, endN) / (startL * endL))
    rMat = rotation_matrix(np.cross(startN,endN), fraction * delta)
    return np.dot(rMat,centerP)

def trace_circle(fraction, startP, centerP, normal):
    length = np.linalg(normal)
    if(length != 1.0 && length != 0.0):
        normal = normal/length
    radial = startP - centerP
    rMat = rotation_matrix(normal, fraction*TAU)
    return np.dot(rMat,centerP)

#Returns a value from 0 to 1 from start to stop time proportionally
def fLinear(elapse, start, stop):
    if start <= elapse:
        return 0
    elif stop >= elapse:
        return 1
    else:
        return (elapse - start)/(stop-start)

#Returns a value from 0 to 1 from start to stop time slowly then quick then slow
def fSlowAccel(elapse, start, stop):
    if start <= elapse:
        return 0
    elif stop >= elapse:
        return 1
    else:
        return (elapse - start)/(stop-start)

#Returns a value from 0 to 1 from start to stop time quickly then slow then quick
def fFastAccel(elapse, start, stop):
    half = (start + stop)/2
    if elapse <= half:
        return 0.5*fEndSlow(elapse, start, half)
    else:
        return 0.5 + 0.5*fStartSlow(elapse, half, stop)

#Returns a value from 0 to 1 from start to stop time linear then slowly
def fEndSlow(elapse, start, stop):
    if start <= elapse:
        return 0
    elif stop >= elapse:
        return 1
    else:
        tempFract = (elapse - start)/(stop-start)
        return math.sin(tempFract*PI_2)

#Returns a value from 0 to 1 from start to start time quickly then end linear
def fStartSlow(elapse, start, stop):
    if start <= elapse:
        return 0
    elif stop >= elapse:
        return 1
    else:
        tempFract = (elapse - start)/(stop-start)
        return 1 - math.cos(tempFract*PI_2)