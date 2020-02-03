import motorDriver
import rhombusLeg
import unittest
import motionProfiler as mp
import configLoader
import time, math
import numpy as np
import matplotlib.pyplot as plt
from test_constants import *

UPDATE_TIME = 0.025
TIMER = 3
MED_VEL = 10.0*(UPDATE_TIME/0.01)
MED_DIST = 30
REPS = 5

class TestRhombusLeg(unittest.TestCase):

    def test_TraceLinear(self):
        print "> test_TraceLinear"
        tracer = mp.Trace_Linear([0,0,0], [3,4,5])
        max = (int)(1.0 / UPDATE_TIME)
        tracer.setTime( 0.5, time.time()+ 0.5)
        # for indx in range(0, max, 1):
        while tracer.fractionNow() != 1:
            arr = tracer.traceNow()
            plt.plot([arr[0]], [arr[1]], marker='o', markersize=3, color="red")
            time.sleep(UPDATE_TIME)

    def test_TraceArc(self):
        print "> test_TraceArc"
        tracer = mp.Trace_Arc([0, 0, 0], [1, 0, 0], [2, 2, 0])
        max = (int)(1.0 / UPDATE_TIME)
        tracer.setTime(0.5, time.time() + 0.5)
        # for indx in range(0, max, 1):
        while tracer.fractionNow() != 1:
            arr = tracer.traceNow()
            # print arr
            plt.plot([arr[0]], [arr[1]], marker='o', markersize=3, color="green")
            time.sleep(UPDATE_TIME)

    def test_TraceCircle(self):
        print "> test_TraceCircle"
        tracer = mp.Trace_Circle([3, 4, 0], [0, 0, 0], [0, 0, 1])
        max = (int)(2.0 / UPDATE_TIME)
        tracer.setTime(1.5, time.time() + 0.5)
        # for indx in range(0, max, 1):
        while tracer.fractionNow() < 1.0:
            arr = tracer.traceNow()
            plt.plot([arr[0]], [arr[1]], marker='o', markersize=3, color="blue")
            time.sleep(UPDATE_TIME)

    def test_fSlowAccel(self):
        print "> test_fSlowAccel"
        tracer = mp.Trace_Circle([0, 4, 0], [0, 0, 0], [0, 0, 1], mp.fSlowAccel)
        max = (int)(2.0 / UPDATE_TIME)
        tracer.setTime(1.5, time.time() + 0.5)
        # for indx in range(0, max, 1):
        while tracer.fractionNow() < 1.0:
            arr = tracer.traceNow()
            plt.plot([arr[0]], [arr[1]], marker='o', markersize=3, color="blue")
            time.sleep(UPDATE_TIME)

    def test_fFastAccel(self):
        print "> test_fFastAccel"
        tracer = mp.Trace_Circle([0, 4, 0], [0, 0, 0], [0, 0, 1], mp.fFastAccel)
        max = (int)(2.0 / UPDATE_TIME)
        tracer.setTime(1.5, time.time() + 0.5)
        # for indx in range(0, max, 1):
        while tracer.fractionNow() < 1.0:
            arr = tracer.traceNow()
            plt.plot([arr[0]], [arr[1]], marker='o', markersize=3, color="blue")
            time.sleep(UPDATE_TIME)

    def test_fEndSlow(self):
        print "> test_fEndSlow"
        tracer = mp.Trace_Circle([0, 4, 0], [0, 0, 0], [0, 0, 1], mp.fEndSlow)
        max = (int)(2.0 / UPDATE_TIME)
        tracer.setTime(1.5, time.time() + 0.5)
        # for indx in range(0, max, 1):
        while tracer.fractionNow() < 1.0:
            arr = tracer.traceNow()
            plt.plot([arr[0]], [arr[1]], marker='o', markersize=3, color="blue")
            time.sleep(UPDATE_TIME)

    def test_fStartSlow(self):
        print "> test_fStartSlow"
        tracer = mp.Trace_Circle([0, 4, 0], [0, 0, 0], [0, 0, 1], mp.fStartSlow)
        max = (int)(2.0 / UPDATE_TIME)
        tracer.setTime(1.5, time.time() + 0.5)
        # for indx in range(0, max, 1):
        while tracer.fractionNow() < 1.0:
            arr = tracer.traceNow()
            plt.plot([arr[0]], [arr[1]], marker='o', markersize=3, color="blue")
            time.sleep(UPDATE_TIME)

    def tearDown(self):
        print "> tearDown"
        plt.show()


    def setUp(self):
        print "> setUp"

if __name__ == '__main__':
    unittest.main()