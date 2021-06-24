from __future__ import print_function,division
import numpy as np
import time
from klampt import *
from klampt.math import vectorops,so3,se3
from klampt import vis
from klampt.model.create import primitives
from IPython.display import clear_output
from klampt.vis.ipython import EditConfig,EditPoint,EditTransform

#actually solve the IK problem
from klampt import IKObjective,IKSolver
from klampt.model import ik

world = WorldModel()
world.loadFile("inverseKinematics/IKwithKlampt/klampt/data/allNew.xml")
vis.add("world",world)

vis.add("cokeCan",primitives.sphere(0.5,[1,0,2]))
vis.setColor("cokeCan",1,0,0)#rgb


vis.show()

q = world.robot(0).getConfig()
time.sleep(2)
print("\n\n\n")
print("PRINTING Q: ")
print(q)


obj = ik.objective(world.robot(0).link(8), ref=None,local=[0,0,0], world=[0,0,1])#local=[0,0,0], world=[0,0,1]. It fixes a local point to a global coordinate frame. when is this useful?
print("solving")
#solver = ik.solver(obj)
#solver.solve()
#print(world.robot(0).getConfig())
#time.sleep(200)
#Controls:
#left mouse click to rotate the view
#right click or ctrl+click to pan the view
#mouse wheel or shift+click to zoom the view
"""
, iters=100, tol=1e-3, activeDofs=None,
             numRestarts = 10, feasibilityCheck = None, startRandom = False 



             for i in range(1):
    q[5] = i*0.01-1
    world.robot(0).setConfig(q)
    #vis.update()
    time.sleep(0.1)
             """