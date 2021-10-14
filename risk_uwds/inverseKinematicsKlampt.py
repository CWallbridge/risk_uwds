from klampt import *
from klampt import vis
from klampt.model.create import primitives
import numpy as np
#actually solve the IK problem
from klampt.model import ik

q = [x for x in range(12)]


def add_coke(coke_pos=[1,0,2]):
    vis.add("coke can",primitives.sphere(0.025,coke_pos))
    vis.setColor("coke can",1,0,0)#rgb
    vis.add("xOne,yZero",primitives.sphere(0.1,[1,0,0]))
    vis.add("xZero,yOne",primitives.sphere(0.1,[0,1,0]))
    vis.add("xZero,yZero",primitives.sphere(0.1,[0,0,0]))

def visualize():#XYZ
    vis.add("world",world)
    vis.show()

def kill():
    vis.kill()

def update(q):
    world.robot(0).setConfig(q)


def find_intersection(coke_position,robot_id=0,tolerance = 0.05, height_above_coke = 0, num_restarts = 1000):
    """robot_id = 0 for human, 1 for care-o-bot"""
    print("Loading world file")
    world = WorldModel()
    world.loadFile("data/robot.xml")#/home/risk/ws/src/risk_uwds/
    print("Finding intersection")
    #coke_radius = 0.02
    coke_position = coke_position[:]
    #coke_position[1] += coke_radius
    coke_position[2] += height_above_coke
    if robot_id==0:
        obj = ik.objective(world.robot(robot_id).link(8), ref=None,local=[0,0,0], world=coke_position)
    else:
        obj = ik.objective(world.robot(robot_id).link(20), R=[1,0,0,0,0,-1,0,1,0],t=coke_position)
    return ik.solve_global(obj,tol=tolerance,numRestarts=num_restarts, iters=2000),world.robot(robot_id).getConfig()
    #solution_found = solver.solve()
    #print("SOLUTION FOUND: "+str(solver))

