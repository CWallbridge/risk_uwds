from klampt import *
from klampt import vis
from klampt.model.create import primitives

#actually solve the IK problem
from klampt.model import ik
world = WorldModel()
world.loadFile("/home/risk/ws/src/risk_uwds/inverseKinematics/IKwithKlampt/klampt/data/allNew.xml")


def reload_world():
    global world
    world = WorldModel()
    world.loadFile("/home/risk/ws/src/risk_uwds/inverseKinematics/IKwithKlampt/klampt/data/allNew.xml")

def add_coke(coke_pos=[1,0,2]):
    vis.add("coke can",primitives.sphere(0.05,coke_pos))
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

def find_intersection(coke_position,tolerance = 0.05):
    print("FINDING INTERSECTION")
    obj = ik.objective(world.robot(0).link(8), ref=None,local=[0,0,0], world=coke_position)#local=[0,0,0], world=[0,0,1]. It fixes a local point to a global coordinate frame. when is local point useful?
    return ik.solve_global(obj,tol=tolerance)
    #solution_found = solver.solve()
    #print("SOLUTION FOUND: "+str(solver))

q = world.robot(0).getConfig()