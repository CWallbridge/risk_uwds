joint_string = """
joint weld 0
joint weld 1
joint weld 2

joint spin 3
joint spin 4
joint spin 5

joint weld 6
joint weld 7
joint weld 8

joint spin 9
joint spin 10
joint spin 11
"""
#general imports
import time
import rospy
import os
import numpy as np
import math
#Rotations
from scipy.spatial.transform import Rotation as R
#Messages
from gazebo_msgs.msg import ModelStates#In the future, will be used to get position of the coke
from gazebo_msgs.msg import LinkStates#Used to get person's bones
#Plots
import matplotlib
import matplotlib.pyplot as plt
#Inverse Kinematics
import inverseKinematicsKlampt as inverseKinematicsKlampt#inverseKinematics.


###Functions for creating .rob FILE###
def TParent(size):
    return "TParent "+("1 0 0   0 1 0   0 0 1   0 0 0   "*size)
def parents(size):
    output = "parents "
    for i in range(size):
        output=output+str(i-1)+" "
    return output
def axis():
    return "axis"+"   1 0 0   0 1 0   0 0 1"*4#This rotates the axis and not around the axis like euler angles. WHY?
def jointtype():
    return "jointtype p p p r r r p p p r r r"#make all axis have 3 DoF, then remove later
def q(x,y,z,rotation):#rotation is a list of quaternions
    e = 0.001 #fudge factor to give a bit of wiggleroom.
    
    length_of_upper_arm = ((x[1]-x[0])**2+(y[1]-y[0])**2+(z[1]-z[0])**2)**0.5
    rot_1_euler = rotation[0].as_euler('XYZ')
    rot_2_euler = rotation[1].as_euler('XYZ')
    string =  "qMin "+str(x[0]-e)+" "+str(y[0]-e)+" "+str(z[0]-e)+" -3.14 -3.14 -3.14 "+str(length_of_upper_arm-e)+" 0 0 -3.14 -3.14 -3.14"+"\n"
    string += "qMax "+str(x[0]+e)+" "+str(y[0]+e)+" "+str(z[0]+e)+" 6.28 6.28 6.28 "+str(length_of_upper_arm+e)+" 0 0 6.28 6.28 6.28"+"\n"
    string += "q "+str(x[0])+" "+str(y[0])+" "+str(z[0])+" "+str(rot_1_euler[0])+" "+str(rot_1_euler[1])+" "+str(rot_1_euler[2])+" "+str(length_of_upper_arm)+" 0 0 0 0 0"+"\n"
    return string
def geometry(length_of_upper_arm):
    string = 'geometry   ""   ""   "../objects/sphere.off" ""   ""   "../objects/thincube.off" ""   ""   "../objects/sphere.off" "" "" "../objects/thincube.off"'+"\n"
    string+= 'geomscale 0.05 0.05 0.05 0.05 0.05 '+str(0.95*length_of_upper_arm)+' 0.05 0.05 0.05 0 0 0.3'
    return string
def joints():
    return joint_string
def convert_to_robot(x,y,z,rotation):
    """This function converts the positions of the joints into a .rob file that can be later use by inverse kinematics solver (Klampt)
    Currently, I hard code the joints. Shoulder, Wrist = ball, elbow = revolve."""
    f = open("/home/risk/ws/src/risk_uwds/inverseKinematics/IKwithKlampt/klampt/data/robots/new_rob.rob","w+")#create if file doesn't exist
    number_of_nodes = 6+4+2#shoulder needs start position & orientation (6DoF). It needs a link to elbow 3dof & revolve 1 DoF + 2 additional revolve joints (this is to check the simulation against gazebo for elbow.).
    if len(x) != 2:
        raise Exception('Plese provide exactly 2 joint positions!')#only use shoulder and elbow for now
    robot_specification_string = ""
    robot_specification_string += TParent(number_of_nodes)+"\n"
    robot_specification_string += parents(number_of_nodes)+"\n"
    robot_specification_string += axis()+"\n"
    robot_specification_string += jointtype()+"\n"+q(x,y,z,rotation)+"\n"+geometry(((x[1]-x[0])**2+(y[1]-y[0])**2+(z[1]-z[0])**2)**0.5)+"\n"+joints()
    f.write(robot_specification_string)
    f.close()
#update the klampt simulation in real time.
def update_pos(data):
    global make_plot,solution_found,solution
    x,y,z,rotation=[],[],[],[]#store all points considered interesting
    for i in range(len(data.name)):
        if data.name[i] in relevant_names:#always appear in correct order (shoulder, upper arm, lower arm, hand)
            pos = data.pose[i].position
            orientation = data.pose[i].orientation
            x.append(pos.x)
            y.append(pos.y)
            z.append(pos.z)
            rotation.append(R.from_quat([orientation.x,orientation.y,orientation.z,orientation.w]))
    #print(len(x))
    if make_plot==0:
        convert_to_robot(x,y,z,rotation)#write to file the person's limbs positions and set lengths of all joints. This will only update in the next iteration of the simulation because I'm not sure how to reload the world
        inverseKinematicsKlampt.add_coke(coke_pos)
        
    if 1<make_plot<time_before_solving_equation:#there are roughly 300 messages sent per second. WAit 10 seconds before trying to find a solution.
        #update the Klampt simulation with what gazebo publishes.
        e = rotation[0].as_euler('XYZ')#Important. captial XYZ means local coordinate frame.
        #rotation[0] is shoulder in global coordinate frame. however, we want to express it using local euler angles, because we have model a ballsocket joint as 3 individual spin joints.
        #spin joints are defined to rotate around x,y, and z. 
        e1 = (rotation[0].inv()*rotation[1]).as_euler('XYZ')
        length_of_upper_arm = ((x[1]-x[0])**2+(y[1]-y[0])**2+(z[1]-z[0])**2)**0.5

        inverseKinematicsKlampt.q[0:12] = x[0],y[0],z[0],e[0],e[1],math.pi+e[2],length_of_upper_arm,0,0,e1[0],e1[1],e1[2]#x[0],y[0],z[0],e[0],e[1],e[2],length_of_upper_arm,0,0,e1[0]
        #IMPORTANT: I'm not sure why math.pi needs to be added to e[2] to make the person's arm align with Gazebo. It looks about right, but \_(0_0)_/ I really don't know why.
        #perhaps something about equivalent euler angles having a period of pi. What's strange is that the solution found by solver does not have this issue.
        inverseKinematicsKlampt.update(inverseKinematicsKlampt.q)
    if make_plot == time_before_solving_equation:
        solution = inverseKinematicsKlampt.find_intersection(coke_pos,tolerance = 0.05)
        solution_found = True
    
    make_plot +=1


def get_coke_pos(data):
    global coke_pos,coke_pos_found
    for i in range(len(data.name)):
        if data.name[i] == "Coke":
            coke_pos[0] = data.pose[i].position.x
            coke_pos[1] = data.pose[i].position.y
            coke_pos[2] = data.pose[i].position.z
            coke_pos_found = True

def read_links(file_path):
    links = open(file_path,"r")
    links_lines = links.readlines()
    relevant_names = []
    for line in links_lines:
        if line.strip() != "":
            relevant_names.append(line.strip())
    return relevant_names
#Unused function(s)
def create_plot(x,y,z,make_plot):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z, c='r', marker='o')
    plt.savefig('plots/time'+str(make_plot)+'.png', format='png')
    #plt.show()#doesn't show the graph... my best guess is python doesn't like visualizing from different thread -_- ufff.



#Var definitions
coke_pos_found = False
solution_found = False
solution = False #is it possible to knock the cup over, later used to decide if the robot should move.
time_before_solving_equation = 300#300 equals roughly 1 second. should change this to time instead of nr of messages recieved at some point.
coke_pos = [0,0,0]#[1.1,0.35,0.80]
make_plot = 0
file_path = "inverseKinematics/rArmLinksMinimal.txt"#path to actor links that need to be plotted/considered when calculating inverse kinematics.
output_file = "inverseKinematics/solution.txt"#write whether a solution was found inside output.txt. This is hacky!
relevant_names = read_links(file_path)

inverseKinematicsKlampt.visualize()
print("Visualized world!")
rospy.init_node("main")
print("initialized node")
coke_finder = rospy.Subscriber("/gazebo/model_states", ModelStates, get_coke_pos)#Not fully implemented. use hardcoded-pos for now.
print("WAITING TO FIND COKE!")
while not coke_pos_found:
    continue
print("COKE POS FOUND!")
coke_finder.unregister()
#lets try out this thing with global variables
link_updater = rospy.Subscriber("/gazebo/link_states", LinkStates, update_pos)
while not solution_found:
    continue
link_updater.unregister()
print("SOLUTION: "+str(solution))

with open(output_file,"w+") as f:
    f.write(str(solution))
time.sleep(7)#display solution before destructing

#destruct
inverseKinematicsKlampt.kill()
rospy.signal_shutdown("no longer needed")
#rospy.spin()

#arm restrictions
#seperate out fully code (simulation environment/ robot movement)
#robot already in position.
#UDWs?