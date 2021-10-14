#general imports
import time
import rospy
import os
import numpy as np
import math
import random
import pysdf

#Rotations & transformations
from scipy.spatial.transform import Rotation as R
from tf.transformations import *

#Messages
from gazebo_msgs.msg import ModelStates#In the future, will be used to get position of the coke
from gazebo_msgs.msg import LinkStates#Used to get person's bones
from tf2_msgs.msg import TFMessage

#Plots
import matplotlib
import matplotlib.pyplot as plt

#Inverse Kinematics
import inverseKinematicsKlampt

#Global Variables
busy = False
all_poses_human=[[],[],[],[]]
all_poses_robot=[[],[],[],[]]
relative_poses = {'gazebo_world': np.eye(4)}
nodes_parent = {'gazebo_world' : 'world'}
coke_pos_found = False
solution_found = False
link_poses_found = False
solution = False #is it possible to knock the cup over, later used to decide if the robot should move.
coke_pos = [0,0,0]#[1.1,0.35,0.80]
link_poses = [[],[],[],[]]#x,y,z,orientation quaternion
coke_height = 0.12
length_of_gripper = 0.1#hardcode length of gripper. this is because there is no tf frame for palm (only the fingers and those are inaccurate)
human_joint_robot_path = "data/human.rob"
careObot_path = "data/careObot.rob"
robot_global_links = ['robot::arm_left_2_link', 'robot::arm_left_4_link', 'robot::arm_left_6_link']
relevant_names_human = []
#lastUpdateTime = rospy.get_rostime()

###Functions for creating .rob FILE(S)###
def TParent(size):
    return "TParent "+("1 0 0   0 1 0   0 0 1   0 0 0   "*size)
    
def parents(size):
    output = "parents "
    for i in range(size):
        output=output+str(i-1)+" "
    return output
    
def axis(number_of_nodes):
    return "axis"+"   1 0 0   0 1 0   0 0 1"*(number_of_nodes//3)#This rotates the axis and not around the axis like euler angles. WHY?
    
def jointtype(number_of_nodes,human_limits=True):
    string =  "jointtype"+(" p p p r r r"*(number_of_nodes//6)) #make all axis have 3 DoF, then remove later
    if not human_limits:
        string+= " p p p"
    return string
    
def q(x,y,z,human_limits=True):#rotation is a list of quaternions
    global length_of_gripper
    e = 0.001 #fudge factor to give a bit of wiggleroom to the solver.
    length_of_upper_arm = ((x[1]-x[0])**2+(y[1]-y[0])**2+(z[1]-z[0])**2)**0.5
    length_of_lower_arm = ((x[2]-x[1])**2+(y[2]-y[1])**2+(z[2]-z[1])**2)**0.5
    rot_1_euler = R.from_dcm(pointsToRotMat([x[0],y[0],z[0]],[x[1],y[1],z[1]])).as_euler('XYZ')
    rot_2_euler = R.from_dcm(pointsToRotMat([x[1],y[1],z[1]],[x[2],y[2],z[2]])).as_euler('XYZ')
    if human_limits:
        string =  "qMin "+str(x[0]-e)+" "+str(y[0]-e)+" "+str(z[0]-e)+" 0 0 0 "+str(-e)+" "+str(length_of_upper_arm-e)+" "+str(-e)+" "+str(-e)+" -3.14 "+str(-e)+"\n"
        string += "qMax "+str(x[0]+e)+" "+str(y[0]+e)+" "+str(z[0]+e)+" 3.14 1.57 3.14 "+str(e)+" "+str(length_of_upper_arm+e)+" "+str(e)+" "+str(e)+" 6.28 "+str(e)+"\n"
        string += "q "   +str(x[0])+  " "+str(y[0])+  " "+str(z[0])+  " "+str(rot_1_euler[0])+" "+str(rot_1_euler[1])+" "+str(rot_1_euler[2])+" 0 "+str(length_of_upper_arm)+" 0 0 0 0"+"\n"
    else:#robot only has 2 joint rotations... around x and z axis. so fudge y axis
        string =  ("qMin "+
                    str(x[0]-e)+" "+str(y[0]-e)+" "+str(z[0]-e)+
                    str(-e)+" -1.57 -1.57 "+
                    str(-e)+" "+str(length_of_upper_arm-e)+" "+str(-e)+" "+
                    str(-e)+" -1.57 -1.57 "+
                    "0 "+str(length_of_lower_arm-e)+" 0 "+
                    str(-e)+" -1.57 -1.57 "+
                    "0 "+str(length_of_gripper-e)+" 0\n")#only allow y actuator to move. This is because arm in default orientation points downwards.
        string +=  ("qMax "+
                    str(x[0]+e)+" "+str(y[0]+e)+" "+str(z[0]+e)+
                    str(e)+" 1.57 1.57 "+
                    str(e)+" "+str(length_of_upper_arm+e)+" "+str(e)+" "+
                    str(e)+" 1.57 1.57 "+
                    "0 "+str(length_of_lower_arm+e)+" 0 "+
                    str(e)+" 1.57 1.57 "+
                    "0 "+str(length_of_gripper+e)+" 0\n")
        string += ("q "   +
                    str(x[0])+  " "+str(y[0])+  " "+str(z[0])+  " "+
                    str(rot_1_euler[0])+" "+str(rot_1_euler[1])+" "  +str(rot_1_euler[2])+
                    " 0 "+str(length_of_upper_arm)+" 0 "+
                    str(rot_2_euler[0])+" "+str(rot_2_euler[1])+" "  +str(rot_2_euler[2])+
                    " 0 "+str(length_of_lower_arm)+" 0 "+
                    "0.785 0 0 "+
                    "0 "+str(length_of_gripper)+" 0\n")
        
        
        #string =  "qMin "+str(x[0]-e)+" "+str(y[0]-e)+" "+str(z[0]-e)+" -3.14 "              +str(-e)+" -3.14 "+str(-e)+" "+str(length_of_upper_arm-e)+" "+str(-e)+" -3.14 "+str(-e)+" -3.14 0 "+str(length_of_lower_arm-e)+" 0\n"#only allow y actuator to move
        #string += "qMax "+str(x[0]+e)+" "+str(y[0]+e)+" "+str(z[0]+e)+" 3.14 "               +str(e) +" 3.14 "+str(e)+" "+str(length_of_upper_arm+e)+" 0 3.14 "+str(e)+" 3.14 0 "+str(length_of_lower_arm+e)+" 0\n"
        #string += "q "   +str(x[0])+  " "+str(y[0])+  " "+str(z[0])+  " "+str(rot_1_euler[0])+" "+str(rot_1_euler[1])+" "  +str(rot_1_euler[2])+" 0 "+str(length_of_upper_arm)+" 0 "+str(rot_2_euler[0])+" "+str(rot_2_euler[1])+" "  +str(rot_2_euler[2])+" 0 "+str(length_of_lower_arm)+" 0\n"
    return string
    
def geometry(x,y,z,human_limits=True):
    length_of_upper_arm = ((x[1]-x[0])**2+(y[1]-y[0])**2+(z[1]-z[0])**2)**0.5
    length_of_lower_arm = ((x[2]-x[1])**2+(y[2]-y[1])**2+(z[2]-z[1])**2)**0.5

    string = ('geometry   ""   ""   "sphere.off" ""   ""   "thincube.off" ""   ""   "sphere.off" "" "" "thincube.off"'+ 
    (' ""   ""   "sphere.off" "" "" "thincube.off" ""   ""   "sphere.off"'*(not human_limits))+"\n")
    string+= 'geomscale 0.05 0.05 0.05 0.17 0.05 '+str(0.95*length_of_upper_arm)+' 0.05 0.05 0.05 0 0 '+str(0.95*length_of_lower_arm)+(
        ' 0 0 0.05 0 0 0.095 0 0 0.05'*(not human_limits))
    return string
    
def joints(number_of_nodes,human_limits=True):
    finished_string = ""
    for joint in range(number_of_nodes//6):#every joint is made from a ballsocket and line
        finished_string+="\n"
        for line in range(3):
            finished_string+="joint weld "+str(line+joint*6)+"\n"
        finished_string+="\n"
        for ballsocket in range(3):
            finished_string+="joint spin "+str(ballsocket+3+joint*6)+"\n"
    if not human_limits:
        finished_string+="\n"
        print("reached here")
        for ballsocket in range(number_of_nodes-3,number_of_nodes):
            finished_string+="joint weld "+str(ballsocket)+"\n"
    return finished_string


def create_rob_file(x,y,z,path,human_limits=True):
    """This function converts the positions of the joints into a .rob file that can be later use by inverse kinematics solver (Klampt)
    Currently, I hard code the joints. Shoulder, Wrist = ball, elbow = revolve and set the links as also joints that cannot have a variable length"""
    number_of_nodes = 6+4+2#shoulder needs start position & orientation (6DoF). It needs a link to elbow 3dof & revolve 1 DoF + 2 additional revolve joints (this is to check the simulation against gazebo for elbow.).
    
    number_of_nodes+= int(not human_limits)*9#Add 9 additional nodes if we are making a robot. I.e. wrist joint and hand. hand has fixed assumed length.
    
    if len(x) != 3 or len(y) != 3 or len(z) != 3:
        print(x,y,z)
        raise Exception('Please provide exactly 3 joint positions! You provided: '+str(len(x))+"  "+str(len(y))+"  "+str(len(z))+" joints")#the joint angles are being worked out from three points. This is bcs the animation's absolute orientation is messed up.
    robot_specification_string = ""
    robot_specification_string += TParent(number_of_nodes)+"\n"
    robot_specification_string += parents(number_of_nodes)+"\n"
    robot_specification_string += axis(number_of_nodes)+"\n"
    robot_specification_string += jointtype(number_of_nodes,human_limits)+"\n"+q(x,y,z,human_limits)+"\n"+geometry(x,y,z,human_limits)+"\n"+joints(number_of_nodes,human_limits)
    
    #write to file
    f = open(path,"w+")#create if file doesn't exist
    f.write(robot_specification_string)
    f.close()

##Helper functions##
def pointsToRotMat(p1,p2):
    #view https://stackoverflow.com/questions/35613741/convert-2-3d-points-to-directional-vectors-to-euler-angles for more
    #p1&p2 - list of format [x,y,z]
    cos = math.cos
    sin = math.sin
    v_x,v_y,v_z=p2[0]-p1[0],p2[1]-p1[1],p2[2]-p1[2]

    r = math.sqrt(v_x**2+v_y**2+v_z**2)
    #TANa1 = (v_z)/(v_x)
    #TANa2 = (v_y)/(v_x^2+v_z^2)
    a1 = np.arctan2(v_z,v_x)
    a2 = np.arctan2(v_y, math.sqrt(v_x**2+v_z**2))

    j = [cos(a1)*cos(a2),sin(a2),sin(a1)*cos(a2)]
    i = [sin(a1),0,-cos(a1)]
    k = [cos(a1)*sin(a2),-cos(a2),sin(a1)*sin(a2)]
    rot_mat = np.transpose(np.array([i,j,k]))
    return rot_mat
    
def transform2homogeneous(transform):
    translation = translation_matrix((transform.translation.x, transform.translation.y, transform.translation.z))
    rot = quaternion_matrix((transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w))
    return concatenate_matrices(translation,rot)
    
def relative_to_absolute(child_frame, debug = False):
    """return a child node's location in world coordinates as a homogeneous matrix."""
    path_to_parent = []
    current_node = child_frame
    while current_node != 'world':
        path_to_parent.append(current_node)
        current_node=nodes_parent[current_node]
    current_transform = np.eye(4)

    for nodes in path_to_parent[::-1]:
        current_transform = concatenate_matrices(current_transform,relative_poses[nodes])#i still don't get why the arguments are this way around
        if debug:
            print(current_transform)
    return current_transform
    
def klampt_to_gazebo(configuration,padding = 1):
    """function that converts from joint angles calculated by klampt to joint angles required by gazebo.
    Gazebo uses 2 angles to describe orientation of arm + node. Currently 3 are used inside klampt.
    padding will dictate how many zeros to add at the end of klampt's configuration."""
    conf_length = len(configuration)
    nr_joints = conf_length//6 #3 weld and 3 spin joints.
    indexes = []
    for i in range(nr_joints):
        index = i*6
        configuration[index+5]*=-1
        indexes.extend([index+4,index+5])

    configuration=[configuration[x] for x in indexes]
    for i in range(padding):
        configuration.append(0)
    return configuration
    
def process_possible_solution(configuration):
    """save the possible solution as 2 homogeneous matrices"""
    #there is a strange bug when the solution is found, but the arm looks wrong inside the underworlds visualizaiton...
    conf_length = len(configuration)
    nr_joints = conf_length//6 #3 weld and 3 spin joints.
    solution = []
    print("configuration: %s" %configuration)
    for i in range(nr_joints):
        index = i*6
        #configuration[index+3] *= -1
        #configuration[index+4] *= -1
        configuration[index+5] *= -1
        solution.extend(configuration[index+3:index+6])
    return solution
    
def update_pos(data):
    """Main function. It collects the relevant tf frames (data) for human and robot. 
    Then tries to find a trajectory for human to knock over the cup and for robot to pick up the cup."""
    global busy,solution_found,solution,all_poses_human,all_poses_robot,coke_pos,coke_height,human_joint_robot_path,relevant_names_human
    #check if it should update
    #sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
    #if sinceLastUpdateDuration.to_sec() < updatePeriod:
        #return
    #lastUpdateTime = rospy.get_rostime()
    if busy:
        return
    busy = True

    #construct a dictionary tree of child tfs and parent tfs.
    for item in data.transforms:
        child_frame = item.child_frame_id
        parent_name = item.header.frame_id
        if parent_name in nodes_parent:
            nodes_parent[child_frame] = parent_name
            relative_poses[child_frame] = transform2homogeneous(item.transform)
    
    for item in data.transforms:
        child_frame = item.child_frame_id
        #print(len(data.transforms))
        if child_frame in relevant_names_human and child_frame in relative_poses:
            #print("%s child" % (child_frame))
            #print("%s header" % (item.header.frame_id))
            true_position = relative_to_absolute(child_frame)
            pos = true_position[0:3,3]
            orientation = R.from_dcm(true_position[0:3,0:3])
            for i in range(len(relevant_names_human)):
                if len(all_poses_human[0]) == i and child_frame == relevant_names_human[i]:
                    all_poses_human[0].append(pos[0])
                    all_poses_human[1].append(pos[1])
                    all_poses_human[2].append(pos[2])
                    all_poses_human[3].append(orientation)#R.from_quat([orientation.x,orientation.y,orientation.z,orientation.w])
    
    if len(all_poses_human) == 4 and len(all_poses_human[0])==3 and len(all_poses_human[1])==3 and len(all_poses_human[2])==3 and len(all_poses_human[3])==3:
        #print("make_plot: %s" %make_plot)
        print("all_poses_human: %s" %all_poses_human[0])
        x_h,y_h,z_h,rotation=all_poses_human[0],all_poses_human[1],all_poses_human[2],all_poses_human[3]
        x_r,y_r,z_r,rotation=all_poses_robot[0],all_poses_robot[1],all_poses_robot[2],all_poses_robot[3]
        #all_poses_human=[[],[],[],[]]
        #all_poses_robot=[[],[],[],[]]
        #if make_plot==0:
            #first time the function is called, create a new .rob file which will be used as input to IK klampt solver.
        create_rob_file(x_h,y_h,z_h,human_joint_robot_path)#write to file the person's limbs positions and set lengths of all joints.
        coke_pos[2]+=coke_height
            #inverseKinematicsKlampt.add_coke(coke_pos)
        #if make_plot >= time_before_solving_equation:
            
        solution_human,configuration = inverseKinematicsKlampt.find_intersection(coke_pos, height_above_coke = 0.15, tolerance = 0.03)#must pass in a copy otherwise everything falls apart.
        processed_config = process_possible_solution(configuration)
        print(processed_config)
        rospy.set_param('/human_solution', processed_config)

        solution_robot,configuration = inverseKinematicsKlampt.find_intersection(coke_pos, robot_id=1, tolerance = 0.03, height_above_coke = 0.07,num_restarts=2000)#0.03
        solution_trajectory,configuration_trajectory = inverseKinematicsKlampt.find_intersection(coke_pos, robot_id=1, tolerance = 0.10, height_above_coke = 0.17)#0.03
        
        
        print("Person can knock over the cup: "+str(solution_human))
        print("Robot can pick up the cup: "+str(solution_robot))
        print("Robot found trajectory to cup: "+str(solution_trajectory))
        solution = solution_human and solution_robot and solution_trajectory
        solution_found = True
        if solution == True:
            print("setting joint angles for robot to pick up the cup!")
            print("ideal configuration: %s" % configuration)
            configuration = klampt_to_gazebo(configuration)
            configuration_trajectory = klampt_to_gazebo(configuration_trajectory)
            rospy.set_param('/script_server/arm_left/arm_to_coke', [configuration])
            rospy.set_param('/script_server/arm_left/arm_above_coke', [configuration_trajectory])
            
    else:
        busy = False
        
def get_coke_pos(data):
    global coke_pos,coke_pos_found
    for i in range(len(data.name)):
        if data.name[i] == "Coke":
            coke_pos[0] = data.pose[i].position.x
            coke_pos[1] = data.pose[i].position.y
            coke_pos[2] = data.pose[i].position.z
            print(coke_pos)
            coke_pos_found = True
            
def get_link_pos(data):
    global link_poses,link_poses_found,careObot_path,robot_global_links
    link_poses = [[],[],[],[]]
    for i in range(len(data.name)):
        if data.name[i] in robot_global_links:
            link_poses[0].append(data.pose[i].position.x)
            link_poses[1].append(data.pose[i].position.y)
            link_poses[2].append(data.pose[i].position.z)
            o = data.pose[i].orientation
            link_poses[3].append(R.from_quat([o.x,o.y,o.z,o.w]))
    
    create_rob_file(link_poses[0],link_poses[1],link_poses[2],careObot_path,human_limits=False)
    link_poses_found = True

def read_links(file_path):
    links = open(file_path,"r")
    links_lines = links.readlines()
    relevant_names = []
    for line in links_lines:
        if line.strip() != "":
            relevant_names.append(line.strip())
    return relevant_names
##Currently Unused function(s)##
#def create_plot(x,y,z,make_plot):
#    fig = plt.figure()
#    ax = plt.axes(projection='3d')
#    ax.scatter(x, y, z, c='r', marker='o')
#    plt.savefig('plots/time'+str(make_plot)+'.png', format='png')
    #plt.show()#doesn't show the graph... my best guess is python doesn't like visualizing from different thread -_- ufff.

def run_get_Tf_Frames():
    
    global relevant_names_human
    
    print("START")
    
    #time_before_solving_equation = 2#20 equals roughly 1 second. should change this to time instead of nr of messages recieved at some point.
    #make_plot = 0
    file_path = "data/rArmLinksMinimal.txt"#path to actor links that need to be plotted/considered when calculating inverse kinematics.
    relevant_names_human = read_links(file_path)
    #relevant_names_robot = read_links(file_path)#implement this later... I would have to translate robot links into global position. this needs to be also done for the person.

    print(relevant_names_human)
    #inverseKinematicsKlampt.visualize()
    #print("Visualized world!")
    #lastUpdateTime = rospy.get_rostime()
    #updatePeriod = 0#0.05
    #print("initialized node")
    #Find coke
    coke_finder = rospy.Subscriber("/gazebo/model_states", ModelStates, get_coke_pos)
    print("WAITING TO FIND COKE!")
    while not coke_pos_found:
        continue
    print("COKE POS FOUND!")
    coke_finder.unregister()
    #calculate if robot can grasp coke
    robot_grabber = rospy.Subscriber("/gazebo/link_states", LinkStates, get_link_pos)
    while not link_poses_found:
        continue
    print("link poses found!")
    robot_grabber.unregister()
    #lets try out this thing with global variables
    link_updater = rospy.Subscriber("/tf", TFMessage, update_pos)
    while not solution_found:
        continue
    link_updater.unregister()
    print("SOLUTION: "+str(solution))

    rospy.set_param("/solution",solution)
    #time.sleep(7)#display solution before destructing

    #destruct
    inverseKinematicsKlampt.kill()
    rospy.signal_shutdown("no longer needed")
    #rospy.spin()
    
if __name__ == "__main__":
    
    rospy.init_node("getTFFrames")
    run_get_Tf_Frames()
