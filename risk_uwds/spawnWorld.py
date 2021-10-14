#Config (containing links of interest)
from riskHelper import child_link, child_link_robot, exclude_links

#UWDS
import underworlds
from underworlds.helpers import transformations
from underworlds.types import *
from underworlds.tools.edit import *
import underworlds.tools.loader as loader

#ROS
import rospy

#Messages
from gazebo_msgs.msg import ModelStates #In the future, will be used to get position of the coke
from tf2_msgs.msg import TFMessage

#General
import time
import os
import copy
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from tf.transformations import *
import copy

#Managers
from GazeboManager import GazeboManager
from UwdsManager import UwdsManager

#Global variables
all_nodes_human = {}
all_nodes_robot = {}
busy = False
nodes_created = False
gazebo_world = GazeboManager()

def get_coke_pos(data,underworld):
    global coke_pos,table_pos,coke_table_found,coke_found,table_found,moved_on,sinceLastUpdateDurationCoke,lastUpdateTimeCoke
    for i in range(len(data.name)):
        if data.name[i] == "Coke":#there is a way to make this neater
            coke_pos[0] = data.pose[i].position.x
            coke_pos[1] = data.pose[i].position.y
            coke_pos[2] = data.pose[i].position.z
            coke_found = True
        if data.name[i] == "Table":
            table_pos[0] = data.pose[i].position.x
            table_pos[1] = data.pose[i].position.y
            table_pos[2] = data.pose[i].position.z
            table_found = True
    if coke_found and table_found:
        coke_table_found = True
    
    sinceLastUpdateDurationCoke = rospy.get_rostime() - lastUpdateTimeCoke
    #print(sinceLastUpdateDurationCoke.to_sec())

    if sinceLastUpdateDurationCoke.to_sec() > updatePeriodCoke:
        #print("time to update")
        if moved_on:#moved_on
            t = np.eye(4)
            t[0:3,3]=coke_pos
            #print(t)
            underworld.update_coke(t)
            t[0:3,3]=table_pos
            underworld.update_table(t)
            lastUpdateTimeCoke = rospy.get_rostime()

def create_uwds_joints(underworld,special=False):
    """creates joints (long cuboids) between nodes (cubes). 
    If they are already created, then updates their position.
    underworld - object of type Underworld_World for which to update/create links"""
    global moved_on,gazebo_world
    if not underworld.nodes_created:
        i=0
        for node_name in child_link:
            i+=1
            new_node = underworld.create_box(str(i), size = [0.02,0.02,0.02], transformation = gazebo_world.absolute_poses[node_name])
            all_nodes_human[node_name] = new_node#all_nodes_human node_name->node
        underworld.links_human = underworld.link_nodes(gazebo_world, child_link, create=True)
        print("created all human nodes")
        for node_name in child_link_robot:
            i+=1
            new_node = underworld.create_box(str(i), size = [0.02,0.02,0.02], transformation = gazebo_world.absolute_poses[node_name])
            all_nodes_robot[node_name] = new_node#all_nodes_human node_name->node
        underworld.links_robot = underworld.link_nodes(gazebo_world, child_link_robot, create=True, i = 600)

        print("created all robot nodes")
        underworld.nodes_created = True
        moved_on = True
    else:
        for node_name in all_nodes_human:
            #if special:
            #    gazebo_world.absolute_poses[node_name] = np.eye(4)
            all_nodes_human[node_name].transformation = gazebo_world.absolute_poses[node_name]
        transforms = underworld.link_nodes(gazebo_world, child_link, create=False)
        for i,link in enumerate(underworld.links_human):
            link.transformation = transforms[i]
        for node_name in all_nodes_robot:
            all_nodes_robot[node_name].transformation = gazebo_world.absolute_poses[node_name]
        transforms = underworld.link_nodes(gazebo_world, child_link_robot, create=False)
        for i,link in enumerate(underworld.links_robot):
            link.transformation = transforms[i]
        

        underworld.update_world_nodes(all_nodes_human)
        underworld.update_world_nodes(all_nodes_robot)
        underworld.update_world_links(underworld.links_human)
        underworld.update_world_links(underworld.links_robot)

def get_static_frames(data):
    global static_frames, robot_poses_found
    for item in data.transforms:
        child_frame = item.child_frame_id
        parent_name = item.header.frame_id
        transform = item.transform
        static_frames[child_frame]=[parent_name,transform]

def transform2homogeneous(transform):
    """given a transform object with translation and rotation attributes, returns homogeneous matrix that describes the transformation."""
    translation = translation_matrix((transform.translation.x, transform.translation.y, transform.translation.z))
    rot = quaternion_matrix((transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w))
    return concatenate_matrices(translation,rot)

def update_pos(data):
    global solution_found,solution,lastUpdateTime,busy,static_frames,update_period_uwds,gazebo_world
    
    if not(busy):#ensure single thread
        busy = True
        sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
        if sinceLastUpdateDuration.to_sec() < updatePeriod:
            busy = False
            return
        lastUpdateTime = rospy.get_rostime()

        #construct a dictionary tree of child tfs and parent tfs.
        for item in data.transforms:
            child_frame = item.child_frame_id
            parent_name = item.header.frame_id
            gazebo_world.add_node(child_frame, parent_name, transform2homogeneous(item.transform))
            
        #add static_tfs to the tree
        delete_static = []
        for child_frame in static_frames:
            parent_name = static_frames[child_frame][0]
            if gazebo_world.add_node(child_frame,parent_name,transform2homogeneous(static_frames[child_frame][1])):
                delete_static.append(child_frame)

        #delete static frames that have been added to tf tree
        for child_frame in delete_static:
            del static_frames[child_frame]
        
        update_period_uwds = update_period_uwds-1
        
        if update_period_uwds<0 and all(x in gazebo_world.absolute_poses for x in child_link) and all(x in gazebo_world.absolute_poses for x in child_link_robot):
            """Check if all human and robot nodes have been found. If so, update/create everything"""
            update_period_uwds = 100
            print("updating all nodes")
            create_uwds_joints(underworld_real)
            #if underworld_hypo.nodes_created == False:
            create_uwds_joints(underworld_hypo)

        busy = False
    
def read_links(file_path):
    links = open(file_path,"r")
    links_lines = links.readlines()
    relevant_names = []
    for line in links_lines:
        if line.strip() != "":
            relevant_names.append(line.strip())
    return relevant_names

def set_hypothetical_angles(hypothetical_world,gazebo_world):
    """displays the angles in /human_solution inside hypothetical world.
    To view, type "uwds view hypothetical" in a new window.
    This code will only stop running if ctrl+c is pressed to exit or esc is pressed in the underworlds window. """
    parent_name = "actor__spine_03"
    child_name = "actor__upperarm_r"
    #print([x for x in gazebo_world.relative_poses])
    #print("END GAZEBO")
    #print([x for x in all_nodes_human])
    #print("END HUMAN")
    print("setting hypothetical world state")
    try:
        solution = rospy.get_param("/human_solution")
    except KeyError:
        return True
    except KeyboardInterrupt:
        return False
    r1 = R.from_euler("XYZ",solution[0:3]).as_dcm()
    r2 = R.from_euler("XYZ",solution[3:6]).as_dcm()
    R1 = gazebo_world.relative_poses[child_name]
    #print("Homogeneous before: %s"%R1)
    R1[0:3,0:3] = r1
    #print("Homogeneous after: %s"%R1)
    
    
    #print("Homogeneous before: %s"%gazebo_world.absolute_poses[child_name])
    gazebo_world.add_node(child_name,parent_name,R1,update_children=True)#relative_poses[joint_name]
    #print("Homogeneous after: %s"%gazebo_world.absolute_poses[child_name])
    create_uwds_joints(hypothetical_world,special=True)

    #all_nodes_human[joint_name].transformation = r1_original
    #hypothetical_world.update_world_nodes(all_nodes_human)
    print("hypothetical world state set successfully.")
    return True

if __name__ == "__main__":
    
    #Setup - remove later
    os.system('uwds stop')
    os.system('uwds start')
    
    global gazebo_world
    
    real_world_name = "raw"
    hypothetical_world_name = "hypothetical"
    underworld_real = UwdsManager(real_world_name)
    underworld_hypo = UwdsManager(hypothetical_world_name)
    mesh_path = os.path.abspath(os.path.dirname(__file__))+"/meshes/"

    rospy.init_node("underworlds_node")
    #update
    update_period_uwds = 0#this is set in update_pos
    #table and coke
    sinceLastUpdateDurationCoke = rospy.get_rostime()
    lastUpdateTimeCoke = rospy.get_rostime()
    updatePeriodCoke = 0.1
    #robot
    sinceLastUpdateDuration = rospy.get_rostime()
    lastUpdateTime = rospy.get_rostime()
    updatePeriod = 0.001
    #Find coke
    coke_finder = rospy.Subscriber("/gazebo/model_states", ModelStates, get_coke_pos, underworld_real)
    print("WAITING TO FIND COKE!")
    coke_table_found,coke_found,table_found,moved_on = False,False,False,False
    coke_pos = [0,0,0]
    table_pos = [0,0,0]
    while not coke_table_found:
        continue
    print("COKE AND TABLE POSITIONS FOUND!")

    #Load coffee table and coke
    t_t = np.eye(4)
    t_t[0:3,3]=table_pos
    print("loading coke and table meshes")
    underworld_real.load_coffee_table(mesh_path+"cafe_table_new.dae",transformation=t_t)
    underworld_hypo.load_coffee_table(mesh_path+"cafe_table_new.dae",transformation=t_t)
    t_t[0:3,3]=coke_pos
    underworld_real.load_coke(mesh_path+"colourlessCoke.dae",transformation=t_t)#colourlessCoke (unfortunately blender doesnt like the .dae file)
    underworld_hypo.load_coke(mesh_path+"colourlessCoke.dae",transformation=t_t)

    print("gathering static frames")
    #tf frames are split into static_tf and tf. Get static tfs once.
    robot_poses_found = False
    static_frames = {}
    static_tf_subscriber = rospy.Subscriber("/tf_static", TFMessage, get_static_frames)
    print("Collecting static frames")
    time.sleep(1)#todo change this to be until all of the required static frames are found. 
    print("Static frames collected.")
    static_tf_subscriber.unregister()

    link_updater = rospy.Subscriber("/tf", TFMessage, update_pos)#person & robot are moving so continually update them
    print("creating real and hypothetical worlds...")
    try:
        while not underworld_hypo.nodes_created:
            continue
    except KeyboardInterrupt:
        print('Interrupted')
        sys.exit(0)
    print("adjusting the hypothetical world's joint angles")
    loop = True
    while loop:
        loop = set_hypothetical_angles(underworld_hypo,gazebo_world)
        print("setting angles")
        time.sleep(2)

    rospy.spin()

