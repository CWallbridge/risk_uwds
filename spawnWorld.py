#Config (containing links of interest)
from helper import child_link, child_link_robot, exclude_links
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
#Setup - remove later
os.system('uwds stop')
os.system('uwds start')


class Underworld_World:
    """Class that manages the underworlds world."""
    def __init__(self,world_name):
        self.ctx = underworlds.Context("spawnWorld")
        self.target_world = self.ctx.worlds[world_name]
        self.world_name = world_name
        self.model_loader = loader.ModelLoader()
        self.coffee_table = None
        self.coke = None
        self.nodes_created = False
        self.links_human = None
        self.links_robot = None
    def update_world_nodes(self,nodes):
        """takes in a dictionary"""
        try:
            for key in nodes:
                self.target_world.scene.nodes.update(nodes[key])
        except:
            time.sleep(1)
            self.update_world_nodes(nodes)
    def update_world_links(self,links):
        """takes in a list"""
        try:
            for node in links:
                self.target_world.scene.nodes.update(node)
        except:
            time.sleep(1)
            self.update_world_links(links)
    def create_box(self, node_name, size = [1,1,1], transformation = np.eye(4)):
        try:
            mesh_id = create_box_mesh(self.world_name, size[0],size[1],size[2])
            print("Mesh Id: %s" %mesh_id)
            print("Node name: %s" %node_name)
            create_mesh_node(self.world_name, node_name, mesh_id, None)
            a = self.target_world.scene.nodebyname(node_name)[0]
            a.transformation = transformation
            self.target_world.scene.nodes.update(a)
            return a
        except:
            time.sleep(1)
            return self.create_box(node_name,size = size, transformation = transformation)
    def load_mesh(self,file_name):
        root_node = self.target_world.scene.nodebyname("root")[0]

        return self.model_loader.load(file_name, ctx = self.ctx, world=self.world_name, root = root_node)#this returns a mix of meshes and entities.... WHY?
    def load_coffee_table(self,mesh_name,transformation = np.eye(4)):
        try:
            meshes = self.load_mesh(mesh_name)
            c_node = self.target_world.scene.nodebyname("SketchUp")[0]#self.coffee_table
            self.org_table_t = c_node.transformation
            self.coffee_table = c_node
            c_node.transformation = np.matmul(transformation,c_node.transformation)
            self.target_world.scene.nodes.update(c_node)
        except:
            self.load_coffee_table(mesh_name,transformation = transformation)
    def load_coke(self,mesh_name,transformation = np.eye(4)):
        try:
            meshes = self.load_mesh(mesh_name)
            c_node = self.target_world.scene.nodebyname("CokeCan")[0]
            self.org_coke_t = c_node.transformation
            self.coke = c_node
            c_node.transformation = np.matmul(transformation,c_node.transformation)
            self.target_world.scene.nodes.update(c_node)
        except:
            self.load_coke(mesh_name,transformation = transformation)
    def update_coke(self,transformation = np.eye(4)):
        c_node = self.target_world.scene.nodebyname("CokeCan")[0]
        c_node.transformation = np.matmul(transformation,self.org_coke_t)
        self.target_world.scene.nodes.update(c_node)
    def update_table(self,transformation = np.eye(4)):
        c_node = self.target_world.scene.nodebyname("SketchUp")[0]
        c_node.transformation = np.matmul(transformation,self.org_table_t)
        self.target_world.scene.nodes.update(c_node)
    def link_nodes(self, gazebo_world, node_links, create=True, i = 100):
        """Function that creates links between nodes in underworlds. 
        gazebo_world: a gazebo_world instance containing all nodes in node_links
        node_links: dict with structure node_links[child_name] = parent_name between which links should be created.
        create: if true Will create and return a list of nodes with meshes in underworlds. 
        If false, will return a list transforms between nodes
        """
        links = []
        transforms = []
        for key in node_links:
            if key not in exclude_links:
                i+=1
                t1 = gazebo_world.absolute_poses[key]
                t2 = gazebo_world.absolute_poses[node_links[key]]
                
                p1 = t1[0:3,3]
                p2 = t2[0:3,3]
                rotation = pointsToRotMat(p1,p2)

                t_final = np.eye(4)
                t_final[0:3,0:3]=rotation
                t_final[0:3,3]= (p1+p2)/2#since meshes do not start at a corner and instead start at centre of mass, their origin needs to be halfway between two centres
                
                length = 0.95 * np.linalg.norm(p1-p2)
                size = (0.01,length,0.01)
                
                if create:
                    new_node = self.create_box(str(i+200), size = size, transformation=t_final)
                    links.append(new_node)
                else:
                    transforms.append(t_final)
        if create:
            return links
        else:
            return transforms
class Gazebo_World:
    """Class that manages gazebo world. Mostly used for getting the absolute position of a tf frame"""
    def __init__(self):
        self.absolute_poses = {'gazebo_world': np.eye(4), 'world' : np.eye(4)}
        self.relative_poses = {'gazebo_world': np.eye(4), 'world' : np.eye(4)}
        self.nodes_parent = {'gazebo_world' : 'world', 'world' : 'world'}
    def get_path_to_parent(self,child_name):
        path_to_parent = []
        current_node = child_name
        while current_node != 'world':
            path_to_parent.append(current_node)
            current_node=self.nodes_parent[current_node]
        return path_to_parent
    def relative_to_absolute(self, child_frame, debug = False):
        """return a child node's location in world coordinates as a homogeneous matrix."""
        path_to_parent = self.get_path_to_parent(child_frame)
        current_transform = np.eye(4)
        for nodes in path_to_parent[::-1]:
            current_transform = concatenate_matrices(current_transform,self.relative_poses[nodes])
            if debug:
                print(current_transform)
        return current_transform
    def update_children(self,child_name):
        """Recursively update nodes"""
        print("recursively updating nodes: "+str(child_name))
        for node in self.relative_poses:
            if self.nodes_parent[node] == child_name:#check if a node is pointing to the child node. if so update it and all its children.
                self.absolute_poses[node] = self.relative_to_absolute(node)
                return self.update_children(node)

        
        
        
    def add_node(self,child_name,parent_name,relative_transform,update_children=False):
        """Adds a child node with a relative transform to the class. Returns true if the addition of the node was succesful. Otherwise returns false.
        When inserting an existing node, children of that node need to be updated for it to display correctly on gazebo."""
        if parent_name in self.nodes_parent:
            self.nodes_parent[child_name] = parent_name
            self.relative_poses[child_name] = relative_transform
            self.absolute_poses[child_name] = self.relative_to_absolute(child_name)
            if update_children:
                self.update_children(child_name)
            return True
        else:
            return False
    def generate_subtree(self,node_names):
        pass
        """
        This function would be useful because currently one needs to specify how
        nodes are connected to each other when drawing an object in underworlds.

        Takes in a list - node_names and
        returns a dictionary that contains 
        all nodes in node_names and links to the world frame.
        The dictionary has child names as keys and parent names as values. 
        """


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
    global moved_on
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
    global solution_found,solution,lastUpdateTime,busy,static_frames,update_period_uwds
    
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
            if underworld_hypo.nodes_created == False:
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

def set_hypothetical_angles(hypothetical_world,gazebo_world):
    """displays the angles in /human_solution inside hypothetical world.
    To view, type "uwds view hypothetical" in a new window.
    This code will only stop running if ctr+c is pressed to exit. """
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
real_world_name = "test"
hypothetical_world_name = "hypothetical"
underworld_real = Underworld_World(real_world_name)
underworld_hypo = Underworld_World(hypothetical_world_name)
gazebo_world = Gazebo_World()
mesh_path = os.path.abspath(os.path.dirname(__file__))+"/Meshes/"


all_nodes_human = {}
all_nodes_robot = {}

busy = False
nodes_created = False

    
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
underworld_real.load_coffee_table(mesh_path+"cafe_table_new.dae",transformation=t_t)
underworld_hypo.load_coffee_table(mesh_path+"cafe_table_new.dae",transformation=t_t)
t_t[0:3,3]=coke_pos
underworld_real.load_coke(mesh_path+"colourlessCoke.dae",transformation=t_t)#colourlessCoke (unfortunately blender doesnt like the .dae file)
underworld_hypo.load_coke(mesh_path+"colourlessCoke.dae",transformation=t_t)


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
while not underworld_hypo.nodes_created:
    continue
print("adjusting the hypothetical world's joint angles")
loop = True
while loop:
    loop = set_hypothetical_angles(underworld_hypo,gazebo_world)
    print("setting angles")
    time.sleep(2)

#rospy.spin()
#os.system('uwds stop')