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

class UwdsManager:
    """Class that manages the underworlds world."""
    
    def __init__(self,world_name):
        self.ctx = underworlds.Context("UwdsManager")
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
            print("Exception loding coffee table")
            time.sleep(1)
            self.load_coffee_table(mesh_name,transformation = transformation)
            
    def load_coke(self,mesh_name,transformation = np.eye(4)):
        try:
            meshes = self.load_mesh(mesh_name)
            try:
                c_node = self.target_world.scene.nodebyname("CokeCan")[0]
            except:
                time.sleep(1)
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
    
    def pointsToRotMat(self,p1,p2):
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
                rotation = self.pointsToRotMat(p1,p2)

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
