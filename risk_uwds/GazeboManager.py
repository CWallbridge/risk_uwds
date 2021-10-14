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


class GazeboManager:
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
