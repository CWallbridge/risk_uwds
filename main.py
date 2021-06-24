#!/usr/bin/env python

import underworlds
import underworlds.server
from underworlds.tools.loader import ModelLoader
from underworlds.tools.spatial_relations import *
from underworlds.tools.edit import *
from underworlds.helpers.geometry import get_world_transform
from underworlds.helpers.transformations import *

import rospy
import tf
import numpy
import math
import random
import time
import sys
import os
import csv

#Pyswarms IK solver https://pyswarms.readthedocs.io/en/latest/examples/usecases/inverse_kinematics.html

if __name__ == "__main__":
	
	rospy.init_node("risk_uwds");
	
	tl = tf.TransformListener()
	
	#https://stackoverflow.com/questions/18184848/calculate-pitch-and-yaw-between-two-unknown-points
	#Calculate pitch and yaw between two points: 
	#double dX = first_location.getX() - second_location.getX();
	#double dY = first_location.getY() - second_location.getY();
	#double dZ = first_location.getZ() - second_location.getZ();
	
	#double yaw = Math.atan2(dZ, dX);
	
	#double pitch = Math.atan2(Math.sqrt(dZ * dZ + dX * dX), dY) + Math.PI;

	#http://wiki.ros.org/gazebo2rviz
	
	pass
