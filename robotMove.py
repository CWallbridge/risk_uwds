import os
os.system('python inverseKinematics/getTfFrames.py')#I'm not proud of this implementation either \_(._.)_/ todo fix it.

#This script used to be located in simul_risk directory as the main script.
import rospy
#rospy = getTfFrames.rospy

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest, GetWorldProperties#spawn coke & table
from geometry_msgs.msg import Pose, PoseStamped
#from pyquaternion import Quaternion#Might need to account for rotation when picking up the object
from scipy.spatial.transform import Rotation as R
import numpy as np
from simple_script_server import script#Used to move arm
from gazebo_msgs.msg import ModelState,ModelStates
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import LinkStates#Used to get global position of arm
import time #Need this to make script sleep. If publishing too quickly after instantiating publisher it doesnt work. WHY?
import math
#Var definitions
#Dictionary containing all relevant paths
object_locations = {"Table":"/home/risk/.gazebo/models/cafe_table/model.sdf","Coke":"/home/risk/.gazebo/models/coke_can/model.sdf","Chair":"/home/risk/ws/dev/share/cob_gazebo_objects/objects/chair_ikea_borje.urdf"}

attached = False
#rospy.init_node("main")#do not uncomment me
name_of_arm = 'robot::gripper_left_finger_1_link'#'robot::arm_left_7_link'#'robot::gripper_left_finger_1_link'#
cokeState = ModelState()
cokeState.model_name = 'Coke'
original_quaternion = None#Change me
robot_pos,robot_orientation = 0,0
#starting_pos = pose()
#Classes & Functions


perform_scenario = open("inverseKinematics/solution.txt","r").read().strip()=="True"#getTfFrames.solution
if not perform_scenario:
    print("The person cannot knock over the cup! Doing nothing.")
class MyScript(script):
    def Initialize(self):
        pass
        #rospy.loginfo("Initializing all components...")
    def Run(self):
        rospy.loginfo("Grasping an object...")
        self.sss.move("arm_right","arm_to_side")
	self.sss.move("arm_left","arm_to_side")
        #self.sss.move("gripper_left","open")
        #self.sss.move("arm_left","arm_to_coke")
        #self.sss.move("arm_left","lift_coke")
    def liftCoke(self):
        self.sss.move("gripper_left","open")
        self.sss.move("arm_left","arm_to_coke")
        self.sss.move("arm_left","lift_coke")
class pose:
   def __init__(self,x=0,y=0,z=0):
      self.x = x
      self.y = y
      self.z = z

def make_pose(position,quaternion):
    #make a pose of object given xyz, and quaternion
    object_pose = Pose()
    object_pose.position.x = float(position[0])
    object_pose.position.y = float(position[1])
    object_pose.position.z = float(position[2])
    object_pose.orientation.x = quaternion[0]
    object_pose.orientation.y = quaternion[1]
    object_pose.orientation.z = quaternion[2]
    object_pose.orientation.w = quaternion[3]
    return object_pose
def spawn_object(name,position=[1,0,0],quaternion=[0,0,0,1],model_type = "sdf"):
    #Read xml file
    f = open(object_locations[name])
    
    # spawn new model (Chair)
    req = SpawnModelRequest()
    req.model_name = name
    req.model_xml = f.read()
    f.close()
    req.initial_pose = make_pose(position,quaternion)

    rospy.wait_for_service('/gazebo/spawn_'+model_type+'_model',30)
    srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_'+model_type+'_model', SpawnModel)
    res = srv_spawn_model(req)

def armUpdate(data):
    for i in range(len(data.name)):
        if data.name[i] == name_of_arm:
            pos = data.pose[i].position
            orientation = data.pose[i].orientation
            #print(pos)
            updateObject(pos,orientation=orientation,check=True)#,orientation=orientation


def updateObject(pos,orientation=0,check=False,radius=0.5):
    global attached
    global pub
    global original_quaternion
    global prev_distance
    if check and not(attached):#Check if an object is within a certain radius. or if it should stick to the arm.
        m = cokeState.pose.position
        d = math.sqrt((m.x-pos.x)**2 + (m.y-pos.y)**2 + (m.z-pos.z)**2)
        
	print("arm from coke: "+str(d))
        if d < radius:
            print("ATTACHED")
            attached = True
    else:#if attached
        pose_offset = np.array([-0.015,-0.07,-0.05])#0.087,0.203,-0.06[0,0.06,0.04]
        
        rotation = R.from_quat([orientation.x,orientation.y,orientation.z,orientation.w])#newer scipy this would be as_matrix()!
        
        #print(rotation.as_euler('xyz',degrees = True))
        rotation = rotation.as_dcm()
        pose_offset = np.matmul(rotation,pose_offset)
        
        cokeState.pose.position.x = pos.x+pose_offset[0]
        cokeState.pose.position.y = pos.y+pose_offset[1]
        cokeState.pose.position.z = pos.z+pose_offset[2]
        
        cokeState.pose.orientation = orientation
        #print(orientation)
        #print(pos.x,pos.y,pos.z)
        #if orientation !=0 :
        # cokeState.pose.orientation = orientation
        pub.publish(cokeState)
def move_to_table():
    pos_reach_th = 0.5
    #Robot start position is [-8,-4,0]
    p1 = [8,0,0]
    p1_finish = [0,-4,0]
    p2 = [8,4,0]
    p2_finish = [0,0,0]
    goal_pos1 = make_pose(p1,[0,0,0,1])
    move_to = PoseStamped()
    move_to.pose = goal_pos1
    move_to.header.frame_id = "odom_combined"
   
    print(move_to)
    time.sleep(4)
    pub1.publish(move_to)
    while math.sqrt((robot_pos.x-p1_finish[0])**2 + (robot_pos.y-p1_finish[1])**2) > pos_reach_th:#robot sometimes goes rouge...
        time.sleep(1)
        print("waiting for robot to reach pos 1. dist: "+str(math.sqrt((robot_pos.x-p1_finish[0])**2 + (robot_pos.y-p1_finish[1])**2)))
	print("ROBOT X: "+str(robot_pos.x))
	print("goal X: " +str(p1_finish[0]))
	print("ROBOT Y: "+str(robot_pos.y))
	print("goal Y: " +str(p1_finish[1]))

    move_to.pose = make_pose(p2,[0,0,0,1])
    move_to.header.frame_id = "odom_combined"
    move_to.header.seq = 1
    pub1.publish(move_to)
    while math.sqrt((robot_pos.x-p2_finish[0])**2 + (robot_pos.y-p2_finish[1])**2) > pos_reach_th:
        time.sleep(1)
        print("waiting for robot to reach pos 2. dist: "+str(math.sqrt((robot_pos.x-p2_finish[0])**2 + (robot_pos.y-p2_finish[1])**2)))
	print("ROBOT X: "+str(robot_pos.x))
	print("goal X: " +str(p2_finish[0]))
	print("ROBOT Y: "+str(robot_pos.y))
	print("goal Y: " +str(p2_finish[1]))
    print("Final body position reached... waiting 5 seconds before grasping")
    time.sleep(5)
def robot_update(data):
    global robot_pos,robot_orientation
    for i in range(len(data.name)):
        if data.name[i] == "robot":
            robot_pos = data.pose[i].position
            robot_orientation = data.pose[i].orientation
        if data.name[i] == "Coke":
            cokeState.pose = data.pose[i]
            
#Actual script
if perform_scenario:
    #Check if the can gets close ot robot's  arm. if so, attach it
    rospy.Subscriber("/gazebo/link_states", LinkStates, armUpdate)
    rospy.Subscriber("/gazebo/model_states", ModelStates, robot_update)#Used for getting robot's position. & Updating coke position
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    pub1 = rospy.Publisher('/docker_control/move_base_linear_simple/goal', PoseStamped, queue_size=10)#Changed from /move_base_simple/goal
    #Move arms to side so robot can pass through the hallway
    SCRIPT = MyScript()
    SCRIPT.Start()
    #Move robot's base to start position
    print("Moving the robot to table")
    move_to_table()
    #Move robot's arm to grab the coke
    print("Moving the arm to coke")
    SCRIPT.liftCoke()
    rospy.spin()

