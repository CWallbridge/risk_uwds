import rospy
from simple_script_server import script#Used to move arm

class RobotCommand(script):
	
    def Initialize(self):
        rospy.loginfo("Initializing all components...")
        
    def Run(self):
        rospy.loginfo("Initialized.")
        
    def go_to_table(self):
        self.sss.move("arm_right","arm_to_side")
        self.sss.move("arm_left","arm_to_side")

    def larm_to_coke(self):
        self.sss.move("gripper_left","open")
        self.sss.move("arm_left","arm_above_coke")
        self.sss.move("arm_left","arm_to_coke")
        attached=True
        print("ATTACHING")
        return attached
        
    def lift_coke(self):
        self.sss.move("arm_left","arm_above_coke")
