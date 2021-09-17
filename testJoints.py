import rospy
from simple_script_server import script
class MyScript(script):
    def Initialize(self):
        rospy.loginfo("Initializing all components...")
    def Run(self):
        rospy.loginfo("Initialized.")
    def go_to_table(self):
        self.sss.move("arm_right","arm_to_side")
        self.sss.move("arm_left","arm_to_side")

    def lift_coke(self):
        self.sss.move("gripper_left","open")
        self.sss.move("arm_left","arm_to_coke")
        self.sss.move("arm_left","lift_coke")
    def test(self):
        self.sss.move("arm_left","lift_coke")


SCRIPT = MyScript()
SCRIPT.Start()

    #Move robot's base to start position
    #print("Moving the robot to table")
    #move_to_table()
SCRIPT.test()
rospy.spin()

