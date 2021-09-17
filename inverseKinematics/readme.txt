getTfFrames.py is the main script that solves if it is possible for the human to knock the cup over.
inverseKinematicsKlampt.py visualizes the solution and contains implementaiton of IK solver.
createRobotGrasp.py - uses an ik solver that creates joint angles for the robot to grasp the cup.





Notes on implementation.
By default Gazebo does not publish the animation to /tf frames topic.
A plugin would be required to do this:
https://answers.gazebosim.org//question/21083/actors-in-ros-melodic-gazebo-9/

However, they seem to be published to /gazebo/actor... topic.
Thus I write a python script to pick them up!(getTfFrames.py)

Next, inverse kinematics. There is a package "tinyik" that solves inverse kinematics and has nice
visualization... that only works with python3.
Quick fix - use python3 to visualize and 2 at test time.
PYTHONPATH=/home/risk/ws/dev/lib/python2.7/site-packages:/opt/ros/melodic/lib/python2.7/dist-packages

add at the front
/usr/lib/python3/dist-packages

so
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH


This is good, but there are two limitations:
1)you can only rotate around 1 axis.
	This can be overcome by using an additional joint which rotates around a different axis.
2)There is no ways of specifying limits of the rotation. (This is required or would def be of high prority)
	Thus I check out Klampt.

Klampt is a robotics simulator complete with its own inside environment etc.
Setting up a world is not trivial. It is similar to gazebo.

	 
