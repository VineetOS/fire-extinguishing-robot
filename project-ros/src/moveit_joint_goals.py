from os import wait
import sys
import copy
from moveit_commander import move_group
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_commander',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
move_group  = moveit_commander.MoveGroupCommander(group_name)


display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

#### get basic information of robot

plan_frame = move_group.get_planning_frame()
print('planning_frame  =',plan_frame)

eff_link = move_group.get_end_effector_link()

group_names = robot.get_group_names()
#print('Group Names  =',group_names)

current_state = robot.get_current_state()
#print('current state   =',current_state)

while not rospy.is_shutdown():
        
    joint_goals = move_group.get_current_joint_values()

    joint_goals[0]= np.random.random() *2
    joint_goals[1]=np.random.random()
    joint_goals[2]= np.random.random() *2
    joint_goals[3]= np.random.random()
    joint_goals[4]= np.random.random()
    joint_goals[5]=np.random.random()
    print('New goal generated')
    print(joint_goals)

    move_group.go(joint_goals,wait=True)

    move_group.stop()
    print('Goal Reached')
    rospy.sleep(0.1)


