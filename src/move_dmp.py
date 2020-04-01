#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import roslib; roslib.load_manifest('ur_driver')

home = [0, -pi/2, pi/2, 0, pi/2, pi]
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def move_dmp_path(path_from_ode,time_from_ode):
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        set_home = 3.0
        g.trajectory.points = [JointTrajectoryPoint(positions=home, velocities=[0]*6, time_from_start=rospy.Duration(set_home))]
        for via in range(0,len(path_from_ode)):
            joint_update = path_from_ode[via][0:6]
            print('Step %d %s' % (via,joint_update))
            g.trajectory.points.append(JointTrajectoryPoint(positions=joint_update, velocities=[0]*6, time_from_start=rospy.Duration(set_home+time_from_ode[via])))
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def home():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        g.trajectory.points = [JointTrajectoryPoint(positions=home, velocities=[0]*6, time_from_start=rospy.Duration(2))]
        g.trajectory.points.append(JointTrajectoryPoint(positions=home, velocities=[0]*6, time_from_start=rospy.Duration(4)))
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print ("Waiting for server...")
        client.wait_for_server()
        print ("Connected to server")
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print ("This program makes the robot move between the following three poses:")
        print ("Please make sure that your robot can move freely between these poses before proceeding!")
        print ("Waiting, Robot is returning to home position ....")
        # home()
        print ("Robot is now at the home position")
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):
            t = np.linspace(0.1, 2, 50)
            q = np.load('/home/s/catkin_ws/src/ur_modern_driver/q.npy')
            move_dmp_path(q,t)
        else:
            print ("Halting program")
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
