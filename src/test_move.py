#!/usr/bin/env python
#
# Copyright 2015, 2016 Thomas Timm Andersen (original version)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import numpy as np

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Q1 = [2.2,0,-1.57,0,0,0]
# Q2 = [1.5,0,-1.57,0,0,0]
# Q3 = [1.5,-0.2,-1.57,0,0,0]
straight = [0, -pi/2, 0, -pi/2, 0, 0]
# Q2 = [0, -pi/4, 0, -pi/4, 0, 0]
# Q3 = [0, 0, 0, 0, 0, 0]
home = [0, -pi/2, pi/2, 0, pi/2, pi]
path_from_ode = [ [5,10,15,20,25] , [[0, -pi/2, 0, -pi/2, 0, 0],[0, -pi/4, 0, -pi/4, 0, 0],[0, 0, 0, 0, 0, 0],[0, -pi/4, 0, -pi/4, 0, 0],[0, -pi/2, 0, -pi/2, 0, 0]] ]

client = None

# def move_repeated():
#     g = FollowJointTrajectoryGoal()
#     g.trajectory = JointTrajectory()
#     g.trajectory.joint_names = JOINT_NAMES
#     try:
#         joint_states = rospy.wait_for_message("joint_states", JointState)
#         joints_pos = joint_states.position
#         d = 10.0
#         g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
#         for i in range(3):
#             g.trajectory.points.append(JointTrajectoryPoint(positions=Q1, velocities=[0.005]*6, time_from_start=rospy.Duration(d)))
#             d+=10
#             g.trajectory.points.append(JointTrajectoryPoint(positions=Q2, velocities=[0.005]*6, time_from_start=rospy.Duration(d)))
#             d+=10
#             g.trajectory.points.append(JointTrajectoryPoint(positions=Q3, velocities=[0.005]*6, time_from_start=rospy.Duration(d)))
#             d+=10
#         client.send_goal(g)
#         client.wait_for_result()
#     except KeyboardInterrupt:
#         client.cancel_goal()
#         raise
#     except:
#         raise

def move_dmp_path(path_from_ode,time_from_ode):
    # all_via_point = []
    # index_info_t = 0
    # index_info_q = 1
    # for via in range(0,len(path_from_ode[index_info_t])):
    #     via_joint = []
    #     via_joint = path_from_ode[index_info_q][via]
    #     via_joint.append( path_from_ode[index_info_t][via] )
    #     all_via_point.append(via_joint)
    # print(all_via_point)
    # print([all_via_point[0][i] for i in range(0,len(all_via_point[0])-1)])
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        # set_home = 5.0
        # g.trajectory.points = [JointTrajectoryPoint(positions=straight, velocities=[0]*6, time_from_start=rospy.Duration(set_home))]
        set_home = 3.0
        g.trajectory.points = [JointTrajectoryPoint(positions=home, velocities=[0]*6, time_from_start=rospy.Duration(set_home))]
        # print(time_from_ode)
        for via in range(0,len(path_from_ode)):
            joint_update = path_from_ode[via][0:6]
            # joint_update = [round(q,5) for q in joint_update]
            print('Step %d %s' % (via,joint_update))
            g.trajectory.points.append(JointTrajectoryPoint(positions=joint_update, velocities=[0]*6, time_from_start=rospy.Duration(set_home+time_from_ode[via])))
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
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
        # print str([Q1[i]*180./pi for i in xrange(0,6)])
        # print str([Q2[i]*180./pi for i in xrange(0,6)])
        # print str([Q3[i]*180./pi for i in xrange(0,6)])
        print ("Please make sure that your robot can move freely between these poses before proceeding!")
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):
            t = np.linspace(0.1, 2, 50)
            q = np.load('/home/s/catkin_ws/src/ur_modern_driver/q.npy')
            # q = np.load('/home/s/ur5_ws/src/ur_modern_driver/q2.npy')
            #move1()
            #move_repeated()
            #move_disordered()
            #move_interrupt()
            move_dmp_path(q,t)
        else:
            print ("Halting program")
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()



# def move_interrupt():
#     g = FollowJointTrajectoryGoal()
#     g.trajectory = JointTrajectory()
#     g.trajectory.joint_names = JOINT_NAMES
#     try:
#         joint_states = rospy.wait_for_message("joint_states", JointState)
#         joints_pos = joint_states.position
#         g.trajectory.points = [
#             JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
#             JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
#             JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
#             JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]

#         client.send_goal(g)
#         time.sleep(3.0)
#         print "Interrupting"
#         joint_states = rospy.wait_for_message("joint_states", JointState)
#         joints_pos = joint_states.position
#         g.trajectory.points = [
#             JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
#             JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
#             JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
#             JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
#         client.send_goal(g)
#         client.wait_for_result()
#     except KeyboardInterrupt:
#         client.cancel_goal()
#         raise
#     except:
#         raise

# def move1():
#     global joints_pos
#     g = FollowJointTrajectoryGoal()
#     g.trajectory = JointTrajectory()
#     g.trajectory.joint_names = JOINT_NAMES
#     try:
#         joint_states = rospy.wait_for_message("joint_states", JointState)
#         joints_pos = joint_states.position
#         g.trajectory.points = [
#             JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
#             JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
#             JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
#             JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
#         client.send_goal(g)
#         client.wait_for_result()
#     except KeyboardInterrupt:
#         client.cancel_goal()
#         raise
#     except:
#         raise

# def move_disordered():
#     order = [4, 2, 3, 1, 5, 0]
#     g = FollowJointTrajectoryGoal()
#     g.trajectory = JointTrajectory()
#     g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
#     q1 = [Q1[i] for i in order]
#     q2 = [Q2[i] for i in order]
#     q3 = [Q3[i] for i in order]
#     try:
#         joint_states = rospy.wait_for_message("joint_states", JointState)
#         joints_pos = joint_states.position
#         g.trajectory.points = [
#             JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
#             JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
#             JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
#             JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
#         client.send_goal(g)
#         client.wait_for_result()
#     except KeyboardInterrupt:
#         client.cancel_goal()
#         raise
#     except:
#         raise
