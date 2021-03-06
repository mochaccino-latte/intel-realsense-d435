#!/usr/bin/env python2
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
# from universal_robot_kinematics import invKine
# from forward_kinematics import fwd_kin
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
print("Environment Ready")

class IntelRealsense:
    def __init__(self):
        # initiate the pipeline
        self.pp_k = 0
        self.name = 1
        self.colorizer = rs.colorizer()
        self.pipe = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 90)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        profile = self.pipe.start(config)
        s = profile.get_device().query_sensors()[1]
        s.set_option(rs.option.enable_auto_exposure, False)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    def pingpong_detection(self, shift_x=0, shift_y=0, scope_side=45, scope_depth=50, display=True):
        pp_area = 0.001256 # meters
        capture = time.time()
        frameset = self.pipe.wait_for_frames()
        depth_frame = frameset.get_depth_frame()
        depth = np.asanyarray(depth_frame.get_data())
        depth_shape = depth.shape
        depth_crop = depth[0+shift_y:depth_shape[0], 0+shift_x:depth_shape[1]]
        min = depth_crop[depth_crop > 10].min()     # Depth Values
        print(min)
        if min > 950 and min < 1000:
            min_pt = np.where(depth_crop == min)
            depth_scope = depth_crop[int(min_pt[0][0]-scope_side/2):int(min_pt[0][0]+scope_side/2), int(min_pt[1][0]-scope_side/2): int(min_pt[1][0]+scope_side/2)]
            cv2.imshow('Scope side', depth_scope)
            numpix = 0; sumx = 0; sumy = 0
            for row in range(0,depth_scope.shape[0]):
                for col in range(0,depth_scope.shape[1]):
                    if depth_scope[row,col] < min+scope_depth and depth_scope[row,col] > min-scope_depth:numpix+=1; sumx += col; sumy += row
            if numpix != 0:ppscope_x = sumx/numpix; ppscope_y = sumy/numpix
            else:ppscope_x = 0; ppscope_y = 0
            pp_x = ppscope_x+shift_x+min_pt[1][0]-scope_side/2
            pp_y = ppscope_y+shift_y+min_pt[0][0]-scope_side/2
            pp_depth = depth[int(pp_y),int(pp_x)] * self.depth_scale
            if numpix != 0:
                self.pp_k = pp_area/numpix
        else:
            pp_x=0; pp_y=0; pp_depth=0
        if display == True:
            depth_color = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
            cv2.circle(depth_color, (int(pp_x),int(pp_y)), (int)(1),(0,255,0),2)
            cv2.namedWindow('Object Detectiom using Depth Image', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Object Detectiom using Depth Image', depth_color)
            filename = 'pingpong'+str(self.name)+'.png'
            cv2.imwrite('/home/idealab/catkin_ws/src/thesis/src/detect/'+filename,depth_color)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
        return pp_x, pp_y, pp_depth, self.pp_k, capture

    def pingpong_velocity(self, STATE, pp_x, pp_y, pp_depth, capture, display=True):
        v_x = None; v_y = None; v_depth = None
        if pp_x == 0 and pp_y == 0:
            STATE = 'NONE'
        else:
            if STATE == 'NONE':STATE='INITIAL'
            if STATE == 'INITIAL':
                self.lastpp_x = pp_x; self.lastpp_y = pp_y; self.lastcapture = capture
                self.lastpp_depth = pp_depth; STATE = 'VELOCITY'
            elif STATE == 'VELOCITY':
                delt = capture-self.lastcapture
                v_x = (pp_x - self.lastpp_x)/(delt); v_y = (pp_y - self.lastpp_y)/(delt)
                v_depth = (pp_depth - self.lastpp_depth)/(delt)
                self.lastv_x = v_x; self.lastv_y = v_y
                self.lastv_depth = v_depth
                self.lastpp_x = pp_x; self.lastpp_y = pp_y
                self.lastpp_depth = pp_depth
                STATE = 'KALMAN'
            elif STATE == 'KALMAN':
                delt = capture-self.lastcapture
                v_x = (pp_x - self.lastpp_x)/(delt); v_y = (pp_y - self.lastpp_y)/(delt)
                v_depth = (pp_depth - self.lastpp_depth)/(delt)
                if display != True:
                    self.lastv_x = v_x; self.lastv_y = v_y
                    self.lastv_depth = v_depth
                    self.lastpp_x = pp_x; self.lastpp_y = pp_y
                    self.lastpp_depth = pp_depth
        if display == True and  STATE == 'KALMAN' :
            frameset = self.pipe.wait_for_frames()
            depth_frame = frameset.get_depth_frame()
            depth_color = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
            predpp_x = self.lastpp_x + self.lastv_x*delt
            # predpp_y = self.lastpp_y + self.lastv_y*delt + 0.5*9.8*(delt**2)
            self.lastv_x = v_x; self.lastv_y = v_y
            self.lastpp_x = pp_x; self.lastpp_y = pp_y
            cv2.line(depth_color, (int(predpp_x), 0), (int(predpp_x), depth_color.shape[0]), (0,255,0), 1)
            # cv2.line(depth_color, (0, int(predpp_y)), (depth_color.shape[1],int(predpp_y)), (0,255,0), 1)
            cv2.namedWindow('State Prediction', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('State Prediction', depth_color)
            filename = 'pingpong'+str(self.name)+'.png'
            cv2.imwrite('/home/idealab/catkin_ws/src/thesis/src/predict/'+filename,depth_color)
            self.name+=1
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
        return v_x,v_y,v_depth,STATE

    def transformation_matrix(self, dis_x, dis_y, dis_z,):
        A = [[-1, 0, 0, dis_x],
            [0, 0, 0, dis_y],
            [0, 1, 1, dis_z],
            [0, 0, 0, 1]]
        return A

    def rot2qua(self,MAT):
        # quaternion conversion
        w = math.sqrt(1 + MAT[0,0]**2 + MAT[1,1]**2 + MAT[2,2]**2)/2.0
        x = (MAT[2,1] - MAT[1,2])/(4.0*w)
        y = (MAT[0,2] - MAT[2,0])/(4.0*w)
        z = (MAT[1,0] - MAT[0,1])/(4.0*w)
        QUA = np.array([[x, 0, 0, 0],
                [0, y, 0, 0],
                [0, 0, z, 0],
                [0, 0, 0, w]])
        return QUA

# __main__
obj_x = 10; obj_y = 10; obj_z=10
MAT = [[obj_x],
        [obj_y],
        [obj_z],
        [1] ]
IntelRealsense = IntelRealsense()

if __name__ == '__main__':
    print("Initiate Object Detection")
    STATE = 'INITIAL'
    _,_,_,_, last_capture = IntelRealsense.pingpong_detection(display = False)
    TRAN =  IntelRealsense.transformation_matrix(0, 0, 0)
while True:
    # processing time 
    pp_x, pp_y, pp_depth, pp_k, capture = IntelRealsense.pingpong_detection(display = True)
    processing = capture - last_capture
    # print('proccessing time %f' %(processing))
    v_x, v_y, v_depth, STATE = IntelRealsense.pingpong_velocity(STATE, pp_x, pp_y, pp_depth, capture, display = False)
    print('pp_x, pp_y, Vv_x, v_y : %.3f %.3f %s %s' %(pp_x,pp_y,v_x,v_y))
    output = [v_x, v_y, v_depth]
    output2 = []
    for i in output:
        if i!= None:
            output2.append(round(i,3))
        else:output2.append(i)
    last_capture = capture
    # position = fwd_kin([math.pi/4, -math.pi/4, math.pi/4, -math.pi/4, math.pi/4, math.pi/4], o_unit='n')
    # print('fwd_kin',position)
    # INV = invKine(np.matrix(position))
    # DEG = (180/math.pi)*INV
    # print('invKine',INV)
    # print('\n')
