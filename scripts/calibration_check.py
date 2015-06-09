#!/usr/bin/env python
import sys
import rospy
import baxter_external_devices
import argparse
from baxter_interface import CameraController, Gripper, Limb
from sensor_msgs.msg import Image, Range
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import cv_bridge
import cv2
import numpy as np
import math
from skimage import segmentation, measure, morphology
import matplotlib.pyplot as plt
import copy
import time
import tf

import pygst
import gst

from arcbaxter.srv import *
from julia_main.srv import *
from baxter_pykdl import baxter_kinematics



class vision_service():
    
    def __init__(self):

        rospy.init_node('kinect_vision_server')
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.busy = False
        self.dx = 0
        self.dy = 0
        self.avg_dx = -1
        self.avg_dy = -1
        self.framenumber = 0
        self.history_x = np.arange(0,10)*-1
        self.history_y = np.arange(0,10)*-1
        self.newPosition = True
        self.centerx = 230
        self.centery = 350
        self.coefx = 0.1/(526-369)
        self.coefy = 0.1/(237-90)
        self.found = 0
        self.finish = 0
        self.request = 0
        self.close = 0
        self.box = []
        self.u = 0
        self.v = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.depth_image = 0
        
        camera_topic = '/camera/rgb/image_rect_color'
        self.right_camera = rospy.Subscriber(camera_topic, Image, self._on_camera)
        self.depth_image = rospy.Subscriber('/camera/depth_registered/hw_registered/image_rect_raw', Image, self.depth_camera_callback)
        cv2.namedWindow('rgb_image')
        cv2.setMouseCallback('rgb_image', self.mouse_callback)
        print "\nReady to use right hand vision server\n" 


    def _on_camera(self, data):

        self.framenumber += 1
        index = self.framenumber % 10
        
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")
        np_image = np.asarray(cv_image)
        cv2.imshow('rgb_image',np_image)
        cv2.waitKey(1)

    def mouse_callback(self, event,v,u,flags,param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            d =  float(self.depth_image[u, v]) # in mm

            self.u = u
            self.v = v


            if d < 1500:
                # test
                # cx = 311.17
                # cy = 269.89
                # fx = 633.21
                # fy = 603.93

                # good
                cx = 318
                cy = 269.89
                fx = 527.67
                fy = 603.93
                x = (self.v-cx)*d/fx
                y = (self.u-cy)*d/fy
                print x, y, d
                kinect_point = np.asarray([x/1000,y/1000,d/1000,1])
                # good
                transform = np.asarray([[ -7.81514275e-03 , -5.43990398e-01 , 8.39055046e-01 , 1.58875856e-01],[ -9.99896206e-01 ,1.44074926e-02 , 2.76573540e-05 ,9.82764204e-03],[ -1.21037247e-02 ,-8.38967741e-01 ,-5.44046532e-01 , 5.03940029e-01], [  0.00000000e+00 ,  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00]]) #good
                # test
                #transform = np.asarray([[ -0.05211748 , -0.52578232 , 0.84902103 , 0.1523356],[ -0.99854309 ,0.01553473 , -0.05167559 ,0.02471122],[ 0.0139808 ,-0.85047729 ,-0.52582594 , 0.49263835], [  0.00000000e+00 ,  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00]])

                robot_point = np.dot(transform,np.transpose(kinect_point))
                self.x = robot_point[0]
                self.y = robot_point[1]
                self.z = robot_point[2]
                print self.x, self.y, self.z

                hdr = Header(stamp=rospy.Time.now(), frame_id='base')

                if self.y < 0:
                    print "move right arm\n"

                    right_pose = PoseStamped()
                    right_pose.header = hdr
                    right_pose.pose.position.x = self.x
                    right_pose.pose.position.y = self.y
                    right_pose.pose.position.z = self.z + 0.2
                    right_pose.pose.orientation.x = 0
                    right_pose.pose.orientation.y = 1
                    right_pose.pose.orientation.z = 0
                    right_pose.pose.orientation.w = 0
                    move_to_pose(right_pose = right_pose, timeout = 4)

                    

                else:
                    print "move left arm\n"

                    left_pose = PoseStamped()
                    left_pose.header = hdr
                    left_pose.pose.position.x = self.x
                    left_pose.pose.position.y = self.y
                    left_pose.pose.position.z = self.z + 0.2
                    left_pose.pose.orientation.x = 0
                    left_pose.pose.orientation.y = 1
                    left_pose.pose.orientation.z = 0
                    left_pose.pose.orientation.w = 0
                    move_to_pose(left_pose = left_pose, timeout = 4)



    def depth_camera_callback(self, data):
        self.framenumber += 1
        index = self.framenumber % 10
        # cv2.namedWindow('depth_image')
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "16UC1")
        self.depth_image = np.asarray(cv_image)
        d =  float(self.depth_image[self.u, self.v]) # in mm



def ik(pose, side):
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    if side == "left": ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 0

    if (resp.isValid[0]):
        #print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        if side == "right": arm = Limb("right")
        else: arm = Limb("left")
        arm.set_joint_positions(limb_joints)
        return 1
        #rospy.sleep(0.05)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 0

def move_to_pose(left_pose = None, right_pose = None, timeout = 2.0):
    start = rospy.get_time()
    while not rospy.is_shutdown() and (rospy.get_time() - start) < timeout:
        if left_pose != None : ik(left_pose,"left")
        if right_pose != None :ik(right_pose, "right")

def vision_request(name):

    print "request object: %s position from the server" % name

    try:
        rospy.wait_for_service("get_obj_loc")
        vision_server_req = rospy.ServiceProxy("get_obj_loc", GetObjLoc)
        return vision_server_req(name)
    
    except (rospy.ServiceException,rospy.ROSInterruptException), e:
        print "Service call failed: %s" % e

def request_object(name):
    resp = vision_request(name)
    print resp

    



if __name__=='__main__':
    
    vision = vision_service()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    left_pose = PoseStamped()
    left_pose.header = hdr
    left_pose.pose.position.x = 0.3
    left_pose.pose.position.y = 0.7
    left_pose.pose.position.z = 0.2
    left_pose.pose.orientation.x = 0
    left_pose.pose.orientation.y = 1
    left_pose.pose.orientation.z = 0
    left_pose.pose.orientation.w = 0

    right_pose = PoseStamped()
    right_pose.header = hdr
    right_pose.pose.position.x = 0.3
    right_pose.pose.position.y = -0.7
    right_pose.pose.position.z = 0.2
    right_pose.pose.orientation.x = 0
    right_pose.pose.orientation.y = 1
    right_pose.pose.orientation.z = 0
    right_pose.pose.orientation.w = 0



    done = False
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                # rospy.signal_shutdown("Example finished.")
            elif c == 'l':
                print "move left arm"
                move_to_pose(left_pose = left_pose, timeout = 4)

            elif c == 'r':
                print "move right arm"
                move_to_pose(right_pose = right_pose, timeout = 4)

            elif c == 's':
                request_object()

            else:
                print c

    print "Shut down"
    move_to_pose(left_pose = left_pose, right_pose = right_pose,timeout = 4)
