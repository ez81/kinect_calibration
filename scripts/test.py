#!/usr/bin/env python
import rospy
import tf
import numpy
import yaml
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion

def getPoseFromMatrix(matrix):
    trans, quat = getTfFromMatrix(numpy.linalg.inv(matrix))
    return Pose(position=Point(*trans), orientation=Quaternion(*quat))

def getTfFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
    return trans, tf.transformations.quaternion_from_euler(*angles)

def lookupTransform(tf_listener, target, source):
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(target, source, rospy.Time(0), rospy.Duration(4.0))
            break
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            continue
    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time(0))
    euler = tf.transformations.euler_from_quaternion(rot)
    source_target = tf.transformations.compose_matrix(translate = trans,
                                                     angles = euler)
    #print "looked up transform from", source, "to", target, "-", source_target
    return source_target

rospack = rospkg.RosPack()
config_folder = rospack.get_path('kinect_calibration') + "/config/"
with open(config_folder+'camera_config.yaml', 'r') as f:
    cam_params = yaml.load(f)

with open(config_folder+'base_camera_tf.yaml', 'r') as f:
    trans_params = yaml.load(f)

fx = cam_params['fx']
fy = cam_params['fy']
cx = cam_params['cx']
cy = cam_params['cy']

trans = trans_params['trans']
rot = trans_params['rot']
# the same could be obtrianed by listening from base to cam
camera_base = tf.transformations.compose_matrix(
                translate = trans,
                angles = rot)

# frame = '/arm'
# base_reference = lookupTransform(tf_listener, frame, '/base')

# base to camera = refrence to camera * base to reference
# camera to base = reference to base * camera to referene

# base to reference * camera to base =  camera to referene

# marker to base = camera to base * marker to camera

base_camera = numpy.linalg.inv(camera_base)
camera_pose = getPoseFromMatrix(base_camera)

trans, rot = getTfFromMatrix(numpy.linalg.inv(base_camera))
tf_broadcaster = tf.TransformBroadcaster()
rospy.init_node("test")
while not rospy.is_shutdown():
	tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "/camera", "/base")