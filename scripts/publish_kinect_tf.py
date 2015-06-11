#!/usr/bin/env python
import rospy
import tf
import numpy
import yaml
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion

rospack = rospkg.RosPack()
config_folder = rospack.get_path('kinect_calibration') + "/config/"

with open(config_folder+'base_camera_tf.yaml', 'r') as f:
    trans_params = yaml.load(f)

rospy.init_node("kinect_pose_tf_pubilsher")
trans = trans_params['trans']
rot = trans_params['rot']
parent_frame = trans_params['parent']
child_frame = trans_params['child']
tf_broadcaster = tf.TransformBroadcaster()

print "Publish the kinect pose from " + parent_frame + " to " + child_frame
print "trans: " + str(trans)
print "rot: " + str(rot)
print "Press Ctrl+C to shut down"
while not rospy.is_shutdown():
    tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), child_frame, parent_frame)
    rospy.sleep(0.1)