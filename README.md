This document is for the calibration of the Xiton Asus RGBD sensor

# Intrinsic calibration

Follow the tutorial on ros openni wiki
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

The checker borad we used is 9x6 with square of 27 mm width.

## RGB calibration

`rosrun camera_calibration cameracalibrator.py image:=/camera/rgb/image_raw camera:=/camera/rgb --size 9x6 --square 0.027`

## IR calibration (cover the IR projector)

`rosrun camera_calibration cameracalibrator.py image:=/camera/ir/image_raw camera:=/camera/ir --size 9x6 --square 0.027`

# Extrinsic calibration
First we need to know the ar tag position in the robot coordinate and sepcify that in config/ar_calib.yaml. If the setup is the same as the picture, the rotation is [1, 0, 0, 0, 1, 0, 0, 0, 1]. The translation is measured by moving the robot end effector to the tag and readt the end effector position. 

## AR tag tracking
`sudo apt-get install ros-indigo-ar-track-alvar`

## Change openni launch file
depth registration = true
tf publication = true

## Launch ar tag tracker
`roslaunch kinect_calibration yi_calibration.launch`

## Find the transformation
Run the script that look up transformation. The transformation is going to be saved to base_camera_tf.yaml.
`rosrun kinect_calinration get_ar_calib.py`
