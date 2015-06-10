This document is for the calibration of the Xiton Asus RGBD sensor

Intrinsic calibration

Follow the tutorial on ros openni wiki
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

The checker borad we used is 9x7 24.5 mm
The checker borad we used is 9x6 27 mm

RGB calibration
rosrun camera_calibration cameracalibrator.py image:=/camera/rgb/image_raw camera:=/camera/rgb --size 9x7 --square 0.0245

rosrun camera_calibration cameracalibrator.py image:=/camera/rgb/image_raw camera:=/camera/rgb --size 9x6 --square 0.027

IR calibration (cover the IR projector)
rosrun camera_calibration cameracalibrator.py image:=/camera/ir/image_raw camera:=/camera/ir --size 9x7 --square 0.0245

rosrun camera_calibration cameracalibrator.py image:=/camera/ir/image_raw camera:=/camera/ir --size 9x6 --square 0.027

Camera check
rosrun camera_calibration cameracheck.py --size 9x7 monocular:=/camera/rgb image:=image_rect_color

Extrinsic calibration
#AR tag tracking
sudo apt-get install ros-indigo-ar-track-alvar
#change openni launch 
depth registration = true
tf publication = true