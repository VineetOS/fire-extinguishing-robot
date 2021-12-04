#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import CompressedImage
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import time
import cv2
import numpy as np
import fire_detector
import victim_detector
import detection
import pedestrain
import time


def image_callback(CompressedImage):
    # Convert the ROS Image to a grayscale OpenCV image
    time.sleep(2)
    print("function started")
    # cv_bridge = CvBridge()
    # cv_img = cv_bridge.imgmsg_to_cv2(CompressedImage)
    #### direct conversion to CV2 ####
    np_arr = np.fromstring(CompressedImage.data, np.uint8)
    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
    
    is_fire = fire_detector.detect(cv_img)
    print("Fire detector output: ", is_fire)
    #people_trapped = victim_detector.victims(cv_img)
    # people_trapped = detection.person_detected(cv_img)
    people_trapped = pedestrain.detected(cv_img)
    print("People trapped: ",people_trapped)
    # cv2.imshow('cv_img', cv_img)

    #### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', cv_img)[1]).tostring()
    # Publish new image
    image_pub.publish(msg)

    return is_fire, people_trapped


def callback(dt):
    thr1 = 1 
    thr2 = 1
    obs_right = False
    obs_left = False
    right=0
    left=0
    for i in range(15):
        if dt.ranges[i]<thr1:
            right = right + 1
    for i in range(15):
        if dt.ranges[359-i]<thr2:
            left = left + 1
            break
    if right>5:
        obs_right=True
    if left>5:
        obs_left=True    
    if obs_left==False and obs_right==False: 
        print("NO obstacle")
        move.linear.x = 0.3 #move forward
        move.angular.z = 0.0 # dont rotate
    elif obs_left==True and obs_right==False:
        print("LEFT obstacle")
        move.linear.x = 0.0 # stop
        move.angular.z = -0.3 # rotate clockwise
    elif obs_left==False and obs_right==True:
        print("RIGHT Obstacle")
        move.linear.x = 0.0 # stop
        move.angular.z = 0.3 # rotate counter-clockwise
    elif obs_left==True and obs_right==True:
        print("BOTH Obstacle")
        move.linear.x = 0.0 # stop
        move.angular.z = 0.1 # rotate counter-clockwise
    pub.publish(move) # publish the move object

move = Twist() # Creates a Twist message type object
rospy.init_node('main_node') # Initializes a node
print("Start cmd/vel publisher node")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
print("Starting Laser Scan subscriber") 
sub = rospy.Subscriber("/scan", LaserScan, callback) 
print("starting camera subscriber")
rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, image_callback, queue_size=1)
print("Sucessfully started all required publishers and subscribers")
# topic where we publish
image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage)

rospy.spin() 

##################################################################################################
##################################################################################################
## Raspberry pi camera input and processing
# camera = PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(640, 480))
# # allow the camera to warmup
# time.sleep(0.1)
# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
# 	# grab the raw NumPy array representing the image, then initialize the timestamp
# 	# and occupied/unoccupied text
# 	image = frame.array
#     person_present = detection.person_detected(image)
#     fire_present = fire_detector.detect(image)
#     # clear the stream in preparation for the next frame
# 	rawCapture.truncate(0)
#############################################################################
#############################################################################


