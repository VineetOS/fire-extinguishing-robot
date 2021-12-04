#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge
import cv2
import fire_detector
import victim_detector
import detection
import pedestrain
import time
import numpy as np

def image_callback(CompressedImage):
    # Convert the ROS Image to a grayscale OpenCV image
    print("Processing camera input to Detect people trapped and fire detection")
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

    # #### Create CompressedIamge ####
    # msg = CompressedImage()
    # msg.header.stamp = rospy.Time.now()
    # msg.format = "jpeg"
    # msg.data = np.array(cv2.imencode('.jpg', cv_img)[1]).tostring()
    # # Publish new image
    # image_pub.publish(msg)

    return is_fire, people_trapped




    
rospy.init_node("camera")
# rospy.Subscriber("/usb_cam/image_raw", Image, image_callback, queue_size=1)
print("starting Raspberry PI camera subscribe node")
rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, image_callback, queue_size=1)
print("Sucessfully started camera node")
# topic where we publish
# image_pub = rospy.Publisher("/output/image_raw/compressed",
#             CompressedImage)
rospy.spin()
