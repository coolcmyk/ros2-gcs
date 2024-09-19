#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def video_publisher():
    rospy.init_node('video_publisher_node', anonymous=True)
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    bridge = CvBridge()

    # Open webcam (0 is the default webcam)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        rospy.logerr("Cannot open webcam")
        return
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("Failed to capture image")
            break
        
        # Convert the OpenCV image to a ROS Image message
        img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Publish the image message
        image_pub.publish(img_msg)
        
        # Log info
        rospy.loginfo("Video frame published")
        
        rate.sleep()
    
    cap.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
