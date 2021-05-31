#!/usr/bin/env python
import rospy, message_filters, cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg msg import Twist
from cv_bridge import CvBridge, CvBridgeError



class line_follower:
    def __init__(self,name):
        self.name  = name
        colour_image_topic = "/camera/rgb/image_raw"
        depth_image_topic = "/camera/depth/image_raw"
        colour_sub = message_filters.Subscriber(colour_image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_image_topic, Image)
        self.camera_sync = message_filters.TimeSynchronizer([colour_sub, depth_sub], 10)
        self.camera_sync.registerCallback(self.callback_camera)

        rospy.Subscriber(depth_image_topic, Image, self.callback_depth)
        message_filters.Subscriber(depth_image_topic, Image)

        self.camera_sync = message_filters.ApproximateTimeSynchronizer([colour_sub, depth_sub], queue_size=10,
                                                                       slop=0.1)

        # Inital variables for images
        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.num_colour_images = 0
        self.num_depth_images = 0


        self.image = None
        while not rospy.is_shutdown():
            ## not sure for this call method since we define image has None from the lab notes
            self.loop(color_image= self.image)
            if self.image != None:
                cv2.imshow("window", self.image)

    def loop(self, color_image=None):
        ## finding the line 3.3 Lab 10 code
        if self.colour_frame != None and self.depth_frame != None:
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            lower_yellow = (25, 200, 100)
            upper_yellow = (35, 255,255)
            mask = cv2.inRange(hsv,lower_yellow,upper_yellow)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations = 2)
            

    def callback_camera(self, colour_msg, depth_msg):
        rospy.logininfo("[%s] callback_camera()", self.name)
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg, desired_encoding="bgr8")
        depth_msg = self.bridge.imgmsg_to_cv2(depth_msg, desire_encoding="passthrough")

if __name__ == '__main__':
    rospy.init_node('line_follower', anonymouos=True)
    rospy.logininfo("[line_follower] Starting Line Follower Module")
    lf = line_follower("line_follower")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logininfo("[line_follower] Shutting Down Line Follower Module")
