#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
import time
import rospy
import cv2


from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge



class grape_counter:

    """
    This is a class object to initiate the navigation of the thorvald robot to a specified waypoint.
    It grabs a fixed frame from the waypoint by subscribing to the camera, then passes it to be processed.
    The image process involves some open cv techniques to mask out and count the grape bunches in the image. 
    """

    total_grapes = 0
    first = 0 #to help in subscribing to the first image at a waypoint
    points = ["0","1","2","3","4","5","6","7","8","9","10","11"] #waypoints
    entry = 0 #to keep track of when robot gets to waypoint
    numb = 0 
    files = [] #save all the file names of images gotten at waypoints

    right = "/thorvald_001/kinect2_right_camera/hd/image_color_rect"
    left = "/thorvald_001/kinect2_left_camera/hd/image_color_rect"


    def __init__(self):

        self.bridge = CvBridge()

        while len(self.points) > self.entry:
            
            self.present_waypoint = self.points[self.entry]
            self.goto(self.present_waypoint)
            self.first = 0
            self.sub()
            print("subscribed to image")

        else:
            count = 0
            for _ in self.files:
                
                if count == 0:
                    count+=1
                    pass
                else:
                    # img = cv2.imread(_)
                    self.image_process(_)
                    # print("processed ", _)


            print("Endpoint")
            print("total grapes is:", self.total_grapes)
            self.move("0")
            rospy.signal_shutdown("finished count")

    #This subscribes to the camera in use
    def sub(self):

        if self.entry <= 5:
            self.image_sub = rospy.Subscriber(self.right,
                                        Image, self.image_callback)
        else:
            self.image_sub = rospy.Subscriber(self.left,
                                        Image, self.image_callback)

        
    #This is the callback for the camera subscriber
    def image_callback(self,data):

        if self.first == 0:
            img_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
            filename = "images/image"+str(self.numb)+".jpg"
            self.files.append(img_data)
            cv2.imwrite(filename, img_data)
            self.numb += 1
            self.first += 1


        
    def unsubscribe(self):
        self.image_sub.unregister()


    #To go to a certain specified waypoint
    #Inspiration from https://github.com/LCAS/CMP9767M/blob/master/uol_cmp9767m_tutorial/scripts/set_topo_nav_goal.py
    def goto(self,waypoint):

        goal = GotoNodeGoal()
        goal.target = "WayPoint"+waypoint
        rospy.loginfo("going to %s", goal.target)
        client.send_goal(goal)
        status = client.wait_for_result() # wait until the action is complete
        result = client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)
        self.entry += 1

    #To go to a point without processing any image
    #Inspiration from https://github.com/LCAS/CMP9767M/blob/master/uol_cmp9767m_tutorial/scripts/set_topo_nav_goal.py
    def move(self,waypoint):
        goal = GotoNodeGoal()
        goal.target = "WayPoint"+waypoint
        rospy.loginfo("going to startpoint")
        client.send_goal(goal)
        status = client.wait_for_result() # wait until the action is complete
        result = client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)


    #This function process the fixed image frame gotten from the camera at a particular waypoint
    def image_process(self,image):
        
        #Check if it is actually an image that is passed
        if isinstance(image, np.ndarray):

            img = image
            img = cv2.resize(img, None, fx=0.8, fy=0.8, interpolation = cv2.INTER_CUBIC)
            

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_range = np.array([75,50,20])
            upper_range = np.array([160,255,255])
            mask = cv2.inRange(hsv, lower_range, upper_range)
            res = cv2.bitwise_and(hsv,hsv, mask= mask)
            
            gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
            # Applying thresholding to create a binary image
            threshold = cv2.threshold(gray, 0, 10, cv2.THRESH_BINARY)[1]

            # Using morphological operations to remove noise and fill in small gaps
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,4))
            threshold = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel)

            # Using connected components to label and count the grape bunches
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(threshold, 8, cv2.CV_32S)

            # count = 0
            size_thresh = 1
            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] >= size_thresh:
                    #print(stats[i, cv2.CC_STAT_AREA])
                    x = stats[i, cv2.CC_STAT_LEFT]
                    y = stats[i, cv2.CC_STAT_TOP]
                    w = stats[i, cv2.CC_STAT_WIDTH]
                    h = stats[i, cv2.CC_STAT_HEIGHT]
                    # print(x,y,w,h)
                    if h < 30:
                        pass
                    else:
                        self.total_grapes +=1
                        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), thickness=2)
                        cv2.putText(img, str(self.total_grapes), (x,y), cv2.FONT_HERSHEY_PLAIN, 0.8, (0,0,255),2)

            filename = "images/test"+str(self.total_grapes)+".jpg"
            cv2.imwrite(filename, img)

        else:
            pass

rospy.init_node('grape_counter')
client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
client.wait_for_server()
ic = grape_counter()
rospy.spin()
