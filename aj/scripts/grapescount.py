
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



class image_converter:

    speed = 0.3
    total_grapes = 0
    total_dist = 0
    move_dist = 2.2
    first = 0
    # points = ["0","1","2","3","4","5"]
    points = ["0","1","2","3","4","5","12","6","7","8","9","10","11"]
    entry = 0

    right = "/thorvald_001/kinect2_right_camera/hd/image_color_rect"
    left = "/thorvald_001/kinect2_left_camera/hd/image_color_rect"


    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
                                          Image, self.image_callback)
        self.pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=1)

    def unsubscribe(self):
        self.image_sub.unregister()



    def goto(self,waypoint,img_data):
        goal = GotoNodeGoal()
        goal.target = "WayPoint"+waypoint
        rospy.loginfo("going to %s", goal.target)
        client.send_goal(goal)
        status = client.wait_for_result() # wait until the action is complete
        result = client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)
        self.image_process(img_data)
        print("img process")
        self.entry += 1
        self.unsubscribe()

        if self.entry <= 5:
            self.image_sub = rospy.Subscriber(self.right,
                                        Image, self.image_callback)
        else:
            self.image_sub = rospy.Subscriber(self.left,
                                        Image, self.image_callback)


    def image_callback(self,data):

        img_data = self.bridge.imgmsg_to_cv2(data, "bgr8")

        if len(self.points) > self.entry:

            self.goto(self.points[self.entry], img_data)
        
        else:
            print("Endpoint")
            rospy.signal_shutdown("finished count")


        
    def image_process(self,image):

        img = image
        img = cv2.resize(img, None, fx=0.8, fy=0.8, interpolation = cv2.INTER_CUBIC)
        

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_range = np.array([75,50,20])
        upper_range = np.array([160,255,255])
        mask = cv2.inRange(hsv, lower_range, upper_range)
        res = cv2.bitwise_and(hsv,hsv, mask= mask)
        
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        # Apply thresholding to create a binary image
        threshold = cv2.threshold(gray, 0, 10, cv2.THRESH_BINARY)[1]

        # Use morphological operations to remove noise and fill in small gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,4))
        threshold = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel)

        # Use connected components to label and count the grape bunches
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
                if w < 25:
                    pass
                else:
                    self.total_grapes +=1
                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), thickness=2)
                    cv2.putText(img, str(self.total_grapes), (x,y), cv2.FONT_HERSHEY_PLAIN, 0.8, (0,0,255),2)

        filename = "test"+str(self.total_grapes)+".jpg"
        cv2.imwrite(filename, img)
        # self.total_grapes += count
        print("Grape Bunches so far:", self.total_grapes)

        # cv2.imshow("masked", mask)
        # cv2.imshow("hsv",res)
        # print(count)

        # cv2.waitKey(0)

rospy.init_node('image_converter')
client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
client.wait_for_server()
ic = image_converter()
rospy.spin()
