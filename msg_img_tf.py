# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 15:17:33 2020

@author: oscar
"""

#!/usr/bin/env python

import cv2
from geometry_msgs.msg import Quaternion
import numpy as np
import rospy
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
import tf
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
global tf_listener
global tf_broadcaster


from std_msgs.msg import String
def shutdown(self):
 
        exit()

def callback(img_msg):
    
    tf_broadcaster=tf.TransformBroadcaster()
    (trans,rot)=tf_listener.lookupTransform('hand_palm_link', 'map', rospy.Time(0))
    
    goal_hole=np.array((270,243))
   
    
    
   
    
################################################################ DO THINGS HERE
    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    gray = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)
    """orb = cv2.ORB_create()
    kp = orb.detect(cv2_img,None)
    
    kp, des = orb.compute(cv2_img, kp)
    img2 = cv2.drawKeypoints(cv2_img, kp, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

   
    
    
   # np.save('imagen.npy',cv2_img)

    """
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)
    if circles is not None:
        print (circles)
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
		cv2.circle(cv2_img, (x, y), r, (0, 255, 0), 4)
        
        if (r>100):
            px_m=0
        else:
            px_m=.06/r
            tf_broadcaster.sendTransform(
        (   (x-goal_hole[0])*.5*px_m,(y-goal_hole[1])*.5*px_m, 0.15),tf.transformations.quaternion_from_euler(3.1416, 0, 1.57),
        rospy.Time.now(),"chido","hand_palm_link"     
        )
        
    
    
        
        #tf_broadcaster.sendTransform((x-goal_hole[0])*px_m, (y-goal_hole[1])*px_m, 0),
       
         
          
          
#################  
          
    cv2.namedWindow("hand_cam")
    
    cv2.imshow("hand_cam", cv2_img)    
    k = cv2.waitKey(1)
    if k & 0xFF == ord('q'):
        # q key pressed so quit
        print("Quitting...")
        kill_node=True
        cv2.destroyAllWindows()
        exit()

    elif k & 0xFF == ord('c'):
        # c key pressed so capture frame to image file
        cap_cnt=0
        cap_name = "capture_{}.png".format(cap_cnt)
        cv2.imwrite(cap_name, cv2_img)
        print("Saving {}!".format(cap_name))
        

    
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global tf_listener
    rospy.init_node('hand_camera_listener', anonymous=True)
    
    rospy.on_shutdown(shutdown)
    tf_listener = tf.TransformListener()
    
    
    rospy.Subscriber("/hsrb/hand_camera/image_raw", Image, callback)
    global kill_node
    kill_node=False
    # spin() simply keeps python from exiting until this node is stopped
    if not kill_node:
        rospy.spin()
    


if __name__ == '__main__':
    listener()
