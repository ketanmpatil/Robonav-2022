#!/usr/bin/env python
 
# Import the necessary libraries

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import tf2_ros
import tf_conversions
import geometry_msgs
import os
from std_msgs.msg import String

feedback = String()

colour = rospy.get_master('/colour')
colour = 'green' # Debugging

COUNT = 0

blueLower = (0, 200, 100)
blueUpper = (10, 255, 255)

greenLower = (50, 200, 100)
greenUpper =  (100, 255, 255)

yellowLower = (60, 125, 90)
yellowUpper =  (100, 135, 150)

pinkLower = (140, 250, 0)
pinkUpper =  (155, 255, 255)

redLower = (0, 250, 0)
redUpper =  (155, 255, 255)

if colour == 'blue':
    lower = blueLower
    upper = blueUpper

if colour == 'green':
    lower = greenLower
    upper = greenUpper

if colour == 'yellow':
    lower = yellowLower
    upper = yellowUpper

if colour == 'pink':
    lower = pinkLower
    upper = pinkUpper

else:
    lower = redLower
    upper = redUpper

cy = 240.5 # cy and cx are coordinates for center of camers
cx = 320.5
f = 554.254 # Focal length

img = None
dpt = None

br = CvBridge()

broadcaster = tf2_ros.TransformBroadcaster()

def img_callback(msg): # RGB Image callback function
    global img
    img = br.imgmsg_to_cv2(msg, 'bgr8')
    img = np.array(img) 

def depth_callback(dpt_msg): # Depth Image callback function
    global dpt
    dpt = br.imgmsg_to_cv2(dpt_msg, '32FC1')
    dpt = np.array(dpt, dtype= np.dtype('float32'))

def shape_identifier(args, img):

    contour = args

    shape = rospy.Publisher('shape', String, queue_size = 10)
    SHAPE = String()
    
    approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)
    x = approx.ravel()[0]
    y = approx.ravel()[1] - 5
    if len(approx) == 3:
        SHAPE.data = 'Triangle'
        shape.publish(SHAPE.data)
    elif len(approx) == 4 :
        x, y , w, h = cv2.boundingRect(approx)
        aspectRatio = float(w)/h
        # print(aspectRatio)
        if aspectRatio >= 0.95 and aspectRatio < 1.05: 
            SHAPE.data = 'Cube'   
            shape.publish(SHAPE.data)
        else:
            print(cv2.contourArea(contour)/(w*h))
            if cv2.contourArea(contour)/(w*h) > 0.8:
                SHAPE.data = 'Rectangle'
                shape.publish(SHAPE.data)
            else:
                SHAPE.data = 'Rhombus'
                shape.publish(SHAPE.data)

    elif len(approx) == 5 :
        SHAPE.data = 'Pentagon'
        shape.publish(SHAPE.data)
    elif len(approx) == 10 :  
        SHAPE.data = 'Star'  
        shape.publish(SHAPE.data)
    else:
        SHAPE.data = 'Sphere'
        shape.publish(SHAPE.data)


def main():
    rospy.init_node('tomato_tf', anonymous=True) # "tomato_tf" node initilization

    pub = rospy.Publisher('chatter', String, queue_size=10)

    rospy.Subscriber("/head_camera/rgb/image_raw", Image, img_callback) # Subscribe to camera topics
    rospy.Subscriber("/head_camera/depth_registered/image_raw", Image, depth_callback)
    
    rate = rospy.Rate(10) # 10Hz

    rospy.loginfo('Image Feed Recieved....')


    while not rospy.is_shutdown():
        if  (type(img) == np.ndarray) and (type(dpt) == np.ndarray):
            try:    
                img2 = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

                mask = cv2.inRange(img2, lower, upper)
                mask = cv2.dilate(mask, None, iterations=1)
	
                _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                                 
                count = 0
                contours = list(contours)
                if len(contours) == 0:
                    feedback.data = 'next'
                    pub.publish(feedback)

                else:
                    feedback.data = 'stop'
                    pub.publish(feedback)
                for contour in contours:
                    (x, y), _ = cv2.minEnclosingCircle(contour)
                    x = int(x)
                    y = int(y)
                    
                    depth = dpt[y, x]

                    # World Coordinates
                    X = depth * (x-cx)/f
                    Y = depth * (y-cy)/f
                    Z = depth
                    shape_identifier(contour,img) 

                    try:
                        # Frame Id: camera_depth_frame2
                        t = geometry_msgs.msg.TransformStamped()
                        t.header.stamp = rospy.Time.now()
                        t.header.frame_id = 'head_camera_rgb_optical_frame'
                        t.child_frame_id = "shape"

                        # putting world coordinates coordinates as viewed for depth frame
                        t.transform.translation.x = X
                        t.transform.translation.y = Y
                        t.transform.translation.z = Z

                        # not extracting any orientation thus orientation is (0, 0, 0)
                        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
                        t.transform.rotation.x = q[0]
                        t.transform.rotation.y = q[1]
                        t.transform.rotation.z = q[2]
                        t.transform.rotation.w = q[3]

                        if (X != 0) and (Y != 0) and (Z != 0):
                            broadcaster.sendTransform(t)
                    except:
                        pass

		
                cv2.imshow("RGB IMAGE", img)
                cv2.imshow("mask", mask)
                cv2.waitKey(1)

            except KeyboardInterrupt:
                print('Shutting Down')
                cv2.destroyAllWindows()
                break
            except:
                raise
        
        rate.sleep()

if __name__ == '__main__':
    main()



