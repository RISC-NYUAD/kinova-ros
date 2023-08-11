#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import tf

global CamMatrix
global CamDist
global bridge
global pub
global DepthCamMatrix
global depth_array

def img_cb(image_in):
    global CamMatrix
    global CamDist
    global bridge
    global pub
    
    cv_image = bridge.imgmsg_to_cv2(image_in, 'bgr8')
    boundaries = [
        ([0, 0, 80], [80, 80, 255])
        ]

    for(lower, upper) in boundaries:
        # creates numpy array from boundaries
        #Single-loop 'for'
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        # finds colors in boundaries a applies a mask
        mask = cv2.inRange(cv_image, lower, upper)
        output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

        # saves the image
        #cv2.imwrite('Red Image', output)
    """
    dx = 0
    dy = 0
    L = 0
    rows,cols = mask.shape
    for i in range(rows):
        for j in range(cols):
            k = mask[i,j]
            if(k > 0):
                dx += i
                dy += j
                L += 1
    p_x = dx/L
    p_y = dy/L
    """        

    L = list(zip(*np.where(mask>0)))
    if(len(L)>0):
        N = np.array(L)
        PX = np.sum(N[:,0]) / len(L)
        PY = np.sum(N[:,1]) / len(L)
        
        
        tot_pixel = output.size
        red_pixel = np.count_nonzero(output)
        percentage = round(100 * red_pixel / tot_pixel, 2)
        #print(percentage, PX, PY)
        results = Twist()
        results.linear.x = PX
        results.linear.y = PY
        results.linear.z = percentage
        pub.publish(results)
        #cv2.imshow("Image Preview", output)
        #cv2.waitKey(3)
    else:
        results = Twist()
        results.linear.x = -1
        results.linear.y = -1
        results.linear.z = -1
        pub.publish(results)        

def depth_img_cb(depth_image):
    global depth_array
    global bridge
    
    image = bridge.imgmsg_to_cv2(depth_image,"32FC1")
    depth_array = np.array(image, dtype=np.float32)
    return

if __name__ == '__main__':
    rospy.init_node('ball_tracker', anonymous=True)

    pub = rospy.Publisher('/j2s7s300/ball_data', Twist, queue_size=1) 
    bridge = CvBridge()
    
    cam_info = rospy.wait_for_message("/camera_d435/color/camera_info", CameraInfo)
    depth_cam_info = rospy.wait_for_message("/camera_d435/depth/camera_info", CameraInfo)    

    CamMatrix = np.array(cam_info.K).reshape((3,3))
    CamDist = np.array(cam_info.D).reshape((1,5))
    DepthCamMatrix = np.array(depth_cam_info.K).reshape((3,3))

    rospy.Subscriber("/camera_d435/color/image_raw", Image, img_cb, queue_size=1)
    rospy.Subscriber("/camera_d435/depth/image_raw", Image, depth_img_cb, queue_size=1)    
    rospy.spin()    
    
