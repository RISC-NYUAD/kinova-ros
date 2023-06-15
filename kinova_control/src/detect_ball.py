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

def img_cb(image_in):
    global CamMatrix
    global CamDist
    global bridge
    global pub
    
    cv_image = bridge.imgmsg_to_cv2(image_in, 'bgr8')

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()

    
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=params, cameraMatrix=CamMatrix, distCoeff=CamDist)
    if len(corners) > 0:
        if(ids[0]==10):
            rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.375, CamMatrix, CamDist)
            cv2.aruco.drawDetectedMarkers(cv_image, corners)
            cv2.aruco.drawAxis(cv_image, CamMatrix, CamDist, rvec, tvec, 0.01)
            msg = TransformStamped()
            rotation_matrix = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
            rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
            Q = tf.transformations.quaternion_from_matrix(rotation_matrix)
            msg.transform.translation.x = tvec[0][0][0]
            msg.transform.translation.y = tvec[0][0][1]
            msg.transform.translation.z = tvec[0][0][2]
            msg.transform.rotation.x = Q[0]
            msg.transform.rotation.y = Q[1]
            msg.transform.rotation.z = Q[2]
            msg.transform.rotation.w = Q[3]

            stamp = rospy.get_rostime()
            msg.header.stamp = stamp
            msg.header.frame_id = "/m100/camera_front"
            msg.child_frame_id = "/m100/aruco_target"
            pub.publish(msg)

            br = tf.TransformBroadcaster()
            br.sendTransform((msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z), Q, stamp, msg.child_frame_id, msg.header.frame_id)
            
    cv2.imshow("Image Preview", cv_image)
    cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('ball_tracker', anonymous=True)

    pub = rospy.Publisher('/j2s7s300/ball_data', Twist, queue_size=1) 
    bridge = CvBridge()
    
    cam_info = rospy.wait_for_message("/camera_flow/camera_info", CameraInfo)

    CamMatrix = np.array(cam_info.K).reshape((3,3))
    CamDist = np.array(cam_info.D).reshape((1,5))

    rospy.Subscriber("/camera_flow/image_raw", Image, img_cb)
    rospy.spin()    
    
