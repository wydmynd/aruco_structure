#!/usr/bin/env python3
import numpy as np
import time
import rospy
from cv_bridge import CvBridge
import cv2 as cv
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
import tf
from std_msgs.msg import UInt32
from std_msgs.msg import String
import message_filters


### USE CASE PARAMS
CHANGE_USE_CASE = False
CHANGE_USE_CASE_STRING = 'MODE_9_10FPS_1000'

### USE ARUCO PARAMS
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
markerLength = 0.19 # In meters
parameters = aruco.DetectorParameters_create()



class ArucoDetect():

    def __init__(self):
        self.bridge = CvBridge()

        # params for pico
        self.dist_coeffs = np.zeros(5)
        self.camera_matrix = np.eye(3)

        self.my_ids = [2,3,4]

    def Detect(self,imgG):
        if self.camera_matrix[0,0] != 0:
            #gray = np.asarray(self.bridge.imgmsg_to_cv2(imgG, 'mono16')).astype('uint8')
            gray = np.asarray(self.bridge.imgmsg_to_cv2(imgG)).astype('uint8')
            blur = cv.GaussianBlur(gray,(7,7),0) #blurring 
            ret,blur = cv.threshold(blur,180,255,cv.THRESH_OTSU) #thresholding
            corners, ids, rejectedImgPoints = aruco.detectMarkers(blur, aruco_dict, parameters=parameters)
            rvec, tvec,aaa = aruco.estimatePoseSingleMarkers(corners, markerLength, self.camera_matrix, self.dist_coeffs)
            blurShow = cv.cvtColor(blur, cv.COLOR_GRAY2BGR)
            if (type(ids) != type(None) and any(elem in self.my_ids  for elem in ids)):
                aruco.drawDetectedMarkers(blurShow, corners, ids , (255,0,0))
                for iii,id in enumerate(ids):
                    if id in ids:
                        print("id: " + str(id) + "  yaw?: " + str(rvec[iii][0][0]) + "  pitch: " + str(rvec[iii][0][1]) +"  roll?: " + str(rvec[iii][0][2]) + "  X: " + str(tvec[iii][0][0]) + "  Y: " + str(tvec[iii][0][1])  + "  Z: " + str(tvec[iii][0][2]) )
                        
            img_pub = self.bridge.cv2_to_imgmsg(blurShow)
            aruco_pub.publish(img_pub)

    def CamInfoCallBack(self,msg):
        if self.dist_coeffs[0] == 0 and self.camera_matrix[0,0] == 0.0:
            self.dist_coeffs = msg.D
            self.camera_matrix = np.reshape(msg.K,(3,3))

            print('UPDATED CAMERA PROPERTIES')
            if CHANGE_USE_CASE:
                UseCasePub.publish(CHANGE_USE_CASE_STRING)
                print('CHANGED USE_CASE TO ' + CHANGE_USE_CASE_STRING)

        else:
            return

if __name__ == '__main__':
    AD = ArucoDetect()
    rospy.init_node('arucoDetect')
    UseCasePub = rospy.Publisher('use_case', String, queue_size=1,latch=True)

    time.sleep(1)
    rospy.Subscriber("/left/camera_info", CameraInfo, AD.CamInfoCallBack)

    rospy.Subscriber("/left/image_raw", Image, AD.Detect)

    aruco_pub = rospy.Publisher('aruco_stream_debug', Image, queue_size=1)
    #img_sub = message_filters.Subscriber("/royale_camera_driver/gray_image", Image)
    #imd_sub = message_filters.Subscriber("/royale_camera_driver/depth_image", Image)
    #ts = message_filters.TimeSynchronizer([img_sub, imd_sub], 1)
    #ts.registerCallback(AD.Detect)

    print("======= Calculate Aruco Node Initialized =======")
    rospy.spin()
    cv.destroyAllWindows()
