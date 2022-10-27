#! /usr/bin/env python
###########################################################################
# @info: Generate zone visualization for rviz via OpenCV
#
# @author: Chieh Lin
#
# @contact:ga85duq@mytum.de
###########################################################################

from __future__ import print_function
from distutils.command.build import build

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lane_planner.msg import Car_scan


class ZoneVisualization:

    def __init__(self):
        rospy.init_node('zone_visualization', anonymous=True)

        self.image_pub = rospy.Publisher("/zone_image", Image, queue_size=10)

        self.laserscan_msg = Car_scan()
        self.bridge = CvBridge()
        self.laser_sub = rospy.Subscriber('/is_car', Car_scan, self.laser_callback)

        self.line_width = rospy.get_param("/laser_checker/lane_width")
        self.car_len = rospy.get_param("/laser_checker/car_len")
        self.laser_len = rospy.get_param("/laser_checker/laser_len")

        self.bridge = CvBridge()

        rate = rospy.Rate(10)

        try:
            while not rospy.is_shutdown():
                self.build_image()
                rate.sleep()

        except:
            pass

    def laser_callback(self, data):
        self.laserscan_msg = data

    def build_image(self):

        tan = np.tan(self.laserscan_msg.angle)
        shift = tan * 150

        img = np.zeros([512, 512, 3], dtype=np.uint8)
        img.fill(255)
        alpha = 0.4  # Transparency factor.
        overlay = img.copy()

        # setting zones to color blue
        color_array = np.array([(255, 0, 0), (255, 0, 0), (255, 0, 0),
                               (255, 0, 0), (255, 0, 0), (255, 0, 0), (255, 0, 0), (255, 0, 0)])
        # if obstacles detected in zone, change zone color to red
        color_array[np.where(self.laserscan_msg.zones)] = (0, 0, 255)

        # second row
        cv2.rectangle(img, (130, 215), (230, 285), tuple(map(int, color_array[3])), -1)
        cv2.rectangle(img, (270, 215), (370, 285), tuple(map(int, color_array[4])), -1)

        # third row
        cv2.rectangle(img, (130, 285), (210, 425), tuple(map(int, color_array[5])), -1)
        cv2.rectangle(img, (210, 285), (290, 425), tuple(map(int, color_array[6])), -1)
        cv2.rectangle(img, (290, 285), (370, 425), tuple(map(int, color_array[7])), -1)

        # first row(polygon for angle shifting)
        pts_left = np.array([[(130 + shift), 75], [(210 + shift), 75],
                            [210, 215], [130, 215]], np.int32)
        pts_left = pts_left.reshape((-1, 1, 2))
        cv2.fillPoly(img, [pts_left], tuple(map(int, color_array[0])))

        pts_mid = np.array([[(210 + shift), 75], [(290 + shift), 75],
                           [290, 215], [210, 215]], np.int32)
        pts_mid = pts_mid.reshape((-1, 1, 2))
        cv2.fillPoly(img, [pts_mid], tuple(map(int, color_array[1])))

        pts_right = np.array([[(290 + shift), 75], [(370 + shift), 75],
                             [370, 215], [290, 215]], np.int32)
        pts_right = pts_right.reshape((-1, 1, 2))
        cv2.fillPoly(img, [pts_right], tuple(map(int, color_array[2])))

        # Add transparency to the zones
        img_tran = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

        # Adding outlines to every zone
        cv2.rectangle(img_tran, (130, 215), (230, 285), (0, 0, 0), 3)
        cv2.rectangle(img_tran, (270, 215), (370, 285), (0, 0, 0), 3)
        cv2.rectangle(img_tran, (130, 285), (210, 425), (0, 0, 0), 3)
        cv2.rectangle(img_tran, (210, 285), (290, 425), (0, 0, 0), 3)
        cv2.rectangle(img_tran, (290, 285), (370, 425), (0, 0, 0), 3)
        cv2.polylines(img_tran, [pts_left], True, (0, 0, 0), 3)
        cv2.polylines(img_tran, [pts_mid], True, (0, 0, 0), 3)
        cv2.polylines(img_tran, [pts_right], True, (0, 0, 0), 3)

        # car(set as black)
        img_final = cv2.rectangle(img_tran, (230, 215), (270, 285), (0, 0, 0), -1)


        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_final, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    ic = ZoneVisualization()
    # cv2.destroyAllWindows()
