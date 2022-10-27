#! /usr/bin/env python
# coding: utf-8

###########################################################################
# @info: script to defineing the laser_checker node used to check for obsticales
#        arround the TAS car
#
# @author: Sebastian Siegner
#
# @contact: sebastian.siegner@tum.de
###########################################################################

import rospy
from sensor_msgs.msg import PointCloud2
from move_base_msgs.msg import MoveBaseActionFeedback
from control_msgs.msg import JointControllerState
import numpy as np
from lane_planner.msg import Car_scan
import ros_numpy
import rospkg


# LaserChecker Class to check the laser scan and deterimen if there are obsticales arround the TAS car
class LaserChecker:

    # Default constructor for the LaserChecker simulator.
    # Contains all needed constants and settings.
    def __init__(self):
        rospy.init_node('laser_checker', anonymous=True)

        self.lane_width = rospy.get_param("/laser_checker/lane_width")
        self.lane_offset = rospy.get_param("/laser_checker/lane_offset")

        self.car_len = rospy.get_param("/laser_checker/car_len")
        self.laser_len = rospy.get_param("/laser_checker/laser_len")

        self.debug = rospy.get_param(
            "/laser_checker/debug")  # turns on debug mode

        # Turnes on curved zones in corners
        self.Curve_zones = rospy.get_param("/laser_checker/Curve_zones")

        self.Activation_threshold = rospy.get_param(
            "/laser_checker/Activation_threshold")*2
        # Required since size counts x and y extra but threshold is for the number of samples

        # class atribute definition
        self.is_curve = False
        self.is_curve_right = False
        self.curve = 0
        self.curves = np.zeros(5)
        self.alphas = np.ones(5)/5
        self.curve_radius = 25
        self.x_curve = 62
        self.xy_points = np.zeros([0, 2])

        self.rospack = rospkg.RosPack()

        # Initializes all subscribers and publishers for the Laser Checker

        # init the publisher for the zones as a bool array for further processing
        self._is_car_pub = rospy.Publisher('/is_car', Car_scan, queue_size=10)

        # init the publisher for the zones so they can be plotted in RViz
        self._zone0_pub = rospy.Publisher(
            '/cloud_zone0', PointCloud2, queue_size=10)

        self._zone1_pub = rospy.Publisher(
            '/cloud_zone1', PointCloud2, queue_size=10)

        self._zone2_pub = rospy.Publisher(
            '/cloud_zone2', PointCloud2, queue_size=10)

        # Laser scan as pointcloud
        self._scan_sub = rospy.Subscriber(
            '/cloud_merged', PointCloud2, self._preprocess_points, queue_size=10)

        # init subscriber used to get the global position (only if the curved zones are on)
        if self.Curve_zones:
            self._nav_sub = rospy.Subscriber(
                '/move_base/feedback', MoveBaseActionFeedback, self._check_for_curve, queue_size=10)

        # init of subscriber to get the rotation of the wheels
        self._l_steer_sub = rospy.Subscriber(
            '/tas_car/l_steer_controller/state', JointControllerState, self._set_curve_l_steer, queue_size=10)

        rate = rospy.Rate(10)
        rospy.loginfo('Zone filter running!')
        # rospy.spin()

        try:
            while not rospy.is_shutdown():
                rate.sleep()
                self._check_scan()
        except Exception as e:
            rospy.logerr(e)

    # Function sorts the detected points into the zones and publishes the zones
    # The zones in the front can change shape in corners and arjust them self
    # based on the angle of the wheel

    def _check_scan(self):
        # FRONT
        front = self.xy_points[self.xy_points[:, 0] > 0, :]
        front = front[front[:, 0] < self.laser_len, :]

        # Selekt which mask is used (ether curve or square)
        if self.Curve_zones and self.is_curve:
            mask_front = (np.sqrt(np.square(self.curve_radius) -
                          np.square(front[:, 0]))-self.curve_radius)  # curve
            if not self.is_curve_right:
                mask_front = -mask_front
        else:
            tan_fact = np.tan(self.curve)
            mask_front = front[:, 0]*tan_fact  # square

        # FRONT LEFT (ZONE 0)
        front_left = front[(mask_front+1.5*self.lane_width -
                            self.lane_offset) >= front[:, 1], :]
        mask_front_temp = mask_front[(mask_front+1.5*self.lane_width -
                                      self.lane_offset) >= front[:, 1]]
        front_left = front_left[(
            mask_front_temp+0.5*self.lane_width+self.lane_offset) < front_left[:, 1], :]

        # FRONT MIDDLE (ZONE 1)
        front_middle = front[(mask_front+0.5*self.lane_width-self.lane_offset)
                             >= front[:, 1], :]
        mask_front_temp = mask_front[(mask_front+0.5*self.lane_width-self.lane_offset)
                                     >= front[:, 1]]
        front_middle = front_middle[(
            mask_front_temp-0.5*self.lane_width+self.lane_offset) <= front_middle[:, 1]]

        # FRONT RIGHT (ZONE 2)
        front_right = front[(mask_front-0.5*self.lane_width -
                            self.lane_offset) > front[:, 1], :]
        mask_front_temp = mask_front[(mask_front-0.5*self.lane_width -
                                      self.lane_offset) > front[:, 1]]
        front_right = front_right[(
            mask_front_temp-1.5*self.lane_width+self.lane_offset) <= front_right[:, 1], :]

        # Back and Side:
        back_side = self.xy_points[self.xy_points[:, 0] <= 0, :]
        back_side = back_side[np.abs(back_side[:, 1]) <= 1.5*self.lane_width]

        # SIDE:
        side = back_side[back_side[:, 0] >= -self.car_len, :]

        # LEFT (ZONE 3)
        side_left = side[side[:, 1] > 0, :]

        # RIGHT (ZONE 4)
        side_right = side[side[:, 1] < 0, :]

        # BACK:
        back = back_side[back_side[:, 0] < -self.car_len, :]
        back = back[back[:, 0] > -self.car_len-self.laser_len, :]

        # BACK LEFT (ZONE 5)
        back_left = back[back[:, 1] > 0.5*self.lane_width, :]

        # BACK LEFT (ZONE 6)
        back_middle = back[back[:, 1] <= 0.5*self.lane_width, :]
        back_middle = back_middle[back_middle[:, 1] >= -0.5*self.lane_width, :]

        # BACK LEFT (ZONE 7)
        back_right = back[back[:, 1] < -0.5*self.lane_width, :]

        # Build and publish message
        msg_car = Car_scan()

        # True if more points are detected than the threshold is
        msg_car.zones[0] = front_left.size >= self.Activation_threshold
        msg_car.zones[1] = front_middle.size >= self.Activation_threshold
        msg_car.zones[2] = front_right.size >= self.Activation_threshold

        msg_car.zones[3] = side_left.size >= self.Activation_threshold
        msg_car.zones[4] = side_right.size >= self.Activation_threshold

        msg_car.zones[5] = back_left.size >= self.Activation_threshold
        msg_car.zones[6] = back_middle.size >= self.Activation_threshold
        msg_car.zones[7] = back_right.size >= self.Activation_threshold

        # publish angle for vizualisation
        msg_car.angle = self.curve

        self._is_car_pub.publish(msg_car)

        # Publish pointclouds to visualize the zones in rviz
        self._zone0_pub.publish(self._zone2point(front_left))
        self._zone1_pub.publish(self._zone2point(front_middle))
        self._zone2_pub.publish(self._zone2point(front_right))

        self.xy_points = np.zeros([0, 2])

    # Function used to detect steering angle and filter it over time
    def _set_curve_l_steer(self, cmd):
        self.curves = np.append(
            self.curves[1:], -cmd.process_value)  # rechts minus
        self.curve = np.dot(self.curves, self.alphas)
        # rospy.logwarn(f"Road curve: {self.curve:.04f}")

    # Function used to build a Pointcloud2 out of a given 2D array for publishing and visualization in Rviz
    def _zone2point(self, zone_array):
        zone_data = np.zeros(zone_array.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])

        zone_data['x'] = zone_array[:, 0]
        zone_data['y'] = zone_array[:, 1]

        msg_zone = ros_numpy.msgify(PointCloud2, zone_data)
        msg_zone.header.stamp = rospy.Time.now()
        msg_zone.header.frame_id = "laser"
        return msg_zone

    # Function determines if car is in a curve based on
    # its global position on the map
    def _check_for_curve(self, cmd):
        pos_rob = cmd.feedback.base_position.pose.position
        x_rob = pos_rob.x
        y_rob = pos_rob.y
        # Check if car is in curve based on global position using a state machine
        # Also the direction is determined so that the front zones are turned correctly

        if self.is_curve:
            if not (x_rob >= self.x_curve or x_rob <= -self.x_curve):
                rospy.loginfo("LaserChecker: Curve: False")  # print next state
                self.is_curve = False
        else:
            if x_rob >= self.x_curve or x_rob <= -self.x_curve:
                self.is_curve = True
                if y_rob*x_rob > 0:
                    self.is_curve_right = True
                else:
                    self.is_curve_right = False

                rospy.loginfo("LaserChecker: Curve: True")
                rospy.loginfo("LaserChecker: Curve_right: " +
                              str(self.is_curve_right))

    def _preprocess_points(self, msg):
        xyz_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        self.xy_points = np.vstack((self.xy_points, xyz_points[:, :2]))
        if self.debug:
            # Function used to export data from the sim for testing
            np.savetxt(self.rospack.get_path('lane_planner') +
                       '/xy.csv', self.xy_points, delimiter=',')


if __name__ == '__main__':
    checker = LaserChecker()
