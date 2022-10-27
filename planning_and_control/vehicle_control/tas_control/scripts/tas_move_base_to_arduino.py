#!/usr/bin/env python3

"""
     Node description =
     This node subsribes to the topic cmd_vel of the move_base, which publishes
     lienar and angular velocity commconds to it. In the case of TAS, the move_base
     only uses linear velocities in x-direction and angular velocities around the
     yaw axis. These velocity commands are comparable to the two-wheeled kinematic
     model. However, the TAS vehicles are car-like robots, which require different
     velocity commands. For that reason, the linear and angular velocities from
     the topic cmd_vel are transformed into movement commands of the kinematic
     bicycle model. Here, the kinematic bicycle model is used with the reference
     point set on the rear axle refer to
     https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357 or
     https://thef1clan.com/2020/09/21/vehicle-dynamics-the-kinematic-bicycle-model/
     for more information).
     Additionally, the movement commands are scaled, mapped and limited, so that they
     can be processed by the hardware controller. The derived
     movement commands are then published to the servo topic.
"""

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist


class Control:

    def __init__(self):
        """
            Constructor
        """
        # publisher and subscriber
        self.control_servo_pub = rospy.Publisher('servo', Vector3, queue_size=1)  # publisher
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)  # subscriber

        # encoding parameters
        self.linear_scale = 7200  # scaling for linear velocity, so that the velocity can be casted to integer
        self.angle_scale = (500 / 30) * (180 / np.pi)  # scaling for angular velocity, so that the velocity can be casted to integer
        self.wheel_base = 0.3588  # in meters

        # variables
        self.cmd_speed = 0  # scaled and mapped linear speed
        self.cmd_angle = 0  # scaled and mapped steering angle
        self.control_servo = Vector3(0, 0, 0)  # publish container

        self.initialisation()  # initializing the topic servo with (1500, 1500, 0)

    def initialisation(self):
        """
            Initializes the topic servo with not moving commands until a goal is set.
        """
        rospy.sleep(0.5)  # waiting 0.5 s until nodes and topics are up
        self.control_servo_pub.publish(Vector3(1500, 1500, 0))
        rospy.loginfo('initialized')

    def callback_cmd_vel(self, data):
        """
            This method is called, when new velocity commands have been published to the topic cmd_vel.
            These new velocity commands are then ... model of move base ...
        """
        rospy.loginfo('Automatic Control! Use the AUX switch on the controller to trigger this mode')
        # getting data  from topic cmd_vel
        cmd_linear_velocity = data.linear.x
        cmd_angular_velocity = data.angular.z
        
        # calculation of the linear velocity and the steering angle and scaling and mapping
        if abs(cmd_linear_velocity) < 0.000001:  # checking == 0 for floating-point values
            self.cmd_speed = 1500  # forcing no movement in linear direction
            self.cmd_angle = 1500  # stop steering while no linear velocity
        else:
            self.cmd_speed = 1500 + int(cmd_linear_velocity * self.linear_scale)  # encoding of linear velocity
            steering_angle = np.arctan(
                cmd_angular_velocity / cmd_linear_velocity * self.wheel_base)  # calculation of the steering angle
            if abs(steering_angle) < 0.05:   # checking == 0 for floating-point values
                steering_angle = 0  # forcing 0 steering angle
            self.cmd_angle = 1500 - int(steering_angle * self.angle_scale)  # encoding of the steering angle

        # considering input saturation of linear velocity
        if self.cmd_speed > 1900:  # don't enter values greater than 2000
            self.control_servo.x = 1900 
        elif self.cmd_speed < 1100: # don't enter values less than 1000
            self.control_servo.x = 1100
        else:
            self.control_servo.x = self.cmd_speed

        # considering input saturation of the steering angle
        if self.cmd_angle > 2000:  # don't enter values greater than 2000
            self.control_servo.y = 2000
        elif self.cmd_angle < 1000:  # don't enter values less than 1000
            self.control_servo.y = 1000 
        else:
            self.control_servo.y = self.cmd_angle

        self.publish_control_servo()

    def publish_control_servo(self):
        self.control_servo_pub.publish(self.control_servo)


if __name__ == '__main__':
    rospy.init_node('tas_move_base_to_arduino', anonymous=False)
    Control()
    rospy.loginfo('Autonomous Control is active') 
    rospy.spin()
