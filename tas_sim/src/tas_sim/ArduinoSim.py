#! /usr/bin/env python
# coding: utf-8

## @file
# Contains the Arduino mimic for the simulation.

## @author Martin 
## @maintainer Jiangnan 

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import numpy as np
import math


## ArduinoSim Class to mimic the Arduino on the TAS car. 
# Performs all calculations for the Ackermann structure and publishes to the cars controllers.
# @warning Ackermann model used is based on kinematic model only, no vehicle dynamics are considered!
class ArduinoSim:
    
    ## Default constructor for the Arduino simulator.
    # Contains all needed constants and settings.
    def __init__(self):
        self._car_len = 0.35
        self._car_width = 0.19189
        self._linear_scale = 7200
        self._angle_scale = 3000 / math.pi
        self._ard_median = 1500
        self._steer_limit = 0.4  
        self._vel_limit = 15  
        self._wheel_rad = 0.07  
        self._vel_range = 300  
        self._ctrl_hz = 200
        self._vel_cmd_sub = rospy.Subscriber('arduino/servo', Vector3, self._handle_vel_cmd, queue_size=1)
        self._r_steer_pub = rospy.Publisher('tas_car/r_steer_controller/command', Float64, queue_size=1)
        self._l_steer_pub = rospy.Publisher('tas_car/l_steer_controller/command', Float64, queue_size=1)
        self._r_wheel_pub = rospy.Publisher('tas_car/r_wheel_controller/command', Float64, queue_size=1)
        self._l_wheel_pub = rospy.Publisher('tas_car/l_wheel_controller/command', Float64, queue_size=1)

    
    ## Initializes all subscribers and publishers for the Arduino simulator.
    # Control signals are calculated as callback from the 'arduino/servo' topic.
    def run(self):
        rospy.init_node('arduino_sim', anonymous=True)
        self._ctrl_rate = rospy.Rate(self._ctrl_hz)
        rospy.loginfo('Simulated Arduino running!')
        rospy.spin()

    ## Handles the incoming car commands.
    # Decodes the command, calculates the turn angle and radius, computes the individual wheel speeds and angles while considering their limits.
    # Publishes the translated commands to the low level controller interfaces of the robot model.
    # @param cmd Command from the subscriber containing the desired velocity and steering angle of the one-dimensional Ackermann model. Type Vector3.
    def _handle_vel_cmd(self, cmd):
        dec_vel, alpha = self._decode(cmd)
        beta = math.pi/2 - abs(alpha)  # Get the angle of the virtual middle wheel for the desired turn.
        turn_rad = np.tan(beta)*self._car_len  # Compute turn radius.
        if alpha > 0:
            beta_r = np.arctan((turn_rad + self._car_width)/self._car_len)  # Calculates right wheel orthogonal steering angle.
            beta_l = np.arctan((turn_rad - self._car_width)/self._car_len)  # Calculates left wheel orthogonal steering angle.
            alpha_r = math.pi/2 - abs(beta_r)  # Retransform to real steering values.
            alpha_l = math.pi/2 - abs(beta_l)
        else:
            beta_r = np.arctan((turn_rad - self._car_width)/self._car_len)  # Calculates right wheel orthogonal steering angle.
            beta_l = np.arctan((turn_rad + self._car_width)/self._car_len)  # Calculates left wheel orthogonal steering angle.
            alpha_r = - (math.pi/2 - abs(beta_r))  # Retransform to real steering values.
            alpha_l = - (math.pi/2 - abs(beta_l))
        
        alpha, alpha_r, alpha_l = self._limit_steer(alpha, alpha_r, alpha_l)  # Take limits into account for maximum steering possible. Also adjust original virtual steering angle accordingly.
        vel_r, vel_l = self._diff_drive(dec_vel, alpha)  # Calculate speed differential of the wheels.

        self._pub_cmd(alpha_r, alpha_l, vel_r, vel_l)  # Publish calculated controls.
        self._ctrl_rate.sleep()  # Maintain controller loop rate.

    ## Decodes the command according to the encoding in the tas_autonomous_control C++ files.
    # @param cmd Command from the subscriber containing the desired velocity and steering angle of the one-dimensional Ackermann model. Type Vector3.
    # @return Returns the decoded velocity and angle of the one-dimensional Ackermann model. Type list(2).
    def _decode(self, cmd):
        dec_vel = self._decode_vel(cmd.x)
        dec_agl = self._decode_agl(cmd.y)
        return dec_vel, dec_agl
    
    ## Decodes the velocity according to the encoding in the tas_autonomous_control C++ files.
    # @param enc_vel Encoded velocity of the one-dimensional Ackermann model. Type float.
    # @return Returns the decoded velocity of the one-dimensional Ackermann model. Type float.
    def _decode_vel(self, enc_vel):
        return (enc_vel - self._ard_median) / self._linear_scale

    ## Decodes the angle according to the encoding in the tas_autonomous_control C++ files.
    # @param enc_agl Encoded angle of the one-dimensional Ackermann model. Type float.
    # @return Returns the decoded angle of the one-dimensional Ackermann model in radians. Type float.
    def _decode_agl(self, enc_agl):
        return (self._ard_median - enc_agl) / self._angle_scale

    ## Limits the steering to the maximum steering angle of the car in case a greater steer was desired.
    # @param alpha Desired steering angle of the one-dimensional Ackermann model in radians. Type float.
    # @param alpha_r Calculated steering angle of the right wheel in the two-dimensional Ackermann model in radians. Type float.
    # @param alpha_l Calculated steering angle of the left wheel in the two-dimensional Ackermann model in radians. Type float.
    # @return Returns the steering angles (alpha, alpha_r, alpha_l) that consider the maximum steering limits. Type list(3).
    def _limit_steer(self, alpha, alpha_r, alpha_l):
        # Only checks right for negative and left for positive, since for each case the respective one is always more inclined.
        if alpha_r < -self._steer_limit:
            alpha_r = -self._steer_limit
            alpha_l = -(math.pi/2 - np.arctan((np.tan(math.pi/2 - abs(alpha_r))*self._car_len + 2 * self._car_width)/self._car_len))  # Also change left steering if corrected.
            alpha = -(math.pi/2 - np.arctan((np.tan(math.pi/2 - abs(alpha_r))*self._car_len + self._car_width)/self._car_len))
        elif alpha_l > self._steer_limit:
            alpha_l = self._steer_limit
            alpha_r = math.pi/2 - np.arctan((np.tan(math.pi/2 - alpha_l)*self._car_len + 2 * self._car_width)/self._car_len)  # Also change right steering if corrected.
            alpha = math.pi/2 - np.arctan((np.tan(math.pi/2 - alpha_l)*self._car_len + self._car_width)/self._car_len)
        return alpha, alpha_r, alpha_l

    ## Calculates the rear axis differential with the one-dimensional ideal steering angle and the differential drive formulars.
    # @param dec_vel Decoded velocity of the one-dimensional Ackermann model. Type float.
    # @param alpha Desired steering angle of the one-dimensional Ackermann model in radians. Type float.
    # @return Returns the right and left rear wheel velocities as floats. Type list(2).
    def _diff_drive(self, dec_vel, alpha):
        beta = math.pi/2 - abs(alpha)
        turn_rad = np.tan(abs(beta)) * self._car_len  # Calculates the turn circle radius.
        if turn_rad == 0:
            vel_r = dec_vel
            vel_l = dec_vel
        elif alpha > 0:
            vel_r = (1 + self._car_width/(turn_rad * 2)) * dec_vel
            vel_l = (1 - self._car_width/(turn_rad * 2)) * dec_vel
        else:
            vel_r = (1 - self._car_width/(turn_rad * 2)) * dec_vel
            vel_l = (1 + self._car_width/(turn_rad * 2)) * dec_vel
        vel_r, vel_l = self._vel_rad_conv(vel_r, vel_l)
        return vel_r, vel_l
    
    ## Converts the rear wheel velocities to radians/s.
    # Also limits the scaled velocities to the maximum wheel speed.
    # @param vel_r Right rear wheel speed. Type float.
    # @param vel_l Left rear wheel speed. Type float.
    # @return Returns vel_r, vel_l in radians/s and limited to the maximum speed. Type list(2).
    def _vel_rad_conv(self, vel_r, vel_l):
        tmp_mlt = (self._vel_limit * self._linear_scale)/ (self._vel_range)
        vel_r *= tmp_mlt
        vel_l *= tmp_mlt
        vel_r, vel_l = self._limit_vel(vel_r, vel_l)
        return vel_r, vel_l
    
    ## Limits the velocity of the rear wheels to the maximum speed.
    # Necessary when turning the car at max speed makes the outer wheel go faster than max speed according to the differential drive formulas.
    # @param vel_r Right rear wheel speed in radians/s. Type float.
    # @param vel_l Left rear wheel speed in radians/s. Type float.
    # @return Returns vel_r, vel_l in radians/s and limited to the maximum speed. Type list(2).
    def _limit_vel(self, vel_r, vel_l):
        vel_max = max(abs(vel_r),abs(vel_l))
        if vel_max > self._vel_limit:
            tmp_div = vel_max / self._vel_limit
            vel_r /= tmp_div
            vel_l /= tmp_div
        return vel_r, vel_l

    ## Publishes the calculated values for front steering angles and rear wheel velocities.
    # @param alpha_r Calculated steering angle of the right wheel in the two-dimensional Ackermann model in radians. Type float.
    # @param alpha_l Calculated steering angle of the left wheel in the two-dimensional Ackermann model in radians. Type float.
    # @param vel_r Right rear wheel speed in radians/s. Type float.
    # @param vel_l Left rear wheel speed in radians/s. Type float.
    def _pub_cmd(self, alpha_r, alpha_l, vel_r, vel_l):
        self._r_steer_pub.publish(alpha_r)
        self._l_steer_pub.publish(alpha_l)
        self._r_wheel_pub.publish(vel_r)
        self._l_wheel_pub.publish(vel_l)
        
    ## @var _car_len
    # Length of the car from the rear axis to the front axis.
    
    ## @var _car_width
    # Distance from the middle of the car to the wheel link.
    
    ## @var _linear_scale
    # Linear scale from the tas_autonomous_control source code.
    
    ## @var _angle_scale
    # Angular scale from the tas_autonomous_control source code.
    
    ## @var _ard_median
    # 0 value in the arduino control messages.
    
    ## @var _steer_limit
    # Joint limit from the URDF for Ackermann drive joints. Might need adjustment.
    
    ## @var _vel_limit
    # Velocity limit in radians/s. Automatically defines the dynamic range for the rear wheel speeds.
    
    ## @var _wheel_rad
    # Wheel radius of the car.
    
    ## @var _vel_range
    # Nominal range of the speed input in one direction (from 1000 to 2000 with 1500 being zero, see tas_autonomous_control code).
    
    ## @var _ctrl_hz
    # Control loop target frequency in Hz.
    
    ## @var _vel_cmd_sub 
    # ROS subscriber to 'arduino/servo' topic messages of type Vector3.
    
    ## @var _r_steer_pub
    # ROS publisher for 'tas_car/r_steer_controller/command' topic messages of type Float.
    
    ## @var _l_steer_pub
    # ROS publisher for 'tas_car/l_steer_controller/command' topic messages of type Float.
    
    ## @var _r_wheel_pub
    # ROS publisher for 'tas_car/r_wheel_controller/command' topic messages of type Float.
    
    ## @var _l_wheel_pub
    # ROS publisher for 'tas_car/l_wheel_controller/command' topic messages of type Float.

## Main function creates an ArdunioSim class object and runs it.
def main():
    arduino = ArduinoSim()
    arduino.run()

if __name__ == '__main__':
    main()
