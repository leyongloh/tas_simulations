#! /usr/bin/env python
# coding: utf-8

## @file
# Contains the autonomous control node mimic for the simulation.

## @author Martin 
## @maintainer Jiangnan

import rospy
from geometry_msgs.msg import Vector3, Twist
import numpy as np
import math
import rosnode


## ACNSim Class to mimic the autonomous control node on the TAS car. 
class ACNSim:
    
    ## Default constructor for the autonomous control node simulator.
    # Contains all needed constants and settings.
    def __init__(self):
        self._car_len = 0.35  
        self._linear_scale = 7200
        self._angle_scale = 3000 / math.pi  
        self._ard_median = 1500  
        self._ctrl_hz = 200
        self._cmd_vec = Vector3()
        self._vel_cmd_sub = rospy.Subscriber('cmd_vel', Twist, self._handle_vel_cmd, queue_size=3)
        self._servo_pub = rospy.Publisher('servo', Vector3, queue_size=2)
    
    ## Initializes the ROS node and sets the controller rate.
    # Control signals are calculated as callback from the 'cmd_vel' topic.
    def run(self):
        rospy.init_node('acn_sim', anonymous=True)
        rospy.loginfo('Simulated Autonomous Control Node running!')
        self._ctrl_rate = rospy.Rate(self._ctrl_hz)
        rospy.spin()

    ## Handles the incoming velocity commands.
    def _handle_vel_cmd(self, cmd):
        lin_vel, agl_vel = cmd.linear.x, cmd.angular.z
        if ((lin_vel < 0.01) and (lin_vel > -0.01)):
            cmd_vel = self._ard_median
            cmd_agl = self._ard_median
        else:
            cmd_vel = self._ard_median + lin_vel*self._linear_scale  # Encode according to tas_autonomous_control_node C++ code.
            tmp_agl = np.arctan((agl_vel * self._car_len)/lin_vel)
            cmd_agl = 1500 - self._angle_scale * tmp_agl

        cmd_vel, cmd_agl = self._limit_cmd(cmd_vel, cmd_agl)
        self._pub_cmd(cmd_vel, cmd_agl)  # Publish calculated controls.
        self._ctrl_rate.sleep()  # Maintain controller loop rate.

    ## Limits the velocity commands to the values defined in the C++ files of tas_autonomous_control.
    # @param cmd_vel Unconstrained linear velocity command. Type float.
    # @param cmd_agl Unconstrained angular velocity command. Type float.
    # @return Returns the constrained linear and angular velocity. Type list(2).
    def _limit_cmd(self, cmd_vel, cmd_agl):
        if cmd_vel > 1800:
            cmd_vel = 1800
        elif cmd_vel < 1200:
            cmd_vel = 1200
        if cmd_agl > 2000:
            cmd_agl = 2000
        elif cmd_agl < 1000:
            cmd_agl = 1000
        return cmd_vel, cmd_agl
    
    ## Publishes the velocity commands for the ACNSim node.
    def _pub_cmd(self, cmd_vel, cmd_agl):
        self._cmd_vec.x, self._cmd_vec.y = cmd_vel, cmd_agl
        self._servo_pub.publish(self._cmd_vec)
        
    ## @var _car_len
    # Length of the car from the rear axis to the front axis.
    
    ## @var _linear_scale
    # Linear scale from the tas_autonomous_control source code.
    
    ## @var _angle_scale
    # Angular scale from the tas_autonomous_control source code.
    
    ## @var _ard_median
    # 0 value in the arduino control messages.
    
    ## @var _ctrl_hz
    # Control loop target frequency in Hz.
    
    ## @var _cmd_vec
    # Initialized Vector3 message to publish servo messages.
    
    ## @var _vel_cmd_sub
    # ROS subscriber to 'vel_cmd' topic messages of type Twist.
    
    ## @var _servo_pub
    # ROS publisher for 'servo' topic messages of type Vector3.

## Main function creates an AutonomousNodeSim class object and runs it.
def main():
    rospy.sleep(3)  # Give tas_autonomous_control_node time to come up if launched.
    try:
        nodes = rosnode.get_node_names()
        if '/tas_autonomous_control_node' in nodes:
            print('tas_autonomous_control_node is running. \nNo ACNSim required.')
            return
        else:
            print('tas_autonomous_control_node missing. Launching ACNSim instead.')
            acnsim = ACNSim()  # Launches the ACNSim in case the package is missing to enable autonomous driving in simulation.
            acnsim.run()
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
