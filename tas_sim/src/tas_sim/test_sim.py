#! /usr/bin/env python
# coding: utf-8

## @file
# Contains the default test procedure script.

## @author Martin
## @maintainer Jiangnan 

import rospy
import numpy as np
import math
import tf
import threading
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal
from CatmullSpline import CatmullSpline
import yaml
import rospkg

## ObjectController class containing the loop to control objects in the simulation.
# Object's position and orientation are accessed over gazebos set_model_state topic. 
# The objects are moved along Catmull-Rom Splines created from control points read from a yaml config file.
# For the structure of the yaml file, please refer to the file itself and the following functions.
# @see load_files function documentation
# @see CatmullSpline class documentation
# @note More objects to control inside the simulation results in increasing impact on performance.
class ObjectController:

    ## Constructor for the ObjectController class.
    # Initializes flags, the model state publisher and some constants that might need adjustment. 
    # Also prepares threading for the synchronizing loop to make it run independently from the main code. 
    # @param cp_dict Control point dictionary holding the names and control points for all objects to be controlled. Type dict.
    # @param traj_speed Trajectory time parameterization multiplication factor. Determines overall object speed. Type float.
    def __init__(self, cp_dict, traj_speed):
        self._sync = False
        self._ms = ModelState()
        self._ms_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
        self._hz = 1000
        self._sd = False
        self._cp_dict = cp_dict
        self._t_start = 0
        self._traj_speed = traj_speed
        self._ct_spline = CatmullSpline()
        self._syn_thread = threading.Thread(target=self._syn_obj, args=())
        self._syn_thread.daemon = True

    ## Starts the synchronizing loop thread and initializes the loop rate.
    # @warning ROS node has to be initialized for this function to be callable without error.
    def run(self):
        self._sd = False
        self._rate = rospy.Rate(self._hz)
        rospy.loginfo("Object controller running!")
        self._syn_thread.start()
    
    ## Sets the synchronization flag for the threaded loop.
    # If set to true, also starts the start time used to parameterize the trajectories.
    # @param val Value the flag should be changed to. Type bool.
    def set_sync(self, val):
        self._sync = val
        if val == True:
            self._t_start = rospy.get_time()
            while self._t_start == 0:  # Avoids rospy bug that sometimes initializes timer to 0.
                self._t_start = rospy.get_time()

    ## Object control loop function responsible for updating the registered objects's position.
    # @note Should always be called by a Thread object. Function loop will block further code execution otherwise.
    def _syn_obj(self):
        while not rospy.is_shutdown() and not self._sd:
            if self._sync:
                self._set_ms()
            self._rate.sleep()

    ## Sets and publishes the states of all models included in the control point dictionary _cp_dict.
    # Initializes the control points for the Catmull-Rom Spline, gets the current ROS time and parameterizes the trajectory accordingly.
    # Trajectories are uniformly scaled by the _traj_speed multiplicator.
    # Obtained position and orientation on the trajectory is used to update the models in the simulation.
    # Shuts down when the last end of all trajectories has been reached.
    def _set_ms(self):
        for key, val in self._cp_dict.items():
            t_now = rospy.get_time()
            dt = (t_now - self._t_start)*self._traj_speed  # Get parameterization with modifier.
            traj_sect = max(1, int(math.ceil(dt)))  # Max in case ceiling is still 0 to avoid index out of range.
            dt += (1 - traj_sect)  # Norm to [0,1].
            if traj_sect < (len(val[1]) - 2):  # Checks for end of control point list.
                self._ms.model_name = val[0]
                self._ct_spline.set_pts(val[1][traj_sect - 1 : traj_sect + 3])  # Set the current 4 control points.
                pos, agl, vel, agl_vel = self._ct_spline.get_spl(dt)
                agl += math.pi/2  # Additional rotation for the person_walking models in order to get the correct orientation.
                self._ms.pose.position.x, self._ms.pose.position.y = pos
                quat = np.array(tf.transformations.quaternion_from_euler(0,0,agl))
                self._ms.pose.orientation.x, self._ms.pose.orientation.y, self._ms.pose.orientation.z, self._ms.pose.orientation.w = quat
                self._ms_pub.publish(self._ms)
        if not max([len(arr[1]) for arr in self._cp_dict.values()])-3 >= traj_sect:
            rospy.loginfo('Dynamic object movement finished!')
            self.set_sync(False)
            self._sd = True

    ## Shuts down the object synchronization loop and cleanly finishes the thread.
    def shutdown(self):
        self._sd = True
        self._syn_thread.join()
    
    ## Interface function to check the current shutdown status.
    def is_shutdown(self):
        return self._sd

    ## @var _sync
    # Flag to endable/disable the model synchronization loop. Type bool.

    ## @var _ms
    # Model state that is filled with the model name and position/orientation to be published by _ms_pub.

    ## @var _ms_pub
    # ROS publisher for '/gazebo/set_model_state' topic messages of type ModelState.

    ## @var _hz
    # Update rate for the synchronization loop in Hz. Type int.
    
    ## @var _rate
    # Update rate for the synchronization loop. Created later when ROS node is running. Type rospy.Rate.

    ## @var _sd
    # Shutdown flag to shut down the whole controller. Type bool.
    
    ## @var _cp_dict
    # Control point dictionary containing the name and the trajectory control points for each object. Type dict.
    
    ## @var _t_start
    # ROS starting time from the last time set_sync was called.
    
    ## @var _traj_speed
    # Trajectory time parameterization multiplication factor. Determines overall object speed. Type float.
    
    ## @var _ct_spline
    # Catmull-Rom spline used to calculate the current desired object position. Type CatmullSpline.
    
    ## @var _syn_thread
    # Threading object initialized to run _syn_obj() in a seperate thread. Type threading.Thread.


## Function to load the test config and trajectory yaml file under tas_sim/config/ into a specific format..
# @warning Always use this function to create the control point dictionary unless you really understood the structure of the cp_dict.
# @return Returns the settings and the control point dictionary holding the control points for each object and the objects name. Type list(2).
def load_files():
    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('tas_sim')
    cp_dict = {}
    try:
        with open(pack_path + '/config/test_trajectories.yaml', 'r') as stream:
            data = yaml.safe_load(stream)
        for key, val in data.items():
            if val['active']:
                cp_arr = np.zeros((len(val['cps'].keys())+2,2), dtype='float')
                for i in range(1,len(val['cps'].keys())+1):
                    cp_arr[i] = val['cps'][i]
                cp_arr[0] = cp_arr[2]
                cp_arr[-1] = cp_arr[-3]
                cp_dict[int(key.replace('trajectory',''))] = (val['model_name'], cp_arr)
    except IOError:
        rospy.logerr("Could not read control point file! File does not exist!")

    try:
        with open(pack_path + '/config/test_config.yaml', 'r') as stream:
            settings = yaml.safe_load(stream)
    except IOError:
        rospy.logerr("Could not read settings config file! File does not exist!")
    except KeyError:
        rospy.logerr("Error in config file format! Check your file again!")
        
    return settings, cp_dict

## Sets the start pose for the standard test case inside the simulation.
# Creates the required PoseWithCovarianceStamped message from a given position and orientation and published it to '/initpose' for AMCL.
# @param pos Starting position of the car in the frame map. Note that the map frame does not necessarily coincide with the map loaded by hector_map. Type np.array(3,1).
# @param orient Starting orientation in RPY angles of the car in the frame map. Note that the map frame does not necessarily coincide with the map loaded by hector_map. Type np.array(3,1).
def set_start_pose(pos, orient):
    init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    quat = np.array(tf.transformations.quaternion_from_euler(*orient))
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x, pose.pose.pose.position.y = pos
    pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = quat

    rospy.sleep(0.2)  # Give ROS publishers time to come up, otherwise publishing fails.
    init_pub.publish(pose)
    rospy.sleep(0.1)  # Wait to make sure ROS publishes correctly.
    rospy.loginfo('Initialized start pose.')

## Sets the goal pose for the standard test case inside the simulation.
# Creates the required MoveBaseActionGoal message from a given position and orientation and published it to '/movebase/goal' for the move_base node.
# @param pos Goal position of the car in the frame map. Note that the map frame does not necessarily coincide with the map loaded by hector_map. Type np.array(3,1).
# @param orient Goal orientation in RPY angles of the car in the frame map. Note that the map frame does not necessarily coincide with the map loaded by hector_map. Type np.array(3,1).
def set_goal_pose(pos, orient):
    goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
    
    quat = np.array(tf.transformations.quaternion_from_euler(*orient))
    pose = MoveBaseActionGoal()
    pose.goal.target_pose.header.frame_id = "map"
    pose.goal.target_pose.pose.position.x, pose.goal.target_pose.pose.position.y = pos
    pose.goal.target_pose.pose.orientation.x, pose.goal.target_pose.pose.orientation.y, pose.goal.target_pose.pose.orientation.z, pose.goal.target_pose.pose.orientation.w = quat
    
    rospy.sleep(0.2)  # Give ROS publishers time to come up, otherwise publishing fails.
    goal_pub.publish(pose)
    rospy.sleep(0.1)  # Wait to make sure ROS publishes correctly.
    rospy.loginfo('Initialized goal pose.')

## Main function for the test proceedure. Loads the config file, sets start and goal poses, initializes the ObjectController and runs the control loop.
# The dynamic object movement starts as soon as the car is detected in a 1 meter radius around the test trigger point.
# @note Depending on the tests, you might want to change the initial position, goal or trigger position for the test proceedure. 
# @note Changes can be made in the test config file under /config/test_config.yaml.
def main():
    
    # Load the yaml config files.
    settings, cp_dict = load_files()

    # Approximate initial position for the robot inside the simulation. Is equal to the default spawning pose of the tas_car. Type np.array(2,1).
    init_pos = np.array(settings['initial_pose']['position'])
    # Approximate initial orientation for the robot inside the simulation. Is equal to the default spawning pose of the tas_car. Given as RPY angles. Type np.array(3,1).
    init_orient = np.array(settings['initial_pose']['orientation'])
    
    # Goal position for the robot inside the simulation. Type np.array(2,1).
    goal_pos = np.array(settings['goal_pose']['position'])
    # Goal orientation for the robot inside the simulation. Given as RPY angles. Type np.array(3,1).
    goal_orient = np.array(settings['goal_pose']['orientation'])
    
    # Trigger position that starts the dynamic movement if the car comes within a radius of 1m inside the simulation. Type np.array(2,1)
    trigger_pos = np.array(settings['trigger_position'])

    traj_speed = settings['trajectory_speed']
    obj_ctrl = ObjectController(cp_dict, traj_speed)
    
    rospy.init_node('tas_dynamic_test_node', anonymous=True)
    obj_ctrl.run()
    
    # Register the start and goal pose for the move_base node and 
    set_start_pose(init_pos, init_orient)
    rospy.sleep(0.2)  # Let the initial position register.
    set_goal_pose(goal_pos, goal_orient)
    
    print("""
          ######################################
          ##   Dynamic object test started!   ##
          ######################################
          ##       Make sure to run the       ##
          ##   wii_controller and set it to   ##
          ##    autonomous mode to enable     ##
          ##    driving in the simulation!    ##
          ######################################
          ##  Object movement starts once the ##
          ##   car passes the trigger point!  ##
          ######################################
          ## Start, goal and trigger pose can ##
          ##  be adjusted in the config file! ##
          ######################################
          """)
    test_len = max([len(arr[1]) for arr in cp_dict.values()]) - 3
    pos_check = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    while not rospy.is_shutdown():
        ms = pos_check.call('tas_car', '')
        car_pos = np.array([ms.pose.position.x, ms.pose.position.y])
        if np.linalg.norm(car_pos -  trigger_pos) > 1:  # Check if the car has entered the radius.
            rospy.sleep(0.25)
        else:
            obj_ctrl.set_sync(True)  # Dynamic object movement starts, will automatically stop when trajectories stop.
            break
    i = 0.0
    rospy.loginfo('Dynamic object test running...')
    while not obj_ctrl.is_shutdown():
        rospy.sleep(1)
        i += 1
        print('Test progress: ' + str(int(100*(i/test_len)*traj_speed))+ '%')
    rospy.loginfo('Dynamic object test finished, shutting down.')
    rospy.sleep(0.1)
    
if __name__ == '__main__':
    main()
