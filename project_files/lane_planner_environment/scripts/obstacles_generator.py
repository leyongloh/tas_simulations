#! /usr/bin/env python
# coding: utf-8
###########################################################################
# @info: Generate static and dynamic obstacles, set up for each demo mode, publish goal pose
#
# @author: Chieh Lin
#
# @contact:ga85duq@mytum.de
###########################################################################

import rospy
import numpy as np
import math
import tf
import time
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SpawnModel
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from CatmullSpline import CatmullSpline
from pprint import pprint
import yaml
import rospkg
import random


def main():
    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('lane_planner_environment')

    # Load model
    f = open(pack_path+'/gazebo/models/pickup/model.sdf', 'r')
    spickup = f.read()
    b = open(pack_path+'/gazebo/models/box/model.sdf', 'r')
    sbox = b.read()
    p = open(pack_path+'/gazebo/models/person_walking/model.sdf', 'r')
    sperson = p.read()
    c = open(pack_path+'/gazebo/models/construction_barrel/model.sdf', 'r')
    sbarrel = c.read()

    # init Publisher and Service
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    mstate_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=100)
    spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Start node
    rospy.init_node('test_obst', anonymous=True)
    rate = rospy.Rate(500)  # 500hz

    # Load parameters via get_param
    obst_num = rospy.get_param("/obst_sim/obst_num")
    traj_speed = rospy.get_param("/obst_sim/traj_speed")
    demo_mode = rospy.get_param("/obstacles_generator/demo_mode")
    set_goal = rospy.get_param("/obstacles_generator/set_goal")
    goal_position_x = rospy.get_param("/obstacles_generator/goal_position_x")
    goal_position_y = rospy.get_param("/obstacles_generator/goal_position_y")
    goal_angle = rospy.get_param("/obstacles_generator/goal_angle")

    # Check demo_mode, and change obst_num accordingly to the chosen demo
    if demo_mode == 1:
        obst_num = 1
    if demo_mode == 2:
        obst_num = 2
    if demo_mode == 3:
        obst_num = 3
    if demo_mode == 4:
        obst_num = 4
    if demo_mode == 5:
        obst_num = 1
    if demo_mode == 6:
        obst_num = 4
    if demo_mode == 7:
        obst_num = 8
    if demo_mode == 8:
        obst_num = 3

    print("Number of obstacles to be spawned: " + str(obst_num))

    # Load trajectories from yaml file
    loaded_traj = load_trajectory()

    ct_spline = CatmullSpline()

    state_msg = ModelState()

    obst_name = string_array(obst_num)

    # demo_mode = 0 - Randomly spawning obstacles
    if demo_mode == 0:
        # Reassign trajectories randomly to obstacles
        assigned_traj = preprocess_rand(obst_num, loaded_traj)

        # Spawning obstacles based on assigned_traj
        for i in range(0, obst_num):
            start_pose = Pose()
            start_pose.position.x = assigned_traj[i][0][0]
            start_pose.position.y = assigned_traj[i][0][1]

            spawn_sdf_model(obst_name[i], spickup, obst_name[i], start_pose, 'world')
    
        rospy.logwarn("Demo 0 completed.")


    # demo_mode = 1 - Spawning 1 box in front of car
    if demo_mode == 1:
        start_pose = Pose()
        start_pose.position.x = loaded_traj[0][30][0]
        start_pose.position.y = loaded_traj[0][30][1]

        spawn_sdf_model(obst_name[0], sbox, obst_name[0], start_pose, 'world')
    
        rospy.logwarn("Demo 1 completed.")


    # demo_mode = 2 - Spawning 2 boxes
    if demo_mode == 2:
        start_pose1 = Pose()
        start_pose2 = Pose()
        start_pose1.position.x = loaded_traj[0][30][0]
        start_pose1.position.y = loaded_traj[0][30][1]
        start_pose2.position.x = loaded_traj[1][32][0]
        start_pose2.position.y = loaded_traj[1][32][1]

        spawn_sdf_model(obst_name[0], sbox, obst_name[0], start_pose1, 'world')
        spawn_sdf_model(obst_name[1], sbox, obst_name[1], start_pose2, 'world')

        rospy.logwarn("Demo 2 completed.")

    
    # demo_mode = 3 - Spawning 3 boxes (2 on lane 1 and 1 one lane 2)
    if demo_mode == 3:
        start_pose1 = Pose()
        start_pose2 = Pose()
        start_pose3 = Pose()
        start_pose1.position.x = loaded_traj[0][30][0]
        start_pose1.position.y = loaded_traj[0][30][1]
        start_pose2.position.x = loaded_traj[1][32][0]
        start_pose2.position.y = loaded_traj[1][32][1]
        start_pose3.position.x = loaded_traj[0][34][0]
        start_pose3.position.y = loaded_traj[0][34][1]

        spawn_sdf_model(obst_name[0], sbox, obst_name[0], start_pose1, 'world')
        spawn_sdf_model(obst_name[1], sbox, obst_name[1], start_pose2, 'world')
        spawn_sdf_model(obst_name[2], sbox, obst_name[2], start_pose3, 'world')

        rospy.logwarn("Demo 3 completed.")


    # demo_mode = 4 - Spawning 4 moving obstacles in correct direction
    if demo_mode == 4:
        # Assign trajectories to obstacles
        assigned_traj = preprocess_demo4(loaded_traj)

        # Spawning obstacles based on assigned_traj
        for i in range(0, obst_num):
            start_pose = Pose()
            start_pose.position.x = assigned_traj[i][0][0]
            start_pose.position.y = assigned_traj[i][0][1]

            spawn_sdf_model(obst_name[i], spickup, obst_name[i], start_pose, 'world')
    
        rospy.logwarn("Demo 4 completed.")


    # demo_mode = 5 - Spawning 1 moving obstacle as geisterfahrer
    if demo_mode == 5:
        # Assign trajectories to obstacles
        assigned_traj = preprocess_demo5(loaded_traj)

        start_pose = Pose()
        start_pose.position.x = assigned_traj[0][0][0]
        start_pose.position.y = assigned_traj[0][0][1]
        spawn_sdf_model(obst_name[0], spickup, obst_name[0], start_pose, 'world')

        rospy.logwarn("Demo 5 completed.")


    # demo_mode = 6 - Spawning 3 moving obstacels and 1 geisterfahrer driving towards the car
    if demo_mode == 6:
        # Assign trajectories to obstacles
        assigned_traj = preprocess_demo6(loaded_traj)

        # Spawning obstacles based on assigned_traj
        for i in range(0, obst_num):
            start_pose = Pose()
            start_pose.position.x = assigned_traj[i][0][0]
            start_pose.position.y = assigned_traj[i][0][1]

            spawn_sdf_model(obst_name[i], spickup, obst_name[i], start_pose, 'world')

        rospy.logwarn("Demo 6 completed.")


    # demo_mode = 7 - Spawning 8 static obstacles for curve
    if demo_mode == 7:
        start_pose1 = Pose()
        start_pose2 = Pose()
        start_pose3 = Pose()
        start_pose4 = Pose()
        start_pose5 = Pose()
        start_pose6 = Pose()
        start_pose7 = Pose()
        start_pose8 = Pose()
        start_pose1.position.x = loaded_traj[0][29][0]
        start_pose1.position.y = loaded_traj[0][29][1]
        start_pose2.position.x = loaded_traj[1][32][0]
        start_pose2.position.y = loaded_traj[1][32][1]
        start_pose3.position.x = 15.5
        start_pose3.position.y = loaded_traj[2][52][1]
        start_pose4.position.x = loaded_traj[0][40][0]
        start_pose4.position.y = loaded_traj[0][40][1]
        start_pose5.position.x = loaded_traj[0][41][0]
        start_pose5.position.y = loaded_traj[0][41][1]
        start_pose6.position.x = loaded_traj[0][42][0]
        start_pose6.position.y = loaded_traj[0][42][1]
        start_pose7.position.x = loaded_traj[0][30][0]
        start_pose7.position.y = loaded_traj[0][30][1]
        start_pose8.position.x = loaded_traj[0][31][0]
        start_pose8.position.y = loaded_traj[0][31][1]


        spawn_sdf_model(obst_name[0], sbox, obst_name[0], start_pose1, 'world')
        spawn_sdf_model(obst_name[1], sbox, obst_name[1], start_pose2, 'world')
        spawn_sdf_model(obst_name[2], sbox, obst_name[2], start_pose3, 'world')
        spawn_sdf_model(obst_name[3], sbox, obst_name[3], start_pose4, 'world')
        spawn_sdf_model(obst_name[4], sbox, obst_name[4], start_pose5, 'world')
        spawn_sdf_model(obst_name[5], sbox, obst_name[5], start_pose6, 'world')
        spawn_sdf_model(obst_name[6], sbox, obst_name[6], start_pose7, 'world')
        spawn_sdf_model(obst_name[7], sbox, obst_name[7], start_pose8, 'world')

        rospy.logwarn("Demo 7 completed.")
    

    # demo_mode = 8 - Spawning 3 static obstacles for curve in other direction
    if demo_mode == 8:
        start_pose1 = Pose()
        start_pose2 = Pose()
        start_pose3 = Pose()
        start_pose1.position.x = loaded_traj[3][70][0]
        start_pose1.position.y = loaded_traj[3][70][1]
        start_pose2.position.x = loaded_traj[3][71][0]
        start_pose2.position.y = loaded_traj[3][71][1]
        start_pose3.position.x = loaded_traj[2][72][0]
        start_pose3.position.y = loaded_traj[2][72][1]

        spawn_sdf_model(obst_name[0], sbarrel, obst_name[0], start_pose1, 'world')
        spawn_sdf_model(obst_name[1], sbarrel, obst_name[1], start_pose2, 'world')
        spawn_sdf_model(obst_name[2], sbarrel, obst_name[2], start_pose3, 'world')

        rospy.logwarn("Demo 7 completed.")


    # Publisher for goal position
    if set_goal:
    
        pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        time.sleep(0.5)
        
        orientation = np.array([0.0, 0.0, goal_angle])
        quat = np.array(tf.transformations.quaternion_from_euler(*orientation ))
        
        goal_pose = PoseStamped()
        goal_pose .header.frame_id = "map"
        goal_pose.pose.position.x = goal_position_x
        goal_pose.pose.position.y = goal_position_y
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = quat
        
        time.sleep(0.2)
        
        pub.publish(goal_pose)
    
        time.sleep(0.5)
        
        rospy.logwarn('goal pose published')


    t_start = rospy.get_time()


    try:
        while not rospy.is_shutdown():
            # pub.publish(set_ms())
            # rospy.loginfo('something':
            if demo_mode == 0 or demo_mode == 4 or demo_mode == 5 or demo_mode == 6:
                moving_obstacles(state_msg, assigned_traj, t_start, traj_speed,
                                 obst_name, ct_spline, obst_num, mstate_pub)
            rate.sleep()
    except:
        pass


# Generating string for each obstacle
def string_array(obst_num):
    obstacles_name = []
    for i in range(1, obst_num+1):
        obstacles_name.append("obstacle_" + str(i))

    return obstacles_name


# Load each trajectories and assigned it to load_traj
def load_trajectory():
    lane = []
    try:
        lanes = test = rospy.get_param("/lane_planner/trajectories/")
        loaded_traj = []
        for key, val in lanes.items():  # Adding every lane to an array(loaded_traj), in order for the obstacles to randomly choose a lane
            # print(len(val))
            loaded_traj.append(val)
    except IOError:
        rospy.logerr("Could not read control point file! File does not exist!")

    return loaded_traj


# Preprocess for demo 0, assigning trajectories randomly to each obstacle
def preprocess_rand(obst_num, loaded_traj):
    rand_traj_num = np.random.randint(len(loaded_traj), size=obst_num)
    # print(rand_traj_num)

    rand_traj = []
    for j in range(0, obst_num):
        rand_shift_num = np.random.randint(len(loaded_traj[rand_traj_num[j]]))
        shifted_points = np.roll(loaded_traj[rand_traj_num[j]], rand_shift_num, axis=0)
        rand_traj.append(shifted_points)

    return rand_traj


# Preprocess for demo 4
def preprocess_demo4(loaded_traj):
    demo_traj = []
    obst1_shifted = np.roll(loaded_traj[0], -29, axis=0)
    obst2_shifted = np.roll(loaded_traj[1], -30, axis=0)
    obst3_shifted = np.roll(loaded_traj[2], -40, axis=0)
    obst4_shifted = np.roll(loaded_traj[3], -36, axis=0)
    demo_traj = np.array([obst1_shifted, obst2_shifted, obst3_shifted, obst4_shifted], dtype=object)

    return demo_traj


# Preprocess for demo 5
def preprocess_demo5(loaded_traj):
    demo_traj = []
    obst1_shifted = np.roll(loaded_traj[0], -40, axis=0)
    # Flip the trajectory for Geisterfahrer
    obst1_shifted = obst1_shifted[::-1]
    demo_traj.append(obst1_shifted)

    return demo_traj


# Preprocess for demo 6
def preprocess_demo6(loaded_traj):
    demo_traj = []
    obst1_shifted = np.roll(loaded_traj[0], -43, axis=0)
    # Flip the trajectory for Geisterfahrer
    obst1_shifted = obst1_shifted[::-1]
    obst2_shifted = np.roll(loaded_traj[1], -29, axis=0)
    obst3_shifted = np.roll(loaded_traj[2], -46, axis=0)
    obst4_shifted = np.roll(loaded_traj[3], -42, axis=0)
    demo_traj = np.array([obst1_shifted, obst2_shifted, obst3_shifted, obst4_shifted], dtype=object)

    return demo_traj


def moving_obstacles(state_msg, assigned_traj, t_start, traj_speed, model_name, ct_spline, obst_num, mstate_pub):

    for i in range(0, obst_num):
        t_now = rospy.get_time()
        dt = (t_now - t_start)*traj_speed
        # Max in case ceiling is still 0 to avoid index out of range.
        traj_sect = max(1, int(math.ceil(dt)))

        dt += (1 - traj_sect)  # Norm to [0,1].

        if traj_sect < (len(assigned_traj[i]) - 2):
            state_msg.model_name = model_name[i]
            ct_spline.set_pts(assigned_traj[i][traj_sect - 1: traj_sect + 3])
            pos, agl, vel, agl_vel = ct_spline.get_spl(dt)
            agl += math.pi/2
            state_msg.pose.position.x, state_msg.pose.position.y = pos
            quat = np.array(tf.transformations.quaternion_from_euler(0, 0, agl))
            state_msg.pose.orientation.x, state_msg.pose.orientation.y, state_msg.pose.orientation.z, state_msg.pose.orientation.w = quat
            mstate_pub.publish(state_msg)


if __name__ == '__main__':
    main()
