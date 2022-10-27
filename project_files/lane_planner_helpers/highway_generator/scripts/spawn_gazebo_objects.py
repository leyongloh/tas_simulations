#!/usr/bin/env python

"""""" """""" """""" """""" """""" """""" """""" """""" """""" """""" """""" """""" """""
@author: Fabian Zillenbiller

@info: Rosnode to spawn objects into empty Gazebo world

@contact: ga27seq@tum.de

""" """""" """""" """""" """""" """""" """""" """""" """""" """""" """""" """""" """""" ""


import yaml
import glob
from re import X
import rospy, tf
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
from rospkg import RosPack


if __name__ == "__main__":
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Got it.")
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    env_package_path = RosPack().get_path("lane_planner_environment")
    highway_gen_path = RosPack().get_path("highway_generator")

    yaml_path = highway_gen_path + "/config/*.yaml"

    txtfiles = []
    for file in glob.glob(yaml_path):
        txtfiles.append(file)

    for i in range(len(txtfiles)):
        obj_name = txtfiles[i][len(highway_gen_path) + 8 : -5]

        path = env_package_path + "/gazebo/models/" + obj_name + "/model.sdf"
        with open(path, "r") as f1:
            obj_xml = f1.read()

        path = highway_gen_path + "/config/" + obj_name + ".yaml"
        with open(path, "r") as f2:
            loaded_coord = yaml.safe_load(f2)

        for i in range(len(loaded_coord[0])):
            for j in range(0, int(len(loaded_coord) / 3)):
                bin_x = loaded_coord[3 * j][i]
                bin_y = loaded_coord[3 * j + 1][i]
                orient = tf.transformations.quaternion_from_euler(
                    0, 0, loaded_coord[3 * j + 2][i]
                ).tolist()

                item_name = "{0}_{1}.{2}".format(obj_name, i, j)
                print("Spawning model: ", item_name)
                item_pose = Pose()
                item_pose.position.x = bin_x
                item_pose.position.y = bin_y
                item_pose.position.z = 0
                item_pose.orientation.x = orient[0]
                item_pose.orientation.y = orient[1]
                item_pose.orientation.z = orient[2]
                item_pose.orientation.w = orient[3]

                spawn_model(item_name, obj_xml, "", item_pose, "world")
