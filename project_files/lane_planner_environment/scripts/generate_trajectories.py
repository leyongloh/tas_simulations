#! /usr/bin/env python
###########################################################################
# @info: script to generate highway lane trajectories into yaml file
#
# @author: Fabian Zillenbiller
#
# @contact: ga27seq@tum.de
###########################################################################

# import matplotlib.pyplot as plt
import numpy as np
import yaml
import os

import argparse
from rospkg import RosPack

parser = argparse.ArgumentParser()
parser.add_argument(
    "--plot", action="store_true", help="Plot the generated trajectories"
)
args = parser.parse_args()

ds = 20

lane_length = 500
curve_diam = 200

rad_curve = curve_diam / 2

scale = 1 / 4  # ~40 / 155.3


def get_coordinates(offset):
    dphi = (ds * 180) / (np.pi * (rad_curve + offset))

    road_center = np.array([[], []])

    # lane 1
    x = -lane_length / 2
    while x < lane_length / 2 - ds / 2:
        road_center = np.append(road_center, [[x], [-rad_curve - offset]], axis=1)
        x += ds

    # curve 1
    phi = 270
    while phi < 360 - dphi / 2:
        x = lane_length / 2 + (rad_curve + offset) * np.cos(phi * np.pi / 180)
        y = (rad_curve + offset) * np.sin(phi * np.pi / 180)
        road_center = np.append(road_center, [[x], [y]], axis=1)
        phi += dphi

    phi = 0
    while phi < 90 - dphi / 2:
        x = lane_length / 2 + (rad_curve + offset) * np.cos(phi * np.pi / 180)
        y = (rad_curve + offset) * np.sin(phi * np.pi / 180)
        road_center = np.append(road_center, [[x], [y]], axis=1)
        phi += dphi

    # lane 2
    x = lane_length / 2
    while -lane_length / 2 + ds / 2 < x:
        road_center = np.append(road_center, [[x], [rad_curve + offset]], axis=1)
        x -= ds

    # curve 2
    phi = 90
    while phi < 270 - dphi / 2:
        x = -lane_length / 2 + (rad_curve + offset) * np.cos(phi * np.pi / 180)
        y = (rad_curve + offset) * np.sin(phi * np.pi / 180)
        road_center = np.append(road_center, [[x], [y]], axis=1)
        phi += dphi
    return road_center


def prepare_trajectory(lane):
    # Change lane data to list format to be more compact in ROS parameter server
    lane_data = []
    lane_dim = lane.shape
    keys = range(lane_dim[1])
    for i in keys:
        lane_data.append([float(lane[0, i] * scale), float(lane[1, i] * scale)])

    return lane_data


lane_0 = np.flip(get_coordinates(-5.625), axis=1)
lane_1 = np.flip(get_coordinates(-1.875), axis=1)
lane_2 = get_coordinates(1.875)
lane_3 = get_coordinates(5.625)

for i in range(4):
    exec(f"data_lane{i} = prepare_trajectory(lane_{i})")

lanes = dict(
    lane0=data_lane0,
    lane1=data_lane1,
    lane2=data_lane2,
    lane3=data_lane3,
)

# Collect lanes under common parameter to be more adaptable if lanes are chaning
data = {"lane_planner": {"trajectories": lanes}}

file_name = "trajectory_points.yaml"
package_path = RosPack().get_path("lane_planner_environment")
file_path = os.path.join(package_path, "config", file_name)


with open(file_path, "w") as f:
    yaml.dump(data, f, default_flow_style=False)

if args.plot:

    # include plotting library
    import matplotlib.pyplot as plt

    # prepare plot
    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.set_title("Lane Planner Trajectories")

    # create scatter plot
    scat_lane_0 = ax.scatter(*zip(*data_lane0), c="b")
    scat_lane_1 = ax.scatter(*zip(*data_lane1), c="g")
    scat_lane_2 = ax.scatter(*zip(*data_lane2), c="y")
    scat_lane_3 = ax.scatter(*zip(*data_lane3), c="r")

    # plot legend
    plt.legend(
        (scat_lane_0, scat_lane_1, scat_lane_2, scat_lane_3),
        ("lane_0", "lane_1", "lane_2", "lane_3"),
    )

    # equalize
    plt.gca().set_aspect("equal", adjustable="box")

    # show plot
    plt.show()
