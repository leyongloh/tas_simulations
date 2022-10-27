#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <chrono>  // std::chrono
#include <ios>     // std::boolalpha
#include <tuple>   // std::tie
#include <vector>  // std::vector

#include "lane_planner/lane_planner_types.h"
#include "lane_planner/trajectory_helper.h"

/** \addtogroup TestTrajectoryEnvironment
 *  @{
 *
 * @author Johann Erhard
 *
 * @brief Collection of unit tests for trajectory helper library functions using
 * final trajectory points
 *
 */

/**
 * @brief Load trajectories from parameter server
 *
 * @param[in,out] private_nh Pointer to private Node Handle Object
 * @param[in,out] trajectories Returns list of trajectories
 */
void loadTrajectories(
    ros::NodeHandle* private_nh,
    std::vector<std::vector<geometry_msgs::Point>>* trajectories) {
    XmlRpc::XmlRpcValue trajectories_xml;

    if (private_nh->getParam("/lane_planner/trajectories", trajectories_xml)) {
        // Read trajectories_ from parameter server
        geometry_msgs::Point point;
        point.z = 0.0;

        for (const auto& trajectory_xmlrpc : trajectories_xml) {
            // continue if parameter is an array
            if (trajectory_xmlrpc.second.getType() ==
                XmlRpc::XmlRpcValue::TypeArray) {
                std::vector<geometry_msgs::Point> trajectory;

                // loop over trajectory elements
                for (size_t i = 0; i < trajectory_xmlrpc.second.size(); ++i) {
                    XmlRpc::XmlRpcValue trajectoryPoint =
                        trajectory_xmlrpc.second[i];
                    point.x = trajectoryPoint[0];
                    point.y = trajectoryPoint[1];

                    // Add point to trajectory
                    trajectory.push_back(point);
                }

                // Add trajectory to trajectory list
                trajectories->push_back(trajectory);
            }
        }
    }
}

/**
 * @brief Construct a new TEST object: Test loading given number of trajectories
 * from parameter server
 *
 */
TEST(TrajectoryEnvironment, TestLoadTrajectories) {
    const std::string name = "TestLoadTrajectories";
    ROS_WARN_STREAM("TrajectoryEnvironment: " << name);

    ros::NodeHandle private_nh("~");

    // Load trajectories
    std::vector<std::vector<geometry_msgs::Point>> trajectories;
    loadTrajectories(&private_nh, &trajectories);

    // Other parameters
    int num_trajectories;
    private_nh.param<int>("/TrajectoryEnvironmentTest/num_trajectories",
                          num_trajectories, 0);

    // Evaluate result
    EXPECT_EQ(trajectories.size(), num_trajectories);
}

/**
 * @brief Construct a new TEST object: Test id of closest node to target
 * location using liear and linear approximation methods and their similarity
 *
 */
TEST(TrajectoryEnvironment, TestFindClosestNode) {
    const std::string name = "TestFindClosestNode";
    ROS_WARN_STREAM("TrajectoryEnvironment: " << name);

    ros::NodeHandle private_nh("~");

    // Load trajectories
    std::vector<std::vector<geometry_msgs::Point>> trajectories;
    loadTrajectories(&private_nh, &trajectories);

    // Other parameters
    int expected_trajectory;
    double max_distance;

    geometry_msgs::Point target_point;
    target_point.z = 0.0;

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_x",
                             target_point.x, 0.0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_y",
                             target_point.y, 0.0);

    private_nh.param<int>("/TrajectoryEnvironmentTest/expected_trajectory",
                          expected_trajectory, 0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/max_distance",
                             max_distance, 0.0);

    const auto trajectory = trajectories.at(expected_trajectory);

    // Call function
    unsigned int point_id_linear;
    unsigned int distance_linear;

    std::tie(point_id_linear, distance_linear) = lane_planner::findClosestNode(
        trajectory, target_point, lane_planner::FindNodeMethod::LinearSearch);

    ROS_WARN_STREAM(name << ": point_id_linear: " << point_id_linear);
    ROS_WARN_STREAM(name << ": distance_linear: " << distance_linear);
    ROS_WARN_STREAM(name << ": point_linear [x, y]: "
                         << trajectory.at(point_id_linear).x << ", "
                         << trajectory.at(point_id_linear).y);

    unsigned int point_id_approx;
    unsigned int distance_approx;

    std::tie(point_id_approx, distance_approx) = lane_planner::findClosestNode(
        trajectory, target_point,
        lane_planner::FindNodeMethod::LinearApproxSearch);

    ROS_WARN_STREAM(name << ": point_id_approx: " << point_id_approx);
    ROS_WARN_STREAM(name << ": distance_approx: " << distance_approx);
    ROS_WARN_STREAM(name << ": point_approx [x, y]: "
                         << trajectory.at(point_id_approx).x << ", "
                         << trajectory.at(point_id_approx).y);

    // Evaluate result
    EXPECT_EQ(point_id_linear, point_id_approx);
    EXPECT_LT(distance_linear, max_distance);
    EXPECT_LT(distance_linear, max_distance);
}

/**
 * @brief Construct a new TEST object: Test runtime of linear and lienar
 * approximation search methods
 *
 */
TEST(TrajectoryEnvironment, TestFindClosestNodeTime) {
    const std::string name = "TestFindClosestNodeTime";
    ROS_WARN_STREAM("TrajectoryEnvironment: " << name);

    ros::NodeHandle private_nh("~");

    // Load trajectories
    std::vector<std::vector<geometry_msgs::Point>> trajectories;
    loadTrajectories(&private_nh, &trajectories);

    // Other parameters
    int expected_trajectory;
    double max_distance;
    int iterations;

    geometry_msgs::Point target_point;
    target_point.z = 0.0;

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_x",
                             target_point.x, 0.0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_y",
                             target_point.y, 0.0);

    private_nh.param<int>("/TrajectoryEnvironmentTest/expected_trajectory",
                          expected_trajectory, 0);

    private_nh.param<int>("/TrajectoryEnvironmentTest/iterations", iterations,
                          0);

    const auto trajectory = trajectories.at(expected_trajectory);

    // Measure execution time: Linear
    unsigned int point_id_1;
    unsigned int distance_1;

    float time_linear = 0.0;

    for (size_t i = 0; i < iterations; ++i) {
        const auto t0 = std::chrono::high_resolution_clock::now();

        std::tie(point_id_1, distance_1) =
            lane_planner::findClosestNodeLinear(trajectory, target_point);

        const auto t1 = std::chrono::high_resolution_clock::now();
        time_linear +=
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
                .count();
    }

    float time_linear_mean = time_linear / static_cast<float>(iterations);

    // Measure execution time: Approximation
    unsigned int point_id_2;
    unsigned int distance_2;

    float time_approx = 0.0;

    for (size_t i = 0; i < iterations; ++i) {
        const auto t0 = std::chrono::high_resolution_clock::now();

        std::tie(point_id_2, distance_2) =
            lane_planner::findClosestNodeApprox(trajectory, target_point);

        const auto t1 = std::chrono::high_resolution_clock::now();
        time_approx +=
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
                .count();
    }

    float time_approx_mean = time_approx / static_cast<float>(iterations);

    ROS_WARN_STREAM(name << ": time_linear_mean: " << time_linear_mean
                         << " [us]");
    ROS_WARN_STREAM(name << ": time_approx_mean: " << time_approx_mean
                         << " [us]");

    // Evaluate result
    EXPECT_LT(time_approx, time_linear);
    EXPECT_EQ(point_id_1, point_id_2);
}

/**
 * @brief Construct a new TEST object: Test id of optimal trajectory using
 * linear method
 *
 */
TEST(TrajectoryEnvironment, TestFindOptimalTrajectory) {
    const std::string name = "TestFindOptimalTrajectory";
    ROS_WARN_STREAM("TrajectoryEnvironment: " << name);

    ros::NodeHandle private_nh("~");

    // Load trajectories
    std::vector<std::vector<geometry_msgs::Point>> trajectories;
    loadTrajectories(&private_nh, &trajectories);

    // Other parameters
    int expected_trajectory;

    geometry_msgs::Point target_point;
    target_point.z = 0.0;

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_x",
                             target_point.x, 0.0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_y",
                             target_point.y, 0.0);

    private_nh.param<int>("/TrajectoryEnvironmentTest/expected_trajectory",
                          expected_trajectory, 0);

    // Call function
    const auto trajectory = lane_planner::findOptimalTrajectory(
        trajectories, target_point, lane_planner::FindNodeMethod::LinearSearch);

    const auto point = trajectories.at(trajectory.id).at(trajectory.start_node);

    ROS_WARN_STREAM(name << ": target_point: x: " << target_point.x
                         << " y: " << target_point.y);
    ROS_WARN_STREAM(name << ": trajectory_id: " << trajectory.id);
    ROS_WARN_STREAM(name << ": point_id: " << trajectory.start_node);
    ROS_WARN_STREAM(name << ": point: x: " << point.x << " y: " << point.y);

    // Evaluate result
    EXPECT_EQ(trajectory.id, expected_trajectory);
}

/**
 * @brief Construct a new TEST object: Test id of neighbor trajectories based on
 * target location
 *
 */
TEST(TrajectoryEnvironment, TestFindNeighborTrajectories) {
    const std::string name = "TestFindNeighborTrajectories";
    ROS_WARN_STREAM("TrajectoryEnvironment: " << name);

    ros::NodeHandle private_nh("~");

    // Load trajectories
    std::vector<std::vector<geometry_msgs::Point>> trajectories;
    loadTrajectories(&private_nh, &trajectories);

    // Other parameters
    int expected_trajectory;

    geometry_msgs::Pose target_pose;
    target_pose.position.z = 0.0;

    private_nh.param<int>("/TrajectoryEnvironmentTest/expected_trajectory",
                          expected_trajectory, 0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_x",
                             target_pose.position.x, 0.0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_y",
                             target_pose.position.y, 0.0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_orientation_x",
                             target_pose.orientation.x, 0.0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_orientation_y",
                             target_pose.orientation.y, 0.0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_orientation_z",
                             target_pose.orientation.z, 0.0);

    private_nh.param<double>("/TrajectoryEnvironmentTest/target_orientation_w",
                             target_pose.orientation.w, 0.0);

    // Call function
    lane_planner::Trajectory left, right;
    std::tie(left, right) = lane_planner::findNeighborTrajectories(
        trajectories, expected_trajectory, target_pose, 5.0);

    ROS_WARN_STREAM(
        name << ": left: " << left.id << " node: " << left.start_node
             << " values: " << trajectories.at(left.id).at(left.start_node).x
             << ", " << trajectories.at(left.id).at(left.start_node).y
             << " configured: " << std::boolalpha << left.configured);
    ROS_WARN_STREAM(
        name << ": right: " << right.id << " node: " << right.start_node
             << " values: " << trajectories.at(right.id).at(right.start_node).x
             << ", " << trajectories.at(right.id).at(right.start_node).y
             << " configured: " << std::boolalpha << right.configured);

    EXPECT_EQ(left.configured, true);
    EXPECT_EQ(right.configured, true);
}

/** @}*/

auto main(int argc, char** argv) -> int {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_environment_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
