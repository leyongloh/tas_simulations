#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <tuple>   // std::tie
#include <vector>  // std::vector

#include "lane_planner/lane_planner_types.h"
#include "lane_planner/trajectory_helper.h"

/** \addtogroup TestTrajectoryEnvironment
 *  @{
 *
 * @author Johann Erhard
 *
 * @brief Collection of tests for trajectory helper library functions using fake
 * trajectory points
 *
 */

/**
 * @brief Add point to test trajectory
 *
 * @param trajectory Trajectory pointer
 * @param x Point x value to add
 * @param y Point y value to add
 * @param z Point z value to add
 */
void addPoint(std::vector<geometry_msgs::Point> *trajectory, const double x,
              const double y, const double z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;

    trajectory->push_back(p);
}

/**
 * @brief Construct a new TEST object: Test id of closest node to target point
 * using linear method
 *
 */
TEST(TrajectoryHelper, TestFindClosestNode) {
    ROS_WARN_STREAM("TrajectoryHelper: TestFindClosestNode");

    // Prepare test trajectory
    std::vector<geometry_msgs::Point> trajectory;

    addPoint(&trajectory, -3.0, 3.0, 0.0);
    addPoint(&trajectory, -2.0, 2.0, 0.0);
    addPoint(&trajectory, -1.0, 1.0, 0.0);
    addPoint(&trajectory, 0.0, 0.0, 0.0);
    addPoint(&trajectory, 1.0, -1.0, 0.0);
    addPoint(&trajectory, 2.0, -2.0, 0.0);
    addPoint(&trajectory, 3.0, -3.0, 0.0);

    // Prepare target point
    geometry_msgs::Point target_point;
    target_point.x = 1.0;
    target_point.y = 1.0;
    target_point.z = 0.0;

    // Call function
    int point_id;
    double distance;

    std::tie(point_id, distance) =
        lane_planner::findClosestNodeLinear(trajectory, target_point);

    // Evaluate result
    EXPECT_FLOAT_EQ(distance, sqrt(2));
    EXPECT_EQ(point_id, 3);
}

/**
 * @brief Construct a new TEST object: Test length and integrity of all
 * interpolation methods
 *
 */
TEST(TrajectoryHelper, TestPreparePlan) {
    ROS_WARN_STREAM("TrajectoryHelper: TestPreparePlan");

    // Prepare test trajectory
    std::vector<geometry_msgs::Point> trajectory;

    addPoint(&trajectory, 47.5, 23.59375, 0.0);
    addPoint(&trajectory, 52.5, 23.59375, 0.0);
    addPoint(&trajectory, 57.5, 23.59375, 0.0);
    addPoint(&trajectory, 62.5, 23.59375, 0.0);
    addPoint(&trajectory, 69.45604481276341, 22.54503226046322, 0.0);
    addPoint(&trajectory, 74.04250310326388, 20.577552361095904, 0.0);
    addPoint(&trajectory, 78.11252112917262, 17.689381647011817, 0.0);
    addPoint(&trajectory, 81.48399617261387, 14.009744051219558, 0.0);
    addPoint(&trajectory, 84.00608006418088, 9.703275701305293, 0.0);

    // Call function
    const auto interpolation_steps = 20;
    const auto start_node = 1;
    const auto goal_node = 7;

    std::vector<geometry_msgs::PoseStamped> plan_linear;
    std::vector<geometry_msgs::PoseStamped> plan_hermite;
    std::vector<geometry_msgs::PoseStamped> plan_catmull;

    lane_planner::preparePlan(start_node, goal_node, trajectory, &plan_linear,
                              interpolation_steps, false,
                              lane_planner::Interpolation::LinearInterpolation);

    lane_planner::preparePlan(
        start_node, goal_node, trajectory, &plan_hermite, interpolation_steps,
        false, lane_planner::Interpolation::HermiteInterpolation);

    lane_planner::preparePlan(
        start_node, goal_node, trajectory, &plan_catmull, interpolation_steps,
        false, lane_planner::Interpolation::CatmullInterpolation);

    ROS_WARN_STREAM("TrajectoryHelper: plan_sizes: linear: "
                    << plan_linear.size()
                    << ", catmull: " << plan_catmull.size()
                    << ", hermite: " << plan_hermite.size());

    // Evaluate result
    const auto expected_points = interpolation_steps * (goal_node - start_node);
    EXPECT_EQ(plan_linear.size(), expected_points);
    EXPECT_EQ(plan_catmull.size(), expected_points);
    EXPECT_EQ(plan_hermite.size(), expected_points);
}

/**
 * @brief Construct a new TEST object: Test id of optimal trajectory using
 * linear method
 *
 */
TEST(TrajectoryHelper, TestFindOptimalTrajectory) {
    ROS_WARN_STREAM("TrajectoryHelper: TestFindOptimalTrajectory");

    // Prepare test trajectories
    std::vector<geometry_msgs::Point> trajectory1;
    std::vector<geometry_msgs::Point> trajectory2;

    addPoint(&trajectory1, -3.0, 1.0, 0.0);
    addPoint(&trajectory1, -2.0, 1.0, 0.0);
    addPoint(&trajectory1, -1.0, 1.0, 0.0);
    addPoint(&trajectory1, 0.0, 1.0, 0.0);
    addPoint(&trajectory1, 1.0, 1.0, 0.0);
    addPoint(&trajectory1, 2.0, 1.0, 0.0);
    addPoint(&trajectory1, 3.0, 1.0, 0.0);

    addPoint(&trajectory2, -3.0, 2.0, 0.0);
    addPoint(&trajectory2, -2.0, 2.0, 0.0);
    addPoint(&trajectory2, -1.0, 2.0, 0.0);
    addPoint(&trajectory2, 0.0, 2.0, 0.0);
    addPoint(&trajectory2, 1.0, 2.0, 0.0);
    addPoint(&trajectory2, 2.0, 2.0, 0.0);
    addPoint(&trajectory2, 3.0, 2.0, 0.0);

    std::vector<std::vector<geometry_msgs::Point>> trajectories;
    trajectories.push_back(trajectory1);
    trajectories.push_back(trajectory2);

    // Prepare target point
    geometry_msgs::Point target_point;
    target_point.x = 2.0;
    target_point.y = 5.0;
    target_point.z = 0.0;

    // Call function
    const auto trajectory = lane_planner::findOptimalTrajectory(
        trajectories, target_point, lane_planner::FindNodeMethod::LinearSearch);

    // Evaluate result
    EXPECT_EQ(trajectory.id, 1);
    EXPECT_EQ(trajectory.start_node, 5);
}

/**
 * @brief Construct a new TEST object: Test ids of neighbor trajectories using
 * different orientations
 *
 */
TEST(TrajectoryHelper, TestFindNeighborTrajectories) {
    ROS_WARN_STREAM("TrajectoryHelper: TestFindNeighborTrajectories");

    // Prepare test trajectories
    std::vector<geometry_msgs::Point> trajectory1;
    std::vector<geometry_msgs::Point> trajectory2;
    std::vector<geometry_msgs::Point> trajectory3;

    addPoint(&trajectory1, -3.0, 1.0, 0.0);
    addPoint(&trajectory1, -2.0, 1.0, 0.0);
    addPoint(&trajectory1, -1.0, 1.0, 0.0);
    addPoint(&trajectory1, 0.0, 1.0, 0.0);
    addPoint(&trajectory1, 1.0, 1.0, 0.0);
    addPoint(&trajectory1, 2.0, 1.0, 0.0);
    addPoint(&trajectory1, 3.0, 1.0, 0.0);

    addPoint(&trajectory2, -3.0, 2.0, 0.0);
    addPoint(&trajectory2, -2.0, 2.0, 0.0);
    addPoint(&trajectory2, -1.0, 2.0, 0.0);
    addPoint(&trajectory2, 0.0, 2.0, 0.0);
    addPoint(&trajectory2, 1.0, 2.0, 0.0);
    addPoint(&trajectory2, 2.0, 2.0, 0.0);
    addPoint(&trajectory2, 3.0, 2.0, 0.0);

    addPoint(&trajectory3, -3.0, 3.0, 0.0);
    addPoint(&trajectory3, -2.0, 3.0, 0.0);
    addPoint(&trajectory3, -1.0, 3.0, 0.0);
    addPoint(&trajectory3, 0.0, 3.0, 0.0);
    addPoint(&trajectory3, 1.0, 3.0, 0.0);
    addPoint(&trajectory3, 2.0, 3.0, 0.0);
    addPoint(&trajectory3, 3.0, 3.0, 0.0);

    std::vector<std::vector<geometry_msgs::Point>> trajectories;
    trajectories.push_back(trajectory1);
    trajectories.push_back(trajectory2);
    trajectories.push_back(trajectory3);

    // Function parameters
    unsigned int trajectory_id;
    double yaw_angle;
    double pose_yaw;

    lane_planner::Trajectory left_trajectory;
    lane_planner::Trajectory right_trajectory;

    geometry_msgs::Pose target_pose;

    // Test1
    trajectory_id = 1;
    yaw_angle = 0.0;

    target_pose.position.x = 1.0;
    target_pose.position.y = 2.0;
    target_pose.position.z = 0.0;
    target_pose.orientation = tf::createQuaternionMsgFromYaw(yaw_angle);

    pose_yaw = tf::getYaw(target_pose.orientation);

    std::tie(left_trajectory, right_trajectory) =
        lane_planner::findNeighborTrajectories(trajectories, trajectory_id,
                                               target_pose, 5.0);

    // Evaluate results
    EXPECT_EQ(left_trajectory.id, 2);
    EXPECT_EQ(right_trajectory.id, 0);
    EXPECT_DOUBLE_EQ(yaw_angle, pose_yaw);

    // Test1
    trajectory_id = 1;
    yaw_angle = M_PI;

    target_pose.position.x = 1.0;
    target_pose.position.y = 2.0;
    target_pose.position.z = 0.0;
    target_pose.orientation = tf::createQuaternionMsgFromYaw(yaw_angle);

    pose_yaw = tf::getYaw(target_pose.orientation);

    std::tie(left_trajectory, right_trajectory) =
        lane_planner::findNeighborTrajectories(trajectories, trajectory_id,
                                               target_pose, 5.0);

    // Evaluate results
    EXPECT_EQ(left_trajectory.id, 0);
    EXPECT_EQ(right_trajectory.id, 2);
    EXPECT_DOUBLE_EQ(yaw_angle, pose_yaw);
}

/** @}*/

auto main(int argc, char **argv) -> int {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_helper_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
