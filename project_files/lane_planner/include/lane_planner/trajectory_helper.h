#ifndef __LANE_PLANNER_TRAJECTORY_HELPER_H__
#define __LANE_PLANNER_TRAJECTORY_HELPER_H__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <utility>  // std::pair
#include <vector>   // std::vector

#include "lane_planner/lane_planner_types.h"

namespace lane_planner {

/** \addtogroup TrajectoryHelper
 *  @{
 *
 * @author Johann Erhard
 *
 * @brief Collection of custom trajectory helper functions, i.e.,
 * findClosestNodes, findOptimalTrajectory, findNeighborTrajectories,
 * and Interpolation
 *
 */

/**
 * @brief Find n closest trajectory points with respect to target location
 *
 * @param[in] trajectory Reference trajectory
 * @param[in] target Target point
 * @param[in] n Number of points to search for
 * @return std::vector<unsigned int> Returns list of point indeces on trajectory
 */
auto findNClosestNodesLinear(
    const std::vector<geometry_msgs::Point>& trajectory,
    const geometry_msgs::Point& target, unsigned int n)
    -> std::vector<unsigned int>;

/**
 * @brief Find closest trajectory point with respect to target location
 *
 * @param[in] trajectory Reference trajectory
 * @param[in] target Target point
 * @param[in] method Specify computation method
 * @return std::pair<int, double> Returns index of trajectory point and distance
 * to target
 */
auto findClosestNode(const std::vector<geometry_msgs::Point>& trajectory,
                     const geometry_msgs::Point& target,
                     enum FindNodeMethod method)
    -> std::pair<unsigned int, double>;

/**
 * @brief Find closest trajectory point with respect to target location using
 * linear computation method
 *
 * @param[in] trajectory Reference trajectory
 * @param[in] target Target point
 * @return std::pair<int, double> Returns index of trajectory point and distance
 * to target
 */
auto findClosestNodeLinear(const std::vector<geometry_msgs::Point>& trajectory,
                           const geometry_msgs::Point& target)
    -> std::pair<unsigned int, double>;

/**
 * @brief Find closest trajectory point with respect to target location using
 * linear approximation computation method
 *
 * @param[in] trajectory Reference trajectory
 * @param[in] target Target point
 * @return std::pair<int, double> Returns index of trajectory point and distance
 * to target
 */
auto findClosestNodeApprox(const std::vector<geometry_msgs::Point>& trajectory,
                           const geometry_msgs::Point& target)
    -> std::pair<unsigned int, double>;

/**
 * @brief Find optimal trajectory based on total distance from a list of given
 * trajectories
 *
 * @param[in] trajectories List of all trajectories
 * @param[in] target Target point outside of the trajectories
 * @param[in] method Method for finding closest node
 * @return Trajectory Returns optimal trajectory
 */
auto findOptimalTrajectory(
    const std::vector<std::vector<geometry_msgs::Point>>& trajectories,
    const geometry_msgs::Point& target, const enum FindNodeMethod method)
    -> Trajectory;

/**
 * @brief Normalize angle value in radians to [-pi;  pi]
 *
 * @param angle Angle in radians
 * @return double Normalized angle
 */
auto normalizeAngle(double angle) -> double;

/**
 * @brief Find neighbor trajectories on both sides based on a list of given
 * trajectories and a pose (position, orientation)
 *
 * @param[in] trajectories List of all trajectories
 * @param[in] trajectory_id Id of current trajectory from trajectories list
 * @param[in] pose Target position and orientation
 * @param[in] max_distance Maximum distance a trajectory can be from pose to be
 * registered as a neighbor
 * @return std::pair<Trajectory, Trajectory> Returns left and right trajectory
 */
auto findNeighborTrajectories(
    const std::vector<std::vector<geometry_msgs::Point>>& trajectories,
    unsigned int trajectory_id, const geometry_msgs::Pose& pose,
    double max_distance) -> std::pair<Trajectory, Trajectory>;

/**
 * @brief Linear interpolation between p1 and p2
 *
 * @param[in,out] position Resulting position
 * @param[in] p1 Point to interpolate from
 * @param[in] p2 Point to interpolate to
 * @param[in] alpha Blending value between p0 and p1
 */
void linearInterpolation(geometry_msgs::Point* position,
                         const geometry_msgs::Point& p1,
                         const geometry_msgs::Point& p2, double alpha);

/**
 * @brief Catmull interpolation between p1 and p2. Linear interpolation in x and
 * catmull interpolation in y axis.
 *
 * @param[in,out] position Resulting position
 * @param[in] p0 Point before p1 for smoother transition
 * @param[in] p1 Point to interpolate from
 * @param[in] p2 Point to interpolate to
 * @param[in] p3 Point after p2 for smoother transition
 * @param[in] alpha Blending value between p1 and p2
 */
void catmullInterpolation(geometry_msgs::Point* position,
                          const geometry_msgs::Point& p0,
                          const geometry_msgs::Point& p1,
                          const geometry_msgs::Point& p2,
                          const geometry_msgs::Point& p3, double alpha);

/**
 * @brief Hermite interpolation between p1 and p2. Linear interpolation in x and
 * hermite interpolation in y axis.
 *
 * @param[in,out] position Resulting position
 * @param[in] p0 Point before p1 for smoother transition
 * @param[in] p1 Point to interpolate from
 * @param[in] p2 Point to interpolate to
 * @param[in] p3 Point after p2 for smoother transition
 * @param[in] alpha Blending value between p1 and p2
 * @param[in] bias Bias towards a p1 or p2 [0, 1]
 * @param[in] tension Spline tension factor [-1; 1]
 */
void hermiteInterpolation(geometry_msgs::Point* position,
                          const geometry_msgs::Point& p0,
                          const geometry_msgs::Point& p1,
                          const geometry_msgs::Point& p2,
                          const geometry_msgs::Point& p3, double alpha,
                          int bias, double tension);

/**
 * @brief Compute orientation between two positions
 *
 * @param[in,out] orientation Orientation between p0 and p1
 * @param[in] p0 Current positon
 * @param[in] p1 Previous position
 */
void computeOrientation(geometry_msgs::Quaternion* orientation,
                        const geometry_msgs::Point p0,
                        const geometry_msgs::Point p1);

/**
 * @brief Update iterator for bi-directional cyclic for-loop
 *
 * @param[in] iter Last iterator value
 * @param[in] default_dir Loop direction is increasing
 * @param[in] size Number of trajectory points
 * @return int Returns new iterator value
 */
auto updateCyclicIterator(int iter, bool default_dir, size_t size) -> int;

/**
 * @brief Update iterator and correct overflow
 *
 * @param[in] iter Last iterator value
 * @param[in] default_direction Loop direction is increasing
 * @param[in] size Number of trajectory points
 * @param[in] offset Update iterator with offset value
 * @return int Returns new iterator value
 */
auto checkOverflow(const int iter, const bool default_direction,
                   const size_t size, const int offset) -> int;

/**
 * @brief Prepare a plan of target trajectory points to follow. Interpolation
 * method may be chosen between reference trajectory points.
 *
 * @param[in] start_node Index of first node in target trajectory
 * @param[in] goal_node Index of goal node in target trajectory
 * @param[in] trajectory Final trajectory to prepare plan with
 * @param[in] interpolation_steps Number of steps between two points
 * @param[in] optimal_direction Find optimal or use default loop direction
 * @param[in] interpolation_method Linear or spline interpolation
 * @param[in,out] plan Global plan to be prepared
 */
void preparePlan(int start_node, int goal_node,
                 const std::vector<geometry_msgs::Point>& trajectory,
                 std::vector<geometry_msgs::PoseStamped>* plan,
                 int interpolation_steps, bool optimal_direction,
                 enum Interpolation interpolation_method);

/** @}*/

}  // namespace lane_planner

#endif
