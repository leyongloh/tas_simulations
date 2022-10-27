#include "lane_planner/trajectory_helper.h"

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>  // std::min_element, std::min
#include <cmath>      // std::hypot, std::floor, std::ceil, std::atan2, std::abs
#include <iterator>   // std::next, std::prev, std::distance
#include <limits>     // std::numeric_limits
#include <tuple>      // std::tie
#include <vector>     // std::vector

namespace lane_planner {

auto findNClosestNodesLinear(
    const std::vector<geometry_msgs::Point>& trajectory,
    const geometry_msgs::Point& target, const unsigned int n)
    -> std::vector<unsigned int> {
    std::vector<unsigned int> closest_nodes(n, 0);
    std::vector<double> closest_distances(
        n, std::numeric_limits<double>::infinity());

    for (unsigned int i = 0; i < trajectory.size(); ++i) {
        const auto distance_i = std::hypot(target.x - trajectory.at(i).x,
                                           target.y - trajectory.at(i).y);

        for (auto j = 0; j < n; ++j) {
            if (distance_i < closest_distances.at(j)) {
                closest_distances.at(j) = distance_i;
                closest_nodes.at(j) = i;
                break;
            }
        }
    }

    return closest_nodes;
}

auto findClosestNode(const std::vector<geometry_msgs::Point>& trajectory,
                     const geometry_msgs::Point& target,
                     enum FindNodeMethod method)
    -> std::pair<unsigned int, double> {
    if (method == FindNodeMethod::LinearSearch) {
        return findClosestNodeLinear(trajectory, target);
    }

    return findClosestNodeApprox(trajectory, target);
}

auto findClosestNodeLinear(const std::vector<geometry_msgs::Point>& trajectory,
                           const geometry_msgs::Point& target)
    -> std::pair<unsigned int, double> {
    double distance = std::numeric_limits<double>::infinity();
    unsigned int point_id = 0;

    for (unsigned int i = 0; i < trajectory.size(); ++i) {
        const auto distance_i = std::hypot(target.x - trajectory.at(i).x,
                                           target.y - trajectory.at(i).y);

        if (distance_i < distance) {
            distance = distance_i;
            point_id = i;
        }
    }

    return {point_id, distance};
}

auto findClosestNodeApprox(const std::vector<geometry_msgs::Point>& trajectory,
                           const geometry_msgs::Point& target)
    -> std::pair<unsigned int, double> {
    // Calculate values in regular step distances
    const size_t step_count = 10;
    std::vector<double> distances;

    for (unsigned int i = 0; i < trajectory.size(); i += step_count) {
        const auto distance_i = std::hypot(target.x - trajectory.at(i).x,
                                           target.y - trajectory.at(i).y);

        distances.push_back(distance_i);
    }

    // Get minimum approximation
    const auto min_distance =
        std::min_element(distances.begin(), distances.end());

    const auto min_distance_i =
        static_cast<size_t>(std::distance(distances.begin(), min_distance)) *
        step_count;

    // Calculate left and right distance from approximation
    const auto left_distance =
        std::hypot(target.x - trajectory.at(min_distance_i - 1).x,
                   target.y - trajectory.at(min_distance_i - 1).y);

    const auto right_distance =
        std::hypot(target.x - trajectory.at(min_distance_i + 1).x,
                   target.y - trajectory.at(min_distance_i + 1).y);

    // Linear search for global minimum starting from approximation
    int point_id = min_distance_i;
    double distance = *min_distance;

    const auto loop_size =
        right_distance < left_distance
            ? std::min(trajectory.size() - min_distance_i + 1, step_count)
            : std::min(min_distance_i, step_count);

    for (int i = 1; i <= loop_size; ++i) {
        // Test point i
        int point_i = static_cast<int>(min_distance_i);
        right_distance < left_distance ? point_i += i : point_i -= i;

        const auto distance_i = std::hypot(target.x - trajectory.at(point_i).x,
                                           target.y - trajectory.at(point_i).y);

        if (distance_i < distance) {
            distance = distance_i;
            point_id = point_i;
        }
    }

    return {point_id, distance};
}

auto findOptimalTrajectory(
    const std::vector<std::vector<geometry_msgs::Point>>& trajectories,
    const geometry_msgs::Point& target, const enum FindNodeMethod method)
    -> Trajectory {
    // Result values
    Trajectory result = {0, 0, false};

    // Result target
    double min_distance = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < trajectories.size(); ++i) {
        // Get closest node to target in trajectory i
        int point_id;
        double distance;

        std::tie(point_id, distance) =
            findClosestNode(trajectories.at(i), target, method);

        // Update result if distance is shorter than before
        if (distance < min_distance) {
            result.configured = true;
            result.start_node = point_id;
            result.id = i;
            min_distance = distance;
        }
    }

    return result;
}

auto normalizeAngle(double angle) -> double {
    if (angle > M_PI) {
        return angle - 2 * M_PI;
    } else if (angle <= -M_PI) {
        return angle + 2 * M_PI;
    }

    return angle;
}

auto findNeighborTrajectories(
    const std::vector<std::vector<geometry_msgs::Point>>& trajectories,
    const unsigned int trajectory_id, const geometry_msgs::Pose& pose,
    const double max_distance) -> std::pair<Trajectory, Trajectory> {
    // Define results
    Trajectory result_left = {0, 0, false};
    Trajectory result_right = {0, 0, false};

    // Absolute robot angle
    const auto pose_yaw = tf::getYaw(pose.orientation);

    // Pre-compute perpendicular reference line
    const auto x2 = pose.position.x + cos(pose_yaw + 1.5708);
    const auto y2 = pose.position.y + sin(pose_yaw + 1.5708);

    const auto dx1 = pose.position.x - x2;
    const auto dy1 = pose.position.y - y2;

    const auto dm1 = pose.position.x * y2 - pose.position.y * x2;

    // Define dynamic loop values
    double distance;
    double min_distance_right = 0.0;
    double min_distance_left = 0.0;

    for (size_t i = 0; i < trajectories.size(); ++i) {
        if (i != trajectory_id) {
            // Retrieve closest two points to target on trajectory i
            const auto closest_nodes =
                findNClosestNodesLinear(trajectories.at(i), pose.position, 2);

            // Compute line between two closest points on trajectory i
            const auto x3 = trajectories.at(i).at(closest_nodes.at(0)).x;
            const auto y3 = trajectories.at(i).at(closest_nodes.at(0)).y;

            const auto x4 = trajectories.at(i).at(closest_nodes.at(1)).x;
            const auto y4 = trajectories.at(i).at(closest_nodes.at(1)).y;

            const auto dx2 = x3 - x4;
            const auto dy2 = y3 - y4;

            const auto dm2 = x3 * y4 - y3 * x4;
            const auto D = dx1 * dy2 - dy1 * dx2;

            // Check for errors in upcoming computation
            if (D == 0) {
                continue;
            }

            // Compute intersection between perpendicular reference line and
            // trajectory line
            const auto px = (dm1 * dx2 - dm2 * dx1) / D;
            const auto py = (dm1 * dy2 - dm2 * dy1) / D;

            // Distance from intersection to robot
            distance = std::hypot(px - pose.position.x, py - pose.position.y);

            // Check if intersection can be from a neighbor trajectory
            if (distance > max_distance) {
                continue;
            }

            // Angle between intersection and robot direction
            double diff_angle =
                atan2(py - pose.position.y, px - pose.position.x) - pose_yaw;

            // Determine if intersection is on the right/left side of the robot
            const auto is_right = normalizeAngle(diff_angle) < 0;

            // Check if result on specific side is better than before
            if (is_right &&
                (!result_right.configured || distance < min_distance_right)) {
                result_right.configured = true;
                result_right.id = i;
                result_right.start_node = closest_nodes.at(0);
                min_distance_right = distance;
            }

            if (!is_right &&
                (!result_left.configured || distance < min_distance_left)) {
                result_left.configured = true;
                result_left.id = i;
                result_left.start_node = closest_nodes.at(0);
                min_distance_left = distance;
            }
        }
    }

    return {result_left, result_right};
}

void linearInterpolation(geometry_msgs::Point* position,
                         const geometry_msgs::Point& p1,
                         const geometry_msgs::Point& p2, const double alpha) {
    position->x = p1.x + alpha * (p2.x - p1.x);
    position->y = p1.y + alpha * (p2.y - p1.y);
}

void catmullInterpolation(geometry_msgs::Point* position,
                          const geometry_msgs::Point& p0,
                          const geometry_msgs::Point& p1,
                          const geometry_msgs::Point& p2,
                          const geometry_msgs::Point& p3, const double alpha) {
    const auto alpha2 = alpha * alpha;

    const auto a0x = -0.5 * p0.x + 1.5 * p1.x - 1.5 * p2.x + 0.5 * p3.x;
    const auto a1x = p0.x - 2.5 * p1.x + 2.0 * p2.x - 0.5 * p3.x;
    const auto a2x = -0.5 * p0.x + 0.5 * p2.x;
    const auto a3x = p1.x;

    const auto a0y = -0.5 * p0.y + 1.5 * p1.y - 1.5 * p2.y + 0.5 * p3.y;
    const auto a1y = p0.y - 2.5 * p1.y + 2.0 * p2.y - 0.5 * p3.y;
    const auto a2y = -0.5 * p0.y + 0.5 * p2.y;
    const auto a3y = p1.y;

    position->x = a0x * alpha * alpha2 + a1x * alpha2 + a2x * alpha + a3x;
    position->y = a0y * alpha * alpha2 + a1y * alpha2 + a2y * alpha + a3y;
}

void hermiteInterpolation(geometry_msgs::Point* position,
                          const geometry_msgs::Point& p0,
                          const geometry_msgs::Point& p1,
                          const geometry_msgs::Point& p2,
                          const geometry_msgs::Point& p3, const double alpha,
                          const int bias, const double tension) {
    const auto alpha2 = alpha * alpha;
    const auto alpha3 = alpha * alpha2;

    const auto bias_pos = static_cast<double>(1 + bias) * (1.0 - tension) / 2.0;
    const auto bias_neg = static_cast<double>(1 - bias) * (1.0 - tension) / 2.0;

    const auto m0x = (p1.x - p0.x) * bias_pos + (p2.x - p1.x) * bias_neg;
    const auto m1x = (p2.x - p1.x) * bias_pos + (p3.x - p2.x) * bias_neg;

    const auto m0y = (p1.y - p0.y) * bias_pos + (p2.y - p1.y) * bias_neg;
    const auto m1y = (p2.y - p1.y) * bias_pos + (p3.y - p2.y) * bias_neg;

    const auto a0 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0;
    const auto a1 = alpha3 - 2.0 * alpha2 + alpha;
    const auto a2 = alpha3 - alpha2;
    const auto a3 = -2.0 * alpha3 + 3.0 * alpha2;

    position->x = a0 * p1.x + a1 * m0x + a2 * m1x + a3 * p2.x;
    position->y = a0 * p1.y + a1 * m0y + a2 * m1y + a3 * p2.y;
}

void computeOrientation(geometry_msgs::Quaternion* orientation,
                        const geometry_msgs::Point p0,
                        const geometry_msgs::Point p1) {
    // Compute absolute angle between last point and point
    const auto angle = atan2(p1.y - p0.y, p1.x - p0.x);

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, angle);

    tf2::convert(quat, *orientation);
}

auto updateCyclicIterator(const int iter, const bool default_dir,
                          const size_t size) -> int {
    if (default_dir) {
        return iter > size - 2 ? 0 : iter + 1;
    }

    return iter < 1 ? size - 1 : iter - 1;
}

auto checkOverflow(const int iter, const bool default_direction,
                   const size_t size, const int offset) -> int {
    int result = iter;

    bool direction = default_direction;

    if (offset < 0) {
        direction = !direction;
    }

    for (auto i = 0; i < abs(offset); ++i) {
        result = updateCyclicIterator(result, direction, size);
    }

    return result;
}

void preparePlan(const int start_node, const int goal_node,
                 const std::vector<geometry_msgs::Point>& trajectory,
                 std::vector<geometry_msgs::PoseStamped>* plan,
                 const int interpolation_steps, const bool optimal_direction,
                 const enum Interpolation interpolation_method) {
    // Prepare intermediate pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";

    // Determine loop direction: Default direction is increasing
    bool default_dir = true;

    if (optimal_direction) {
        const auto diff = static_cast<int>(start_node - goal_node);
        const auto dist_default_dir = abs(diff);
        const auto dist_counter_dir = trajectory.size() - dist_default_dir;

        default_dir = (diff > 0 && dist_default_dir > dist_counter_dir) ||
                      (diff < 0 && dist_default_dir < dist_counter_dir);
    }

    // Blending step-size for interpolation
    const auto interpolation_blending =
        1.0 / static_cast<double>(interpolation_steps);

    // Determine iterator boundaries
    const auto iter_start =
        updateCyclicIterator(start_node, default_dir, trajectory.size());

    const auto iter_end =
        updateCyclicIterator(goal_node, default_dir, trajectory.size());

    // Previous trajectory point
    geometry_msgs::Point last_point = trajectory.at(start_node);

    if (interpolation_method == Interpolation::LinearInterpolation) {
        // Loop over points
        for (auto i = iter_start; i != iter_end;
             i = updateCyclicIterator(i, default_dir, trajectory.size())) {
            // Current trajectory point
            const auto point = trajectory.at(i);

            // Static orientation for all interpolated points
            computeOrientation(&pose.pose.orientation, last_point, point);

            // Linear position interpolation
            for (auto j = 0; j < interpolation_steps; ++j) {
                ++pose.header.seq;
                linearInterpolation(&pose.pose.position, last_point, point,
                                    j * interpolation_blending);

                plan->push_back(pose);
            }

            // Update last point
            last_point = pose.pose.position;
        }
    }

    if (interpolation_method != Interpolation::LinearInterpolation) {
        // Loop over points
        for (auto i = iter_start; i != iter_end;
             i = updateCyclicIterator(i, default_dir, trajectory.size())) {
            // Points for interpolation
            const auto s0 =
                checkOverflow(i, default_dir, trajectory.size(), -2);
            const auto s1 =
                checkOverflow(i, default_dir, trajectory.size(), -1);
            const auto s2 = i;
            const auto s3 = checkOverflow(i, default_dir, trajectory.size(), 1);

            const auto p0 = trajectory.at(s0);
            const auto p1 = trajectory.at(s1);
            const auto p2 = trajectory.at(s2);
            const auto p3 = trajectory.at(s3);

            // Spline position interpolation
            for (auto j = 0; j < interpolation_steps; ++j) {
                ++pose.header.seq;

                if (interpolation_method ==
                    Interpolation::CatmullInterpolation) {
                    catmullInterpolation(&pose.pose.position, p0, p1, p2, p3,
                                         j * interpolation_blending);
                } else {
                    hermiteInterpolation(&pose.pose.position, p0, p1, p2, p3,
                                         j * interpolation_blending, 0, 0.0);
                }

                // Dynamic orientation for all interpolated points
                computeOrientation(&pose.pose.orientation, last_point,
                                   pose.pose.position);

                plan->push_back(pose);

                // Update last point
                last_point = pose.pose.position;
            }
        }
    }
}
}  // namespace lane_planner
