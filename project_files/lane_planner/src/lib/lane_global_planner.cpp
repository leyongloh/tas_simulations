#include "lane_planner/lane_global_planner.h"

#include <ros/console.h>  // ROS_INFO_STREAM
#include <ros/time.h>     //ros::Time

#include <tuple>  // std::tie

namespace lane_planner {
LaneGlobalPlanner::LaneGlobalPlanner(
    const std::string& name,
    const std::vector<std::vector<geometry_msgs::Point>>& trajectories,
    const geometry_msgs::Pose& initial_pose)
    : name_(name),
      trajectories_(trajectories),
      last_pose_(initial_pose),
      state_(PlannerState::Idle),
      trajectory_{0, 0, false},
      trigger_update_(false) {
    goal_stamp_ = ros::Time::now().toSec();
}

void LaneGlobalPlanner::updateParameters(
    const bool debug, const int decision_mode, const double lane_width,
    const bool trigger_left, const bool trigger_right,
    const int interpolation_steps, const double maneuver_timeout,
    const double no_maneuver_timeout, const int interpolation_method,
    const int find_node_method, const bool maneuver_timeout_trigger_update) {
    debug_ = debug;
    decision_mode_ = decision_mode;
    lane_width_ = lane_width;
    interpolation_steps_ = interpolation_steps;
    maneuver_timeout_ = maneuver_timeout;
    no_maneuver_timeout_ = no_maneuver_timeout;
    interpolation_method_ =
        static_cast<enum Interpolation>(interpolation_method);
    find_node_method_ = static_cast<enum FindNodeMethod>(find_node_method);
    maneuver_timeout_trigger_update_ = maneuver_timeout_trigger_update;

    if (!maneuver_timeout_trigger_update_) {
        trigger_update_ = false;
    }

    if (!trigger_left_ && trigger_left) {
        state_ = PlannerState::UpdatePlan;
        trigger_left_ = true;
    } else if (!trigger_right_ && trigger_right) {
        state_ = PlannerState::UpdatePlan;
        trigger_right_ = true;
        return;
    }
}

void LaneGlobalPlanner::updateLastPose(const geometry_msgs::Pose& last_pose) {
    last_pose_ = last_pose;
}

void LaneGlobalPlanner::updateLastScan(const Car_scan& last_scan) {
    // Overwrite current scan if it has changed
    if (last_scan_.zones != last_scan.zones && state_ != PlannerState::Wait &&
        trajectory_.configured) {
        state_ = PlannerState::UpdatePlan;
        last_scan_ = last_scan;
    }

    if (last_scan_.zones != last_scan.zones && trigger_update_) {
        last_scan_ = last_scan;
    }
}

auto LaneGlobalPlanner::updatePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>* plan) -> bool {
    // Find neighbor trajectories_
    Trajectory left_trajectory, right_trajectory;

    std::tie(left_trajectory, right_trajectory) = findNeighborTrajectories(
        trajectories_, trajectory_.id, last_pose_, 2.0 * lane_width_);

#ifdef PRINT_DEBUG
    ROS_INFO_STREAM(name_ << ": Update Plan: Last pose: " << last_pose_);
    ROS_INFO_STREAM(name_ << ": Update Plan: Trajectory configured: "
                          << std::boolalpha << trajectory_.configured
                          << " id: " << trajectory_.id);
    ROS_INFO_STREAM(name_ << ": Update Plan: Left configured: "
                          << std::boolalpha << left_trajectory.configured
                          << " id: " << left_trajectory.id);
    ROS_INFO_STREAM(name_ << ": Update Plan: Right configured: "
                          << std::boolalpha << right_trajectory.configured
                          << " id: " << right_trajectory.id);
#endif

    DriveMode maneuver = DriveMode::Keep;

    // Make maneuver decision
    if (!debug_) {
        maneuver = planManeuver(decision_mode_, last_scan_, trajectory_,
                                left_trajectory, right_trajectory);
    } else {
        if (trigger_left_) {
            trigger_left_ = false;

            if (left_trajectory.configured) {
                maneuver = DriveMode::Left;
            }
        } else if (trigger_right_) {
            trigger_right_ = false;

            if (right_trajectory.configured) {
                maneuver = DriveMode::Right;
            }
        }
    }

    // Update Trajectory based on maneuver decision
    if (maneuver == DriveMode::Left) {
        trajectory_ = left_trajectory;
    }

    if (maneuver == DriveMode::Right) {
        trajectory_ = right_trajectory;
    }

    // Prepare updated plan
    if (maneuver != DriveMode::Keep && trajectory_.configured) {
        unsigned int goal_node;
        double goal_distance;

        // Calculate goal node for optimal trajectory
        std::tie(goal_node, goal_distance) =
            findClosestNode(trajectories_.at(trajectory_.id),
                            goal.pose.position, find_node_method_);

        // Create plan based on target trajectory in optimal direction
        preparePlan(trajectory_.start_node, goal_node,
                    trajectories_.at(trajectory_.id), plan,
                    interpolation_steps_, true, interpolation_method_);

        return true;
    }

    return false;
}

auto LaneGlobalPlanner::createPlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>* plan) -> bool {
    // Calculate start node and optimal trajectory
    if (!trajectory_.configured) {
        trajectory_ = findOptimalTrajectory(trajectories_, start.pose.position,
                                            find_node_method_);
    } else {
        double start_distance;

        std::tie(trajectory_.start_node, start_distance) =
            findClosestNode(trajectories_.at(trajectory_.id),
                            start.pose.position, find_node_method_);
    }

#ifdef PRINT_DEBUG
    ROS_INFO_STREAM(name_ << ": Create Plan: start: " << start.pose);
    ROS_INFO_STREAM(name_ << ": Create Plan: goal: " << goal.pose);
    ROS_INFO_STREAM(name_ << ": Create Plan: trajectory configured: "
                          << std::boolalpha << trajectory_.configured
                          << " id: " << trajectory_.id
                          << " start_node: " << trajectory_.start_node);
#endif

    if (!trajectory_.configured) {
        return false;
    }

    // Calculate goal node for optimal trajectory
    unsigned int goal_node;
    double goal_distance;

    std::tie(goal_node, goal_distance) =
        findClosestNode(trajectories_.at(trajectory_.id), goal.pose.position,
                        find_node_method_);

    // Create plan based on target trajectory in default direction
    preparePlan(trajectory_.start_node, goal_node,
                trajectories_.at(trajectory_.id), plan, interpolation_steps_,
                false, interpolation_method_);

    return true;
}

void LaneGlobalPlanner::resetState() {
    if (state_ == PlannerState::Wait && !trigger_update_) {
        state_ = PlannerState::Idle;
    }

    if (state_ == PlannerState::Wait && trigger_update_) {
        state_ = PlannerState::UpdatePlan;
        trigger_update_ = false;
    }
}

void LaneGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>* plan) {
    double goal_stamp = goal.header.stamp.toSec();

    if (abs(goal_stamp - goal_stamp_) > 0.5) {
#ifdef PRINT_DEBUG
        ROS_INFO_STREAM(name_ << ": makePlan: New plan");
#endif

        // Update last goal stamp
        goal_stamp_ = goal_stamp;

        createPlan(start, goal, plan);
    }

    if (state_ == PlannerState::UpdatePlan) {
#ifdef PRINT_DEBUG
        ROS_INFO_STREAM(name_ << ": makePlan: Update plan");
#endif

        if (updatePlan(start, goal, plan)) {
            timer_callback_(maneuver_timeout_);

            // Trigger update if maneuver was updated
            if (maneuver_timeout_trigger_update_) trigger_update_ = true;
        } else {
            timer_callback_(no_maneuver_timeout_);
        }

        state_ = PlannerState::Wait;
    }
}
}  // namespace lane_planner
