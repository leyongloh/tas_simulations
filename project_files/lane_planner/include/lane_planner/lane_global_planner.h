#ifndef __LANE_PLANNER_LANE_GLOBAL_PLANNER_H__
#define __LANE_PLANNER_LANE_GLOBAL_PLANNER_H__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <functional>  // std::function
#include <string>      // std::string
#include <vector>      // std::vector

#include "lane_planner/Car_scan.h"
#include "lane_planner/decision_helper.h"
#include "lane_planner/lane_planner_types.h"
#include "lane_planner/trajectory_helper.h"

namespace lane_planner {

/**
 * @author Johann Erhard
 *
 * @brief Lane Global Planner functionallity. This class contains all
 * library functionallity without the dependency of a ROS version.
 *
 */
class LaneGlobalPlanner {
   public:
    /**
     * @brief Construct a new Lane Global Planner object
     *
     * @param[in] name Planner name
     * @param[in] trajectories List of configured trajectories
     * @param[in] initial_pose Initial robot pose in map frame
     */
    LaneGlobalPlanner(
        const std::string& name,
        const std::vector<std::vector<geometry_msgs::Point>>& trajectories,
        const geometry_msgs::Pose& initial_pose);

    /**
     * @brief Update dynamic parameters
     *
     * @param[in] debug Debug mode
     * @param[in] decision_mode Maneuver decision mode
     * @param[in] lane_width Width of lane to drive on
     * @param[in] trigger_left Manual lane changing trigger to the left
     * @param[in] trigger_right Manual lane changing trigger to the right
     * @param[in] interpolation_steps Number of interpolation steps between two
     * trajectory points
     * @param[in] maneuver_timeout Timeout if maneuver has changed
     * @param[in] no_maneuver_timeout Timeout if maneuver has not changed
     * @param[in] interpolation_method Interpolation method betewen two
     * trajectory points
     * @param[in] find_node_method Method for finding closest node on trajectory
     * @param[in] maneuver_timeout_trigger_update Successful maneuver timeout
     * triggers another update
     */
    void updateParameters(bool debug, int decision_mode, double lane_width,
                          bool trigger_left, bool trigger_right,
                          int interpolation_steps, double maneuver_timeout,
                          double no_maneuver_timeout, int interpolation_method,
                          int find_node_method,
                          bool maneuver_timeout_trigger_update);

    /**
     * @brief Update dynamic pose feedback
     *
     * @param[in] last_pose Last registered robot pose
     */
    void updateLastPose(const geometry_msgs::Pose& last_pose);

    /**
     * @brief Update dynamic car scan
     *
     * @param[in] last_scan Last registered lidar car scan
     */
    void updateLastScan(const Car_scan& last_scan);

    /**
     * @brief Update plan whilst car is driving. Include car feedback, i.e.,
     * pose and car_scan to determine a suitable maneuver to lanes lying to the
     * cars side.
     *
     * @param[in] start Robot start position in map frame
     * @param[in] goal Robot goal position in map frame
     * @param[in,out] plan Pointer to plan to be updated
     */
    auto updatePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>* plan) -> bool;

    /**
     * @brief Create a new plan if the current trajectory is not known
     *
     * @param[in] start Robot start position in map frame
     * @param[in] goal Robot goal position in map frame
     * @param[in,out] plan Pointer to plan to be updated
     */
    auto createPlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>* plan) -> bool;

    /**
     * @brief Determine if the plan should be updated or newly created based on
     * goal and planner sate
     *
     * @param[in] start Robot start position in map frame
     * @param[in] goal Robot goal position in map frame
     * @param[in,out] plan Pointer to plan to be updated
     */
    void makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>* plan);

    /**
     * @brief Register callback function for activating the ROS Timer
     *
     * @param timer_callback Function handling the timer
     */
    void registerTimerCallback(
        std::function<void(const double)> timer_callback) {
        timer_callback_ = timer_callback;
    }

    /**
     * @brief Get the Trajectory object
     *
     * @return unsigned int Trajectory id
     */
    auto getTrajectory() -> int {
        return trajectory_.configured ? trajectory_.id : -1;
    };

    /**
     * @brief Reset State after ROS Timer has timed out
     *
     */
    void resetState();

   private:
    // General parameters
    std::string name_;

    // Dynamic Parameters
    bool debug_;                  //!< Debug mode
    bool trigger_left_;           //!< Manual lane changing trigger to the left
    bool trigger_right_;          //!< Manual lane changing trigger to the right
    int decision_mode_;           //!< Maneuver decision mode
    double lane_width_;           //!< Width of lane to drive on
    int interpolation_steps_;     //!< Number of interpolation steps between two
                                  //!< trajectory points
    double maneuver_timeout_;     //!< Timeout if maneuver has changed
    double no_maneuver_timeout_;  //!< Timeout if maneuver has not changed
    enum Interpolation interpolation_method_;  //!< Interpolation method betewen
                                               //!< two trajectory points
    enum FindNodeMethod
        find_node_method_;  //!< Method for finding closest node on trajectory
    bool maneuver_timeout_trigger_update_;  //!< Successful maneuver timeout
                                            //!< triggers another update

    // State Machine
    PlannerState state_;   //!< State for state machine
    bool trigger_update_;  //!< Trigger another Update if update was successful
    double goal_stamp_;    //!< Time of last registered goal
    std::function<void(const double)> timer_callback_;  //!< Start timer

    // Maneuver controller
    std::vector<std::vector<geometry_msgs::Point>>
        trajectories_;       //!< Configured trajectories
    Trajectory trajectory_;  //!< Current trajectory

    // Meneuver update
    Car_scan last_scan_;             //!< Last zone occupations
    geometry_msgs::Pose last_pose_;  //!< Last car position
};

}  // namespace lane_planner

#endif
