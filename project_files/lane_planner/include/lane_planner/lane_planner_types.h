#ifndef __LANE_PLANNER_LANE_PLANNER_TYPES_H__
#define __LANE_PLANNER_LANE_PLANNER_TYPES_H__

namespace lane_planner {

/** \addtogroup PlannerTypes
 *  @{
 *
 * @author Johann Erhard
 *
 * @brief Collection of types needed for the global lane planner plugin
 *
 */

/**
 * @brief Lane global planner satate machine
 *
 */
enum class PlannerState { Idle, UpdatePlan, Wait };

/**
 * @brief Drive state: result from maneuver controller
 *
 */
enum class DriveMode { Left, Right, Keep };

/**
 * @brief Interpolation method for trajectory point smoothing
 *
 */
enum class Interpolation {
    LinearInterpolation = 0,
    CatmullInterpolation = 1,
    HermiteInterpolation = 2
};

/**
 * @brief Method for finidng closest node on trajectory
 *
 */
enum class FindNodeMethod { LinearSearch = 0, LinearApproxSearch = 1 };

/**
 * @brief Trajectory indicator as input for planManeuver
 *
 */
struct Trajectory {
    unsigned int id;          //!< Trajectory id in trajectories array
    unsigned int start_node;  //!< Trajectory start_node
    bool configured;          //!< Trajectory is valid and active
};

/** @}*/

}  // namespace lane_planner

#endif