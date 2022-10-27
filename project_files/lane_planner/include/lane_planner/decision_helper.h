#ifndef __LANE_PLANNER_DECISION_HELPER_H__
#define __LANE_PLANNER_DECISION_HELPER_H__

#include "lane_planner/Car_scan.h"
#include "lane_planner/lane_planner_types.h"

namespace lane_planner {

/** \addtogroup DecisionHelper
 *  @{
 *
 * @author Sebastian Siegner
 *
 * @brief This group is designed to detrermine which lane the car should take
 * based on the given lidar scan.
 */

/**
 * @brief This function decides based on the data from the laser scan, the
 current, and the surrounding lanes where to go. The results are ether
 to change to the left or right or to stay there. It can also determine if the
 car should break or not. However, this is not yet implemented in the global
 planner and therefore not returned from this function.
 *
 * @param[in] decision_mode Number of backend decision function
 * @param[in] last_scan Current car laser scan
 * @param[in] current_trajectory Current trajectory
 * @param[in] left_trajectory Trajectory to the left
 * @param[in] right_trajectory Trajectory to the right
 * @return DriveMode
 */
auto planManeuver(int decision_mode, const Car_scan& last_scan,
                  const Trajectory& current_trajectory,
                  const Trajectory& left_trajectory,
                  const Trajectory& right_trajectory) -> DriveMode;

/** @}*/

}  // namespace lane_planner

#endif
