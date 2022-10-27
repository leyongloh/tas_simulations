#include "lane_planner/decision_helper.h"

#include <ros/console.h>  // ROS_INFO_STREAM

namespace lane_planner {
auto planManeuver(const int decision_mode, const Car_scan &last_scan,
                  const Trajectory &current_trajectory,
                  const Trajectory &left_trajectory,
                  const Trajectory &right_trajectory) -> DriveMode {
#ifdef PRINT_DEBUG
    ROS_INFO_STREAM("DecisionHelper: planManeuver");
    ROS_INFO_STREAM("DecisionHelper: Zones Front:  "
                    << static_cast<int>(last_scan.zones[0]) << " "
                    << static_cast<int>(last_scan.zones[1]) << " "
                    << static_cast<int>(last_scan.zones[2]));
    ROS_INFO_STREAM("DecisionHelper: Zones Middle: "
                    << static_cast<int>(last_scan.zones[3]) << "   "
                    << static_cast<int>(last_scan.zones[4]));
    ROS_INFO_STREAM("DecisionHelper: Zones Back:   "
                    << static_cast<int>(last_scan.zones[5]) << " "
                    << static_cast<int>(last_scan.zones[6]) << " "
                    << static_cast<int>(last_scan.zones[7]));
#endif

    switch (decision_mode) {
        case 0:
            // US Driving Rules
            if (right_trajectory.configured &&
                left_trajectory.configured)  // Status: Middle Lane
            {
#ifdef PRINT_DEBUG
                ROS_INFO_STREAM("DecisionHelper: Status: Middle Lane");
#endif
                if (last_scan.zones[1])  // if a car is infront
                {
                    if (!last_scan.zones[3] &&
                        !last_scan.zones[0]) {  // No car on the left
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM("DecisionHelper: Maneuver: Going Left");
#endif
                        return DriveMode::Left;
                    } else if (!last_scan.zones[4] &&
                               !last_scan.zones[2]) {  // No car on the right
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM(
                            "DecisionHelper: Maneuver: Going Right");
#endif
                        return DriveMode::Right;
                    } else if (!last_scan
                                    .zones[3]) {  // No car on the left (but
                                                  // left front) --> no overtake
// Break
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM(
                            "DecisionHelper: Maneuver: Going Left slow");
#endif
                        return DriveMode::Left;
                    } else if (!last_scan.zones[4]) {  // No car on the right
                                                       // (but right front) -->
                                                       // no overtake
// Break
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM(
                            "DecisionHelper: Maneuver: Going Right slow");
#endif
                        return DriveMode::Right;
                    } else {
                        // Break
                        return DriveMode::Keep;
                    }
                }
            } else if (!right_trajectory.configured &&
                       left_trajectory.configured)  // Status: Right Lane
            {
#ifdef PRINT_DEBUG
                ROS_INFO_STREAM("DecisionHelper: Status: Right Lane");
#endif
                if (last_scan.zones[1])  // if a car is infront
                {
                    if (!last_scan.zones[3]) {  // no car left
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM("DecisionHelper: Maneuver: Going Left");
#endif
                        return DriveMode::Left;  // move left
                    } else {
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM("DecisionHelper: Maneuver: Stay");
#endif

                        return DriveMode::Keep;
                        // Break
                    }
                } else {
                    return DriveMode::Keep;  // stay on lane
                }
            } else if (right_trajectory.configured &&
                       !left_trajectory.configured)  // Status: Left Lane
            {
#ifdef PRINT_DEBUG
                ROS_INFO_STREAM("DecisionHelper: Status: Left Lane");
#endif

                if (last_scan.zones[1])  // if a car is infront
                {
                    if (!last_scan.zones[4]) {  // no car right --> go right
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM(
                            "DecisionHelper: Maneuver: Going right");
#endif
                        if (!last_scan
                                 .zones[2]) {  // no overtake if car is right
                                               // front (only lane change)
                            // Break
                        }
                        return DriveMode::Right;
                    } else {
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM("DecisionHelper: Maneuver: Stay");
#endif
                        return DriveMode::Keep;
                        // Break
                    }
                }
            } else {  // one lane road
                if (last_scan.zones[1]) {
                    // Break
                } else {
                    // Speed up
                }
                return DriveMode::Keep;
            }
            break;
        case 1:
            // German Driving Rules (more or less)
            if (right_trajectory.configured &&
                left_trajectory.configured)  // Status: Middle Lane
            {
#ifdef PRINT_DEBUG
                ROS_INFO_STREAM("DecisionHelper: Status: Middle Lane");
#endif
                if (last_scan.zones[1])  // if a car is infront
                {
                    if (!last_scan.zones[3] &&
                        !last_scan
                             .zones[0]) {  // If left free go left and overtake
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM("DecisionHelper: Maneuver: Going Left");
#endif
                        return DriveMode::Left;
                    } else if (!last_scan.zones[4] &&
                               !last_scan.zones[2]) {  // If right free go right
                                                       // and overtake
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM(
                            "DecisionHelper: Maneuver: Going Right");
#endif
                        return DriveMode::Right;
                    } else if (!last_scan.zones[3]) {  // If left front is full
                                                       // but side is empty go
                                                       // left and dont overtake
// Break
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM(
                            "DecisionHelper: Maneuver: Going Left slow");
#endif
                        return DriveMode::Left;
                    } else {
                        // Break
                        return DriveMode::Keep;
                    }
                } else if (!last_scan.zones[2] &&
                           !last_scan.zones[4]) {  // Right Front and Right zone
                                                   // if free go right
// are free
#ifdef PRINT_DEBUG
                    ROS_INFO_STREAM("DecisionHelper: Maneuver: Going Right");
#endif

                    // Break statement
                    return DriveMode::Right;  // Rechtsfahrgebot
                }
            } else if (!right_trajectory.configured &&
                       left_trajectory.configured)  // Status: Right Lane
            {
#ifdef PRINT_DEBUG
                ROS_INFO_STREAM("DecisionHelper: Status: Right Lane");
#endif
                if (last_scan.zones[1])  // if a car is infront
                {
                    if (!last_scan.zones[3]) {
#ifdef PRINT_DEBUG
                        ROS_INFO_STREAM("DecisionHelper: Maneuver: Going Left");
#endif
                        if (last_scan.zones[0]) {  // Break
                        }

                        return DriveMode::Left;  // move left
                    } else {
                        return DriveMode::Keep;
                        // Break
                    }
                }
                return DriveMode::Keep;
            } else if (right_trajectory.configured &&
                       !left_trajectory.configured)  // Status: Left Lane
            {
#ifdef PRINT_DEBUG
                ROS_INFO_STREAM("DecisionHelper: Status: Left Lane");
#endif
                if (!last_scan.zones[2] && !last_scan.zones[4]) {
                    // Nothing in the right lane --> go right
#ifdef PRINT_DEBUG
                    ROS_INFO_STREAM("DecisionHelper: Maneuver: Going Right");
#endif
                    return DriveMode::Right;
                } else {
                    // Else: stay and maybe break
                    if (last_scan.zones[1]) {
                        // Break
                    }
                    return DriveMode::Keep;
                }
            } else {  // one lane road
                if (last_scan.zones[1]) {
                    // Break
                } else {
                    // Speed up
                }
                return DriveMode::Keep;
            }
            break;
        default:
            return DriveMode::Keep;
    }
    return DriveMode::Keep;
}
}  // namespace lane_planner