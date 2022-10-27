#ifndef __LANE_PLANNER_LANE_GLOBAL_PLANNER_ROS_H__
#define __LANE_PLANNER_LANE_GLOBAL_PLANNER_ROS_H__

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

#include <memory>  // std::unique_ptr
#include <string>  // std::string
#include <vector>  // std::vector

#include "lane_planner/Car_scan.h"
#include "lane_planner/LaneGlobalPlannerConfig.h"
#include "lane_planner/lane_global_planner.h"

namespace lane_planner {

/**
 * @author Johann Erhard
 *
 * @brief Lane Global Planner ROS interface. This class handles only ROS
 * dependent tasks, i.e., configure parameter server, configure topics, callback
 * handling. All functionallities beyond ROS are handled by LaneGlobalPlanner.
 *
 */
class LaneGlobalPlannerROS : public nav_core::BaseGlobalPlanner {
   public:
    /**
     * @brief Construct a new Lane Global Planner ROS object
     *
     */
    LaneGlobalPlannerROS();

    /**
     * @brief Construct a new Lane Global Planner ROS object
     *
     * @param[in] name Name of Global Planner
     * @param[in,out] costmap_ros Global costmap
     */
    LaneGlobalPlannerROS(std::string name,
                         costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Global planner update callback controlled by planner_frequency
     *
     * @param[in] start Plan's start position
     * @param[in] goal Plan's goal position
     * @param[in,out] plan Current Plan (Populated plan will be published)
     * @return true: New plan was created
     */
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

   private:
    /**
     * @brief Initialize Lane Global Planner ROS object
     *
     * @param[in] name Name of Global Planner
     * @param[in,out] costmap_ros Global costmap
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Configure private topics
     *
     * @param[in,out] private_nh Private Node Handle object
     */
    void configureTopics(ros::NodeHandle* private_nh);

    /**
     * @brief Read and configure parameters
     *
     * @param[in,out] private_nh Private Node Handle object
     */
    void configureParameters(
        ros::NodeHandle* private_nh,
        std::vector<std::vector<geometry_msgs::Point>>* trajectories,
        geometry_msgs::Pose* initial_pose, LaneGlobalPlannerConfig* config);

    /**
     * @brief Configure overlay messages
     *
     */
    void configureOverlays();

    /**
     * @brief Publish overlay messages
     *
     * @param trajectory Trajectory id feedback from backend
     */
    void publishOverlays(int trajectory);

    /**
     * @brief Configure dynamic reconfigure server
     *
     * @param[in,out] private_nh Private Node Handle object
     * @param[in] config Initial parameter config
     */
    void configureDynamicReconfigure(ros::NodeHandle* private_nh,
                                     const LaneGlobalPlannerConfig& config);

    /**
     * @brief Move base feedback callback
     *
     * @param[in] feedback Position and orientation feedback
     */
    void feedbackCallback(
        const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedback);

    /**
     * @brief Car scan callback
     *
     * @param[in] scan Scan zone occupation
     */
    void carScanCallback(const lane_planner::Car_scan::ConstPtr& scan);

    /**
     * @brief Dynamic reconfigure callback: "move_base/LaneGlobalPlannerROS"
     *
     * @param[in] cfg Incoming configuration data
     * @param[in] level Parameter level
     */
    void reconfigureCallback(const LaneGlobalPlannerConfig& cfg,
                             uint32_t level);

    /**
     * @brief Handle Timer callback from LaneGlobalPlanner
     *
     */
    void libTimerCallback(double timeout);

    /**
     * @brief Local ROS Timer has timed out. Notifies LaneGlobalPlanner of this
     * event.
     *
     */
    void timerCallback(const ros::TimerEvent&);

    ros::Timer timer_;  //!< ROS Singleshot Timer

    // Topics
    ros::Subscriber mb_feedback_sub_;        //!< Subscriber: Move base feedback
    ros::Subscriber car_scan_sub_;           //!< Subscriber: Car scan feedback
    ros::Publisher trajectory_overlay_pub_;  //!< Publisher: RVIZ overlay

    jsk_rviz_plugins::OverlayText
        trajectory_overlay_;  //!< RVIZ overlay message

    // General parameters
    std::string name_;  //!< Planner name
    bool initialized_;  //!< ROS interface was initialized

    // Dynamic Reconfigure Server
    std::unique_ptr<dynamic_reconfigure::Server<LaneGlobalPlannerConfig>> srv_;
    boost::recursive_mutex mutex_;

    // Lane Global Planner functionallity
    std::unique_ptr<LaneGlobalPlanner> lane_global_planner_;
};
}  // namespace lane_planner
#endif  // __LANE_GLOBAL_PLANNER_H__
