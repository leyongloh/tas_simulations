#include "lane_planner/lane_global_planner_ros.h"

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>  // ROS_INFO_STREAM
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <ios>     // std::boolalpha
#include <memory>  // std::make_unique

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(lane_planner::LaneGlobalPlannerROS,
                       nav_core::BaseGlobalPlanner)

using namespace std;

namespace lane_planner {

LaneGlobalPlannerROS::LaneGlobalPlannerROS() {}

LaneGlobalPlannerROS::LaneGlobalPlannerROS(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
}

void LaneGlobalPlannerROS::initialize(std::string name,
                                      costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        name_ = name;

        // Private nodehandle for topics and parameters
        ros::NodeHandle private_nh("~/" + name_);

        // Configure subscribers and publishers
        configureTopics(&private_nh);

        std::vector<std::vector<geometry_msgs::Point>> trajectories;
        geometry_msgs::Pose initial_pose;
        LaneGlobalPlannerConfig config;

        // Load parameters from parameter server
        configureParameters(&private_nh, &trajectories, &initial_pose, &config);

        // Setup lane global planner
        lane_global_planner_ = std::make_unique<LaneGlobalPlanner>(
            name, trajectories, initial_pose);

        lane_global_planner_->registerTimerCallback(
            [this](const double timeout) { this->libTimerCallback(timeout); });

        timer_ = private_nh.createTimer(ros::Duration(1.0),
                                        &LaneGlobalPlannerROS::timerCallback,
                                        this, true);

        // Configure dynamic reconfigure server
        configureDynamicReconfigure(&private_nh, config);

        // Prepare Overlays
        configureOverlays();
        publishOverlays(-1);

        initialized_ = true;

#ifdef PRINT_DEBUG
        ROS_INFO_STREAM(name_ << ": Initialized");
#endif
    }
#ifdef PRINT_DEBUG
    else {

        ROS_INFO_STREAM(name_ << ": Already Initialized");
    }
#endif
}

void LaneGlobalPlannerROS::configureTopics(ros::NodeHandle* private_nh) {
    mb_feedback_sub_ =
        private_nh->subscribe("/move_base/feedback", 50,
                              &LaneGlobalPlannerROS::feedbackCallback, this);

    car_scan_sub_ = private_nh->subscribe(
        "/is_car", 50, &LaneGlobalPlannerROS::carScanCallback, this);

    trajectory_overlay_pub_ =
        private_nh->advertise<jsk_rviz_plugins::OverlayText>(
            "/trajectory_overlay", 50);
}

void LaneGlobalPlannerROS::configureParameters(
    ros::NodeHandle* private_nh,
    std::vector<std::vector<geometry_msgs::Point>>* trajectories,
    geometry_msgs::Pose* initial_pose, LaneGlobalPlannerConfig* config) {
    // Parameter success
    bool success_lane_planner = true;
    bool success_tas = true;
    bool success_trajectories = true;

    // ##############################################################
    // ######################## Lane Planner ########################
    // ##############################################################

    std::string trajectory_topic;

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/trajectories/param",
        trajectory_topic);

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/trajectories/width",
        config->lane_width);

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/trigger_left", config->trigger_left);

    success_lane_planner &=
        private_nh->getParam("/move_base_node/LaneGlobalPlanner/trigger_right",
                             config->trigger_right);

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/debug", config->debug);

    success_lane_planner &=
        private_nh->getParam("/move_base_node/LaneGlobalPlanner/decision_mode",
                             config->decision_mode);

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/maneuver_timeout",
        config->maneuver_timeout);

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/no_maneuver_timeout",
        config->no_maneuver_timeout);

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/interpolation_steps",
        config->interpolation_steps);

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/interpolation_method",
        config->interpolation_method);

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/find_node_method",
        config->find_node_method);

    success_lane_planner &= private_nh->getParam(
        "/move_base_node/LaneGlobalPlanner/maneuver_timeout_trigger_update",
        config->maneuver_timeout_trigger_update);

#ifdef PRINT_DEBUG
    ROS_INFO_STREAM(name_ << ": LanePlanner Parameters loaded: "
                          << std::boolalpha << success_lane_planner);
#endif

    // ##############################################################
    // ############################# TAS ############################
    // ##############################################################

    double inital_yaw = 0.0;
    initial_pose->position.z = 0;

    success_tas &=
        private_nh->getParam("/amcl/initial_pose_x", initial_pose->position.x);
    private_nh->getParam("/amcl/initial_pose_y", initial_pose->position.y);

    // Read inital orientation
    success_tas &= private_nh->getParam("/amcl/initial_pose_a", inital_yaw);
    initial_pose->orientation = tf::createQuaternionMsgFromYaw(inital_yaw);

#ifdef PRINT_DEBUG
    ROS_INFO_STREAM(name_ << ": TAS Parameters loaded: " << std::boolalpha
                          << success_tas);
#endif

    // ##############################################################
    // ######################## Trajectories ########################
    // ##############################################################

    XmlRpc::XmlRpcValue trajectories_xml;

    success_trajectories &=
        private_nh->getParam(trajectory_topic.c_str(), trajectories_xml);

    if (success_trajectories) {
        // Read trajectories_ from parameter server
        geometry_msgs::Point point;
        point.z = 0.0;

        for (const auto& trajectory_xmlrpc : trajectories_xml) {
#ifdef PRINT_DEBUG
            ROS_INFO_STREAM(name_ << ": Read trajectory: "
                                  << trajectory_xmlrpc.first);
#endif

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
#ifdef PRINT_DEBUG
    ROS_INFO_STREAM(name_ << ": Trajectories loaded: " << std::boolalpha
                          << success_trajectories);
#endif
}

void LaneGlobalPlannerROS::configureOverlays() {
    trajectory_overlay_.action = jsk_rviz_plugins::OverlayText::ADD;
    trajectory_overlay_.width = 180;
    trajectory_overlay_.height = 50;
    trajectory_overlay_.left = 10;
    trajectory_overlay_.top = 10;
    trajectory_overlay_.text_size = 30;
    trajectory_overlay_.fg_color.r = 1.0;
    trajectory_overlay_.fg_color.g = 1.0;
    trajectory_overlay_.fg_color.b = 1.0;
    trajectory_overlay_.fg_color.a = 1.0;
}

void LaneGlobalPlannerROS::publishOverlays(int trajectory) {
    // Prepare Trajectory Overlay
    trajectory_overlay_.text = "Lane: " + std::to_string(trajectory);

    if (trajectory < 0) {
        trajectory_overlay_.bg_color.r = 1.0;
        trajectory_overlay_.bg_color.g = 0.0;
        trajectory_overlay_.bg_color.b = 0.0;
        trajectory_overlay_.bg_color.a = 0.6;
    } else {
        trajectory_overlay_.bg_color.r = 0.0;
        trajectory_overlay_.bg_color.g = 1.0;
        trajectory_overlay_.bg_color.b = 0.0;
        trajectory_overlay_.bg_color.a = 0.6;
    }

    // Publish Trajectory Overlay
    trajectory_overlay_pub_.publish(trajectory_overlay_);
}

void LaneGlobalPlannerROS::configureDynamicReconfigure(
    ros::NodeHandle* private_nh, const LaneGlobalPlannerConfig& config) {
    // Create dynamic reconfigure server
    srv_ =
        std::make_unique<dynamic_reconfigure::Server<LaneGlobalPlannerConfig>>(
            mutex_, *private_nh);

    // Update dynamic values from parameter server
    srv_->updateConfig(config);

    // Reigster dynamic reconfigure server
    srv_->setCallback([this](LaneGlobalPlannerConfig& config, uint32_t level) {
        this->reconfigureCallback(config, level);
    });
}

void LaneGlobalPlannerROS::feedbackCallback(
    const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedback) {
    lane_global_planner_->updateLastPose(feedback->feedback.base_position.pose);
}

void LaneGlobalPlannerROS::carScanCallback(
    const lane_planner::Car_scan::ConstPtr& scan) {
    lane_global_planner_->updateLastScan(*scan);
}

void LaneGlobalPlannerROS::reconfigureCallback(
    const LaneGlobalPlannerConfig& cfg, const uint32_t level) {
    lane_global_planner_->updateParameters(
        cfg.debug, cfg.decision_mode, cfg.lane_width, cfg.trigger_left,
        cfg.trigger_right, cfg.interpolation_steps, cfg.maneuver_timeout,
        cfg.no_maneuver_timeout, cfg.interpolation_method, cfg.find_node_method,
        cfg.maneuver_timeout_trigger_update);
}

void LaneGlobalPlannerROS::libTimerCallback(const double timeout) {
#ifdef PRINT_DEBUG
    ROS_INFO_STREAM(name_ << ": Start Timer: " << timeout << " [seconds]");
#endif

    timer_.stop();
    timer_.setPeriod(ros::Duration(timeout));
    timer_.start();
}

void LaneGlobalPlannerROS::timerCallback(const ros::TimerEvent&) {
#ifdef PRINT_DEBUG
    ROS_INFO_STREAM(name_ << ": Timer Finished");
#endif

    lane_global_planner_->resetState();
}

bool LaneGlobalPlannerROS::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
#ifdef PRINT_DEBUG
        ROS_INFO_STREAM(name_ << ": makePlan not possible");
#endif
        return false;
    }

    // Generate new Plan
    lane_global_planner_->makePlan(start, goal, &plan);

    // Publish Overlays
    publishOverlays(lane_global_planner_->getTrajectory());

    return true;
}
}  // namespace lane_planner
