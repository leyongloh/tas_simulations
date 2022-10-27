#! /usr/bin/env python
# coding: utf-8

## @file
# Contains a main function for the ordered launch the gazebo visualisation depending on the running Gazebo simulation.

## @author Martin 
## @maintainer Jiangnan

import roslaunch
import rospy
import time
import rospkg

## Cleanly starts the visualisation launch file after gazebo comes up.
# @return Returns True in case of success and False in case of failure. Type bool.
def main():

    rospy.init_node('tas_sim_staged_launcher', anonymous=True)

    rospack = rospkg.RosPack()
    time.sleep(5) # Avoid catching emptyWorld as it comes up.
    service = '/gazebo/get_physics_properties'
    rospy.loginfo('waitForService /gazebo/get_physics_properties ...')
    try:
        rospy.wait_for_service(service, timeout=80)  # Might also be unavailable because of a paused simulation!
    except rospy.ROSException:
        rospy.logerr('Could not contact gazebo. Shutting down.')
        rospy.signal_shutdown('timeout')
        time.sleep(0.5)
        return False
    except rospy.ServiceException:
        rospy.logerr('Could not contact gazebo. Shutting down.')
        rospy.signal_shutdown('timeout')
        time.sleep(0.5)
        return False

    rospy.sleep(1)
    rospy.loginfo('/gazebo/get_physics_properties ready, starting..')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    navigation_type = rospy.get_param('~navigation_type')
    launch = roslaunch.parent.ROSLaunchParent(uuid, [rospack.get_path(navigation_type)+'/launch/RvizGazeboTransNode.launch'])
    launch.start()
    rospy.loginfo("started")    

    while not rospy.is_shutdown():
        rospy.spin()

    launch.shutdown()
    return True

if __name__ == '__main__':
    main()
