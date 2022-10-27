#! /usr/bin/env python3
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import rospkg
from tf.transformations import *
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from math import *
#Starts a node that takes the current pose published to the amcl_pose topic and transforms it through translation and quaternion rotation to get the respective pose in the gazebo coordinate frame. translation and rotation needs to be determined manually by comparing rviz and gazebo.
class Gazebo_Visualisation:

    def __init__(self):
        self._map_frame_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._handle_amcl_pose, queue_size=3)
        self._ms_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self._ctrl_hz = 200
        self._trans = [-9.3, 3.8, 0.028 ] # x,y,z translation of gazebo frame to rviz frame
        self._rot = 3.6 #rotation of gazebo to rviz in radiants

## Starts the synchronizing loop thread and initializes the loop rate.

    def run(self):
        rospy.init_node('GazeboVis', anonymous=True)

        self._ctrl_rate = rospy.Rate(self._ctrl_hz)
        rospy.loginfo("Gazebo visualization running!")

        rospy.spin()

    def _handle_amcl_pose(self, pose):
        rviz_position = [pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z]
        rviz_orientation = [pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w]
        gazebo_position, gazebo_orientation = self.transform(rviz_position, rviz_orientation)
        self._pub_to_gazebo(gazebo_position, gazebo_orientation)  # Publish calculated pose.

        print(gazebo_position, gazebo_orientation)

        #self._ctrl_rate.sleep()
    def transform(self, rviz_position, rviz_orientation):
	#calculate respective position in gazebo
        gazebo_position = [(cos(self._rot) * rviz_position[0]) - (sin(self._rot) * rviz_position[1]) + self._trans[0],
                           (sin(self._rot) * rviz_position[0]) + (cos(self._rot) * rviz_position[1]) + self._trans[1],
                            rviz_position[2] + self._trans[2]]
        q_rot = quaternion_from_euler(0,pi,self._rot)
        #calculate respective orientation in gazebo
        gazebo_orientation = quaternion_multiply(q_rot, rviz_orientation)

        return gazebo_position, gazebo_orientation

    def _pub_to_gazebo(self, position, orientation):
        state_msg = ModelState()
        state_msg.model_name = 'tas_car'
        state_msg.pose.position.x = position[0] 
        state_msg.pose.position.y = position[1] 
        state_msg.pose.position.z = position[2]  
        state_msg.pose.orientation.x = orientation[1]
        state_msg.pose.orientation.y = orientation[2]  
        state_msg.pose.orientation.z = orientation[3]  
        state_msg.pose.orientation.w = orientation[0]  
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg)
#
        except rospy.ServiceException:
             print("Service call failed")


def main():
    gazebo_visualisation = Gazebo_Visualisation()
    gazebo_visualisation.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

