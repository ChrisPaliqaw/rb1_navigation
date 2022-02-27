#!/usr/bin/env python

"""
$ rosmsg show move_base_msgs/MoveBaseActionGoal
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalID goal_id
  time stamp
  string id
move_base_msgs/MoveBaseGoal goal
  geometry_msgs/PoseStamped target_pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
"""

"""
poi.yaml format:

init_position:
 position:
  x: -4.28077294766
  y: -3.68536082784
  z: 0.0
 orientation:
  x: 0.0
  y: 0.0
  z: 0.0491798419191
  w: 0.998789939451

loading_position:
 position:
  x: 3.72727024704
  y: 1.62105207312
  z: 0.0
 orientation:
  x: 0.0
  y: 0.0
  z: 0.488753291806
  w: 0.872422042219
"""

import rospy
import actionlib
from rb1_localization.srv import POI, POIRequest, POIResponse
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseFeedback, MoveBaseResult

class GoToPoint():

    SERVICE_NAME = "go_to_point"

    ACTION_NAME = "move_base"
    
    def __init__(self):
        self._service = rospy.Service(GoToPoint.SERVICE_NAME, POI, self.__service_callback)
        rospy.loginfo(f"Started service {GoToPoint.SERVICE_NAME}")
        self._ac = actionlib.SimpleActionClient(GoToPoint.ACTION_NAME, MoveBaseAction)
        rospy.loginfo(f"Attached to action {GoToPoint.ACTION_NAME}")

        self._ctrl_c = False
        rospy.on_shutdown(self.__shutdownhook)
    
    def __service_callback(self, request: POIRequest):
        response = POIResponse()
        rosparam_path = f"{GoToPoint.SERVICE_NAME}/{request.label}"
        if rospy.has_param(rosparam_path):
           poi = rospy.get_param(rosparam_path)
           rospy.loginfo(f"Located rosparam {rosparam_path}")
           rospy.loginfo(f"POI value is:\n {poi}")
           rospy.loginfo(f"Waiting for action server...")
           self._ac.wait_for_server()
           rospy.loginfo(f"Action server is ready")
           action_goal = MoveBaseGoal()
           action_goal.target_pose.header.frame_id = 'map'
           action_goal.target_pose.pose.position.x = poi["position"]["x"]
           action_goal.target_pose.pose.position.y = poi["position"]["y"]
           action_goal.target_pose.pose.position.z = poi["position"]["z"]
           action_goal.target_pose.pose.orientation.x = poi["orientation"]["x"]
           action_goal.target_pose.pose.orientation.y = poi["orientation"]["y"]
           action_goal.target_pose.pose.orientation.z = poi["orientation"]["z"]
           action_goal.target_pose.pose.orientation.w = poi["orientation"]["w"]
           self._ac.send_goal(action_goal)
           rospy.loginfo(f"Waiting for action result...")
           self._ac.wait_for_result()
           rospy.loginfo(f"Action complete")
           response.success = True
        else:
            rospy.logerr(f"Couldn't locate rosparam {rosparam_path}")
            response.success = False
        return response
        
    def __shutdownhook(self):
        self.ctrl_c = True
            
if __name__ == '__main__':
    rospy.init_node(GoToPoint.SERVICE_NAME)
    GoToPoint()
    rospy.spin()