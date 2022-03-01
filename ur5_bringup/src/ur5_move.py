#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float64
from arm_operation.srv import *
from ur5_bringup.srv import *

class Move():
    def __init__(self):

        rospy.Subscriber("/robot/stick_move_joint_position_controller/command", Float64, self.move, queue_size = 1)
        self.goto_pose = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_pose', arm_operation.srv.target_pose)
        self.goto_home = rospy.ServiceProxy('/ur5/go_home', Trigger)
        self.get_pose = rospy.ServiceProxy("/ur5/get_pose", cur_pose)
        self.tri_req = TriggerRequest()

    def move(self, data):

        
        pose_req = cur_poseRequest()
        pose = self.get_pose(pose_req)

        goto_pose_req = arm_operation.srv.target_poseRequest()
        goto_pose_req.target_pose.position.x = 0.34 + data.data / 2
        goto_pose_req.target_pose.position.y = pose.pose.position.y
        goto_pose_req.target_pose.position.z = pose.pose.position.z
        goto_pose_req.target_pose.orientation.x = pose.pose.orientation.x
        goto_pose_req.target_pose.orientation.y = pose.pose.orientation.y
        goto_pose_req.target_pose.orientation.z = pose.pose.orientation.z
        goto_pose_req.target_pose.orientation.w = pose.pose.orientation.w
        
        self.goto_pose(goto_pose_req)

        rospy.sleep(0.5)

        self.goto_home(self.tri_req)

if __name__ == "__main__":
    rospy.init_node('ur5_move_node')
    move = Move()
    rospy.spin()