#!/usr/bin/env python

import rospy
from arm_operation.srv import *
from arm_operation.msg import *
from std_srvs.srv import Trigger, TriggerResponse
from ur5_bringup.srv import *
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from control_msgs.msg import *
from trajectory_msgs.msg import *

class UR5():
    def __init__(self):

        name = rospy.get_param("~name")

        rospy.Service("ur5/go_home", Trigger, self.ur5_home)
        rospy.Service("gripper/open", Trigger, self.open)
        rospy.Service("gripper/close", Trigger, self.close)
        rospy.Service("ur5/get_pose", cur_pose, self.get_pose)
        self.gripper_control_pub = rospy.Publisher("gripper_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)
        self.goto_pose = rospy.ServiceProxy('ur5_control_server/ur_control/goto_pose', arm_operation.srv.target_pose)
        self.mani_joint_srv = 'ur5_control_server/ur_control/goto_joint_pose'
        self.mani_move_srv = rospy.ServiceProxy(self.mani_joint_srv, joint_pose)
        
        self.listener = tf.TransformListener()

    def open(self, req):

        res = TriggerResponse()

        req = FollowJointTrajectoryActionGoal()

        req.goal.trajectory.joint_names = ["gripper_finger1_joint"]
        p = JointTrajectoryPoint()
        p.positions = [0]
        p.velocities = [0]
        p.accelerations = [0]
        p.effort = [0]
        p.time_from_start.secs = 1

        req.goal.trajectory.points = [p]

        self.gripper_control_pub.publish(req)

        res.success = True

        return res

    def close(self, req):

        res = TriggerResponse()

        req = FollowJointTrajectoryActionGoal()

        req.goal.trajectory.joint_names = ["gripper_finger1_joint"]
        p = JointTrajectoryPoint()
        p.positions = [1]
        p.velocities = [0]
        p.accelerations = [0]
        p.effort = [0]
        p.time_from_start.secs = 1

        req.goal.trajectory.points = [p]

        self.gripper_control_pub.publish(req)

        res.success = True

        return res

    def get_pose(self, req):

        res = cur_poseResponse()

        try:
            trans, rot = self.listener.lookupTransform("ur5/base_link", "object_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Service call failed: %s"%e)

        res.pose.position.x = trans[0]
        res.pose.position.y = trans[1]
        res.pose.position.z = trans[2]
        res.pose.orientation.x = rot[0]
        res.pose.orientation.y = rot[1]
        res.pose.orientation.z = rot[2]
        res.pose.orientation.w = rot[3]

        return res

    def ur5_home(self, req):
        self.p = joint_value()
        self.mani_req = joint_poseRequest()

        self.p.joint_value = [0.0011875617783516645, -2.1486170927630823, 2.3022329807281494, -3.3030384222613733, -1.5724604765521448, -1.5706184546100062]
        self.mani_req.joints.append(self.p)
        res = TriggerResponse()

        try:
            rospy.wait_for_service(self.mani_joint_srv)
            mani_resp = self.mani_move_srv(self.mani_req)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)

        return res

if __name__ == "__main__":
    rospy.init_node('ur5_control_node')
    ur5 = UR5()
    rospy.spin()
