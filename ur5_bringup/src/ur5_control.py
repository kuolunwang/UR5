#!/usr/bin/env python3

import rospy
from arm_operation.srv import *
from arm_operation.msg import *
from std_srvs.srv import Trigger, TriggerResponse
from ur5_bringup.srv import *
import tf

class UR5():
    def __init__(self):
        rospy.Service("/ur5/go_home", Trigger, self.ur5_home)
        rospy.Service("/ur5/get_pose", cur_pose, self.get_pose)
        self.mani_joint_srv = '/ur5_control_server/ur_control/goto_joint_pose'
        self.mani_move_srv = rospy.ServiceProxy(self.mani_joint_srv, joint_pose)
        self.mani_req = joint_poseRequest()
        self.p = joint_value()

        self.listener = tf.TransformListener()

    def get_pose(self, req):

        res = cur_poseResponse()

        try:
            trans, rot = self.listener.lookupTransform("base_link", "ee_gripper_link", rospy.Time(0))
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
        self.p.joint_value = [-3.125488344823019, -0.7493508497821253, -2.38697320619692, -1.5348437468158167, 1.5634725093841553, -1.5657637755023401]
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
    rospy.init_node('ur5_control_node', anonymous=True)
    ur5 = UR5
    rospy.spin()