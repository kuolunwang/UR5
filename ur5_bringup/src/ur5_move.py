#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float64

class Move():
    def __init__(self):

        rospy.Subscriber("/robot/stick_move_joint_position_controller/command", Float64, self.move, queue_size = 1)
        self.goto_home = rospy.ServiceProxy('/ur5/go_home', Trigger)
        self.goto_move = rospy.ServiceProxy('/ur5/go_move', Trigger)
        self.tri_req = TriggerRequest()

    def move(self, data):

        if(data.data >= 0.2):
            self.goto_move(self.tri_req)
        else:
            self.goto_home(self.tri_req)

if __name__ == "__main__":
    rospy.init_node('ur5_move_node')
    move = Move()
    rospy.spin()