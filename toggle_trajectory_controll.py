#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState,
    )
from control_msgs.srv import (
    QueryTrajectoryState
    )
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    )
from std_srvs.srv import (
    Empty,
    )

import yaml
import argparse

class SingleTrajectory:
    def __init__(self, namespace='AizuSpiderAA'):
        self.act_ = actionlib.SimpleActionClient(
            '%s/fullbody_controller/follow_joint_trajectory'%(namespace),
            FollowJointTrajectoryAction,
        )
        self.act_.wait_for_server(rospy.Duration(10.0))

        self.sub_ = rospy.Subscriber('%s/fullbody_controller/state'%(namespace),
                                     JointTrajectoryControllerState, self.state_callback)

        self.finish_action_ = False

    def go_(self):
        state_srv = rospy.ServiceProxy('/AizuSpiderAA/fullbody_controller/query_state',
                                       QueryTrajectoryState)
        res = state_srv(rospy.Time.now())

        names     = res.name
        positions = res.position

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = names

        goal.trajectory.points = []
        p = JointTrajectoryPoint()
        p.time_from_start = rospy.Duration(0.1)
        p.positions = positions
        goal.trajectory.points.append(p)
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.01)

        print(goal.trajectory)
        self.act_.send_goal(goal, feedback_cb=self.feedback)

        self.act_.wait_for_result(rospy.Duration(0.2))
        self.finish_action_ = True

    def state_callback(self, msg):
        print('callback')
        ## shutdown self
        self.sub_.unregister()

        ##
        names = msg.joint_names
        #positions = msg.desired.positions
        positions = msg.actual.positions

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = names

        goal.trajectory.points = []

        p = JointTrajectoryPoint()
        p.time_from_start = rospy.Duration(0.1)
        p.positions = positions
        goal.trajectory.points.append(p)
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        print(goal.trajectory)
        self.act_.send_goal(goal, feedback_cb=self.feedback)

        self.act_.wait_for_result(rospy.Duration(0.2))
        self.finish_action_ = True

    def feedback(self, msg):
        pass

    def call(self, ref_list, offset = 0.0):
        '''ref_list = [ [tm, joints, q], ... ]'''
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ['FR_FLIPPER', 'FL_FLIPPER', 'BR_FLIPPER', 'BL_FLIPPER',
                                       'SHOULDER', 'ARM', 'FOREARM', 'WRIST1', 'WRIST2', 'HAND',
                                       'FINGER1', 'FINGER2', 'FINGER3', ]
        goal.trajectory.points = []
        for ref in ref_list:
            p = JointTrajectoryPoint()
            p.time_from_start = rospy.Duration(ref[0] + offset)
            qlst = []
            q = ref[2]
            for j in ref[1]:
                qlst.append(q[j])
            p.positions = qlst
            goal.trajectory.points.append(p)

        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.01)

        print(goal.trajectory)
        self.act_.send_goal(goal, feedback_cb=self.feedback)

    def wait(self, tm=rospy.Duration(10)):
        self.act_.wait_for_result(tm)

# index_map = {
#     0: 'FR_FLIPPER',
#     1: 'FL_FLIPPER',
#     2: 'BR_FLIPPER',
#     3: 'BL_FLIPPER',
#     4: 'SHOULDER',
#     5: 'ARM',
#     6: 'FOREARM',
#     7: 'WRIST1',
#     8: 'WRIST2',
#     9: 'HAND',
#     10: 'FINGER1',
#     11: 'FINGER2',
#     12: 'FINGER3',
#     }
index_lst = [
    'FR_FLIPPER',
    'FL_FLIPPER',
    'BR_FLIPPER',
    'BL_FLIPPER',
    'SHOULDER',
    'ARM',
    'FOREARM',
    'WRIST1',
    'WRIST2',
    'HAND',
    'FINGER1',
    'FINGER2',
    'FINGER3',
    ]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='call trajectory action by PoseSeq')
    parser.add_argument('-N', '--name', default = None, type=str)
    parser.add_argument('--wait',       default = None, type=str)

    rospy.init_node('toggle_trajectory_controll', anonymous=True)

    args = parser.parse_args()

    name  = args.name
    if not name:
        name = 'AizuSpiderAA'

    wait_tm = args.wait
    if not wait_tm:
        wait_tm = 1.0
    else:
        wait_tm = float(wait_tm)

    sg = SingleTrajectory(namespace=name)

    start = rospy.Time.now()
    while not sg.finish_action_:
        now = rospy.Time.now()
        if (now.to_sec() - start.to_sec()) > 3.0:
            print("timeout")
            exit(1)

    rospy.wait_for_service('%s/toggle_controller'%(name))
    try:
        toggle_srv = rospy.ServiceProxy('%s/toggle_controller'%(name), Empty)
        res = toggle_srv()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        exit(1)

    print("finish")
