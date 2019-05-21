#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import (
        FollowJointTrajectoryAction,
        FollowJointTrajectoryGoal,
    )
from trajectory_msgs.msg import (
        JointTrajectoryPoint,
    )

import yaml
import argparse

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

jaxon_index_lst = [
    'RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5',
    'LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5',
    'CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2', 'HEAD_JOINT0', 'HEAD_JOINT1',
    'RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7',
    'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7',
    'LARM_F_JOINT0', 'LARM_F_JOINT1', 'RARM_F_JOINT0', 'RARM_F_JOINT1',
]

class SingleTrajectory:
    def __init__(self, namespace='AizuSpiderAA', action_name='fullbody_controller/follow_joint_trajectory'):
        print '%s/%s'%(namespace, action_name)
        self.act_ = actionlib.SimpleActionClient(
            '%s/%s'%(namespace, action_name),
            FollowJointTrajectoryAction,
        )
        self.act_.wait_for_server(rospy.Duration(10.0))

    def feedback(self, msg):
        pass

    def call(self, ref_list, offset = 0.0):
        '''ref_list = [ [tm, joints, q], ... ]'''
        goal = FollowJointTrajectoryGoal()

        if len(ref_lst[0][1]) < 14:
            goal.trajectory.joint_names = index_lst
        else:
            goal.trajectory.joint_names = jaxon_index_lst

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

        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        print(goal.trajectory)
        self.act_.send_goal(goal, feedback_cb=self.feedback)

    def wait(self, tm=rospy.Duration(10)):
        self.act_.wait_for_result(tm)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='call trajectory action by PoseSeq')
    parser.add_argument('-N', '--name', default = None, type=str)
    parser.add_argument('-F', '--file_name', default = None, type=str)
    parser.add_argument('--offset',     default = None, type=str)
    parser.add_argument('--wait',       default = None, type=str)
    parser.add_argument('--action',     default = None, type=str)

    rospy.init_node('call_from_pseq', anonymous=True)

    args = parser.parse_args()

    name  = args.name
    if not name:
        name = ''

    filename  = args.file_name
    if not filename:
        print('PoseSeq file required with -F option')
        exit(1)

    offset = args.offset
    if not offset:
        offset = 2.0
    else:
        offset = float(offset)

    wait_tm = args.wait
    if not wait_tm:
        wait_tm = 1.0
    else:
        wait_tm = float(wait_tm)

    action_name = args.action

    f = open(filename, 'r')
    base_pseq = yaml.load(f)
    pseq = base_pseq['refs']

    ref_lst = []
    for p in pseq:
        ref = p['refer']
        tm =  p['time']
        js = ref['joints']
        q  = ref['q']
        ##print tm,js,q
        ref_lst.append([tm, js, q])

    if action_name:
        sg = SingleTrajectory(namespace=name, action_name=action_name)
    else:
        sg = SingleTrajectory(namespace=name)

    sg.call(ref_lst, offset=offset)
    sg.wait(rospy.Duration(wait_tm))
