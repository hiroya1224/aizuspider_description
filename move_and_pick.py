#!/usr/bin/env python
from __future__ import print_function

import click_and_pick as cap
from move import MovePub

import rospy
import tf
import math
import argparse

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

msg = """

"""

def callback(ptmsg):
    rospy.loginfo("callback %s", ptmsg)
    #ptmsg.header.frame_id
    #ptmsg.header.stamp
    listener.waitForTransform('/%s/CHASSIS'%(name), ptmsg.header.frame_id, ptmsg.header.stamp, rospy.Duration(5.0))
    try:
        (trans,rot) = listener.lookupTransform('/%s/CHASSIS'%(name), ptmsg.header.frame_id, ptmsg.header.stamp)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print "Tf error %s"%(e)
        rospy.signal_shutdown('tf error')

    print 'tr:', trans
    print 'rt:', rot
    q = Quaternion([ rot[3], rot[0], rot[1], rot[2] ])
    print 'org:', [ ptmsg.point.x, ptmsg.point.y, ptmsg.point.z ]
    ptrans = q.rotate([ ptmsg.point.x, ptmsg.point.y, ptmsg.point.z ])
    print 'tgt:', ptrans

    cds = Pose()
    cds.position.x = trans[0] + ptrans[0]
    cds.position.y = trans[1] + ptrans[1]
    cds.position.z = trans[2] + ptrans[2]
    ##q = Quaternion(axis=[0, 1, 0], angle=0.35)
    cds.orientation.w = 1
    cds.orientation.x = 0
    cds.orientation.y = 0
    cds.orientation.z = 0

    move_to_obj(trans)

    if solve_ik(cds):
        do_grasp()
    else:
        print "IK failed"

    ### pick up ...

    rospy.signal_shutdown('finished')


def move_to_obj(trans):
    ### move mode ###
    rospy.init_node('move_sample')

    mv = MovePub(name)

    rospy.Subscriber('%sground_truth_pose'%(name), PoseStamped, mv.callback)
    mv.wait_first_callback()

    mv.move(trans[0], 0)
    mv.move(0, 0)
    mv.move(0, 0)

### pick mode ###
name = 'AizuSpiderAA'

rospy.init_node('clicked_point', anonymous=True)

listener = tf.TransformListener()

rospy.wait_for_service('%s/grasp'%(name))
rospy.wait_for_service('%s/solve_ik'%(name))

grasp_srv = rospy.ServiceProxy('%s/grasp'%(name), Grasp)
solve_ik_srv = rospy.ServiceProxy('%s/solve_ik'%(name), SolveIK)

rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, callback)
rospy.spin()
