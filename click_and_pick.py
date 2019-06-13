#!/usr/bin/env python

import rospy
import tf

from aizuspider_description.srv import (
    Grasp,
    SolveIK,
    )

from geometry_msgs.msg import (
    Pose,
    PointStamped,
    )

from pyquaternion import Quaternion
import math

name = 'AizuSpiderAA'

rospy.init_node('clicked_point', anonymous=True)

listener = tf.TransformListener()

rospy.wait_for_service('%s/grasp'%(name))
rospy.wait_for_service('%s/solve_ik'%(name))

grasp_srv = rospy.ServiceProxy('%s/grasp'%(name), Grasp)
solve_ik_srv = rospy.ServiceProxy('%s/solve_ik'%(name), SolveIK)

def do_grasp():
    try:
        res = grasp_srv(position=[math.pi/3, math.pi/3, math.pi/3], time=1000, wait=False)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        rospy.signal_shutdown('service error')

def solve_ik(cds):
    try:
        # cds = Pose()
        # cds.position.x =  0.424
        # cds.position.y = -0.1583
        # cds.position.z = 1.0291
        # q = Quaternion(axis=[0, 1, 0], angle=0.35)
        # cds.orientation.w = q.elements[0]
        # cds.orientation.x = q.elements[1]
        # cds.orientation.y = q.elements[2]
        # cds.orientation.z = q.elements[3]
        res = solve_ik_srv(pose=cds, position_ik=False, move=2000, wait=True)
        print res
        dir(res)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        rospy.signal_shutdown('service error')

def callback(ptmsg):
    rospy.loginfo("callback %s", ptmsg)
    #ptmsg.header.frame_id
    #ptmsg.header.stamp
    try:
        (trans,rot) = listener.lookupTransform('/%s/CHASSIS'%(name), ptmsg.header.frame_id, ptmsg.header.stamp)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.signal_shutdown('tf error')

    print trans
    print rot
    q = Quaternion(rot)
    ptrans = q.rotate([ ptmsg.point.x, ptmsg.point.y, ptmsg.point.z ])
    print ptrans
    cds = Pose()
    cds.position.x = trans[0] + ptrans[0]
    cds.position.y = trans[1] + ptrans[1]
    cds.position.z = trans[2] + ptrans[2]
    ##q = Quaternion(axis=[0, 1, 0], angle=0.35)
    cds.orientation.w = 1
    cds.orientation.x = 0
    cds.orientation.y = 0
    cds.orientation.z = 0

    solve_ik(cds)

    rospy.signal_shutdown('finished')

rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, callback)

rospy.spin()
