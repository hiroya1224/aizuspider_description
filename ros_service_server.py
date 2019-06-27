from __future__ import print_function ### for python2 compatible with python3

import cnoid.Base
import cnoid.BodyPlugin
import cnoid.Util

import numpy, time, math

import sys
setattr(sys, 'argv', [''])

import rospy
from roseus.srv import *

thisSimulatorItem = None

def roscallback (req):
    ret = 'not called'
    if (len(req.str)) > 0:
        print('service called: %s'%(req.str))
        ret = eval(req.str)
        print('service result: %s'%(ret))
    return StringStringResponse(ret)

def ros_service_init ():
    rospy.init_node('choreonoid_ros')
    s = rospy.Service('/choreonoid_service', StringString, roscallback)
    ## rospy.spin()

def resetPosition(robotname = "InvPendulum", pos = [0, 0.5, 0.1], rpy = [0,0,0], sleep = 0.1):
    #resetPosition("JAXON_RED", [0, 0, 1.0], [0, 0, 0])
    global thisSimulatorItem

    if callable(cnoid.Base.RootItem.instance):
        robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    else:
        robotItem = cnoid.Base.RootItem.instance.findItem(robotname)
    if robotItem == None:
        return '(:fail "invalid robotname %s")'%(robotname)

    if thisSimulatorItem == None:
        thisSimulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
        if thisSimulatorItem == None:
            return '(:fail "invalid simulatorItem")'

    mat = cnoid.Util.rotFromRpy(rpy)

    trs = numpy.array([[mat[0][0], mat[0][1], mat[0][2], pos[0]],
                       [mat[1][0], mat[1][1], mat[1][2], pos[1]],
                       [mat[2][0], mat[2][1], mat[2][2], pos[2]],
                       [        0,         0,         0,      1] ])

    thisSimulatorItem.setForcedPosition(robotItem, trs)
    tm = thisSimulatorItem.getCurrentTime()
    while tm == thisSimulatorItem.getCurrentTime():
        time.sleep(0.002)
    thisSimulatorItem.pauseSimulation()

    thisSimulatorItem.clearForcedPositions()
    thisSimulatorItem.restartSimulation()

    return '(:success)'

def callSimulation(robotname = "InvPendulum", call = 'pauseSimulation'):
    global thisSimulatorItem

    if callable(cnoid.Base.RootItem.instance):
        robotItem = cnoid.Base.RootItem.instance().find(robotname)
    else:
        robotItem = cnoid.Base.RootItem.instance.find(robotname)
    if robotItem == None:
        return '(:fail "invalid robotname %s")'%(robotname)

    if thisSimulatorItem == None:
        thisSimulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
        if thisSimulatorItem == None:
            return '(:fail "invalid simulatorItem")'

    ret = eval('thisSimulatorItem.%s()'%(call))

    return '(:success "%s")'%(ret)

def getCoordinate(robotname = 'InvPendulum', linkname = 'WAIST'):
    if callable(cnoid.Base.RootItem.instance):
        robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    else:
        robotItem = cnoid.Base.RootItem.instance.findItem(robotname)
    if robotItem == None:
        return '(:fail "invalid robotname %s")'%(robotname)

    if callable(robotItem.body):
        iLink = robotItem.body().link(linkname)
    else:
        iLink = robotItem.body.link(linkname)

    if iLink == None:
        return '(:fail "invalid linkname %s")'%(linkname)

    coords = iLink.position()
    pos = coords[0:3, 3]
    rot = coords[0:3, 0:3]

    return '(:success (make-coords :pos #f(%f %f %f) :rot #2f((%f %f %f) (%f %f %f) (%f %f %f))))'%(pos[0]*1000,pos[1]*1000,pos[2]*1000,rot[0][0],rot[0][1],rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],rot[2][1],rot[2][2])

ros_service_init()

# counter = 0
# thisSimulatorItem = None
# while True:
#     time.sleep(5)
#     robotItem = cnoid.Base.RootItem.instance.find('InvPendulum')
#     if robotItem == None:
#         print('(:fail "invalid robotname %s")'%(robotname))
#     if not thisSimulatorItem:
#         thisSimulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
#     if thisSimulatorItem == None:
#         print('(:fail "invalid simulatorItem")')
#     else:
#         if counter % 2 == 0:
#             thisSimulatorItem.stopSimulation()
#         else:
#             thisSimulatorItem.startSimulation()
#     counter = counter + 1
