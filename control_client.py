#!/usr/bin/env python

import rospy

from aizuspider_description.srv import (
    Control,
    )
#import aizuspider_description.srv

from pyquaternion import Quaternion
import math

def control(req):
    st = req.state
    print st

    tq = 0

    #res = aizuspider_description.srv.ControlResponse()
    res = Control._response_class()

    res.action = [tq]

    return res

rospy.init_node('control_client')

s = rospy.Service('/chainerrl/control', Control, control)

rospy.spin()
