#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from respeaker_ros import RespeakerNode

if __name__ == '__main__':
    rospy.init_node("respeaker_node")
    n = RespeakerNode()
    rospy.spin()
