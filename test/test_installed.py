#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import unittest



class TestInstalled(unittest.TestCase):
    def test_installed(self):
        try:
            import angles
            from contextlib import contextmanager
            import usb.core
            import usb.util
            import pyaudio
            import math
            import numpy as np
            import tf.transformations as T
            import os
            import rospy
            import struct
            import sys
            import time
            from audio_common_msgs.msg import AudioData
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Bool, Int32, ColorRGBA
            from dynamic_reconfigure.server import Server
            try:
                from pixel_ring import usb_pixel_ring_v2
            except IOError as e:
                pass
            self.assertTrue(True, 'All modules are installed')
        except ImportError:
            self.fail()


if __name__ == '__main__':
    import rostest
    rostest.rosrun('respeaker_ros', 'test_installed', TestInstalled)
