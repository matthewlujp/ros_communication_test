#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import sys


if __name__ == '__main__':
    # rospy.init_node('test', anonymous=True)
    print(np.zeros((3,3)))
    print("hello {}".format(type(cv2.VideoCapture(0))))
    print("python path: {}".format(sys.executable))