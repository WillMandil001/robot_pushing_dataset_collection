#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import numpy as np
import pandas as pd

from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_matrix

def franka_state_callback(msg):
    """
    Callback function for FrankaState messages.
    Extracts the O_T_EE field and publishes it.
    """
    global publisher

    # Extract the O_T_EE field (End-Effector pose in column-major order)
    ee_state = msg.O_T_EE

    # Convert O_T_EE to a 4x4 numpy array
    T = np.array(ee_state).reshape(4, 4).transpose()

    # Use quaternion_from_matrix to get the quaternion
    q = quaternion_from_matrix(T)

    ee_state = [T[0, 3], T[1, 3], T[2, 3], q[0], q[1], q[2], q[3]]

    # Prepare the message to publish
    ee_state_msg = Float64MultiArray()
    ee_state_msg.data = ee_state

    # Publish the end-effector state
    publisher.publish(ee_state_msg)

def main():
    rospy.init_node('franka_state_listener', anonymous=True)

    # Subscriber to /franka_state_controller/franka_states
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, franka_state_callback)

    # Publisher for robot_ee_state
    global publisher
    publisher = rospy.Publisher('/robot_ee_state', Float64MultiArray, queue_size=10)

    rospy.loginfo("Franka State Listener Node started.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass