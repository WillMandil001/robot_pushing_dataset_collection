#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import datetime
import copy
import numpy as np
import termios
import pandas as pd
import message_filters
import moveit_commander

from cv_bridge import CvBridge
from xela_server.msg import xServerMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
# import the Type: franka_msgs/FrankaState
from franka_msgs.msg import FrankaState

home_pose = PoseStamped()
home_pose.header.frame_id = "panda_link0"
home_pose.pose.position.x = 0.5
home_pose.pose.position.y = 0.0
home_pose.pose.position.z = 0.45
home_pose.pose.orientation.x = 1.0
home_pose.pose.orientation.y = 0.0
home_pose.pose.orientation.z = 0.0
home_pose.pose.orientation.w = 0.0

start_pose = PoseStamped()
start_pose.header.frame_id = "panda_link0"
start_pose.pose.position.x = 0.5
start_pose.pose.position.y = -0.3
start_pose.pose.position.z = 0.025
start_pose.pose.orientation.x = 1.0
start_pose.pose.orientation.y = 0.0
start_pose.pose.orientation.z = 0.0
start_pose.pose.orientation.w = 0.0

class RobotReader(object):
	def __init__(self):
		super(RobotReader, self).__init__()
		rospy.init_node('data_collection_client', anonymous=True, disable_signals=False)
		self.settings = termios.tcgetattr(sys.stdin)
		self.bridge = CvBridge()

		self.robot_sub = message_filters.Subscriber('/joint_states', JointState)      # rate = 499.928
		self.xela_sub = message_filters.Subscriber('/xela1_data', Float64MultiArray)  # rate = 382.624
		self.robot_EE_sub = message_filters.Subscriber('/robot_ee_state', Float64MultiArray)   # rate = 329.289
		self.image_color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
		self.image_depth_sub = message_filters.Subscriber('/camera/color/depth_raw', Image)

		subscribers = [self.robot_sub, self.xela_sub, self.robot_EE_sub, self.image_color_sub, self.image_depth_sub]

		self.robot_publisher = rospy.Publisher('/cartesian_impedance_controller/equilibrium_pose', PoseStamped, queue_size=10)

		while True:
			continue_data_collection = input("Do you want to continue data collection? (y/n): ")
			if continue_data_collection == 'n' or continue_data_collection == 'N':
				break

			# wait for 5 seconds to make sure the robot is in the home position
			self.robot_publisher.publish(home_pose)
			rospy.sleep(5)
			# start_pose.pose.position.x = np.random.uniform(0.3, 0.6)
			self.robot_publisher.publish(start_pose)
			rospy.sleep(5)

			self.stop = False
			self.xelaSensor1 = []
			self.robot_states = []
			self.robot_EE_states = []
			self.image_color_data = []
			self.image_depth_data = []

			self.prev_i = 0
			self.i = 1

			self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=50, slop=0.05, allow_headerless=True)
			self.ts.registerCallback(self.read_robot_data)	

			self.start_time = None

			# final pose:
			final_pose = copy.deepcopy(start_pose)
			final_pose.pose.position.y = 0.3  # np.random.uniform(0.3, 0.6)
			self.robot_publisher.publish(final_pose)

			while not rospy.is_shutdown() and self.stop is False:
				rospy.sleep(0.001)				# place a tiny pause here to avoid the while loop from running too fast
				self.i += 1

			self.stop_time = rospy.get_time()

			print(self.start_time,  self.stop_time)
			print("total time = ", self.stop_time - self.start_time)
			print("time_step = ", (self.stop_time - self.start_time)/len(self.robot_states))

			print("\n Stopped the data collection \n now saving the stored data")
			if len(self.robot_states) > 100:
				self.save_data()

		# wait for 5 seconds to make sure the robot is in the home position
		self.robot_publisher.publish(home_pose)

	def read_robot_data(self, robot_joint_data, xela_data, robot_EE_data, image_color_data, image_depth_data):
		if self.stop == False and self.i != self.prev_i:
			self.prev_i = self.i
			if self.start_time == None:
				self.start_time = rospy.get_time()
			if self.start_time + 5.0 < rospy.get_time():   # (1.0  = 1 second)
				self.stop = True
				
			print("dataset length: ", len(self.robot_states))
			self.robot_states.append(robot_joint_data)
			self.xelaSensor1.append(xela_data.data)
			self.image_color_data.append(image_color_data)
			self.image_depth_data.append(image_depth_data)
			self.robot_EE_states.append(robot_EE_data.data)

	def format_data_for_saving(self):
		print("Formating the data")
		self.robot_states_formated = []
		for data_sample_index in range(len(self.xelaSensor1)):
			self.robot_states_formated.append(list(self.robot_states[data_sample_index].position) + list(self.robot_states[data_sample_index].velocity) + list(self.robot_states[data_sample_index].effort))

	def save_data(self):

		self.format_data_for_saving()

		self.xelaSensor1Formatted = np.array(self.xelaSensor1)

		print("xelaSensor1Formatted length: ", self.xelaSensor1Formatted.shape)
		print("robot_states_formated; ", np.asarray(self.robot_states_formated).shape)

		T1 = pd.DataFrame(self.xelaSensor1Formatted)
		T2 = pd.DataFrame(self.robot_states_formated)
		
		# create new folder for this experiment:
		folder = str('/home/willmandil/Datasets/test_dataset/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
		mydir = os.mkdir(folder)

		xela_Sensor_col = ['txl1_x', 'txl1_y', 'txl1_z', 'txl2_x', 'txl2_y', 'txl2_z','txl3_x', 'txl3_y', 'txl3_z','txl4_x', 'txl4_y', 'txl4_z','txl5_x', 'txl5_y', 'txl5_z','txl6_x', 'txl6_y', 'txl6_z',
		'txl7_x', 'txl7_y', 'txl7_z','txl8_x', 'txl8_y', 'txl8_z','txl9_x', 'txl9_y', 'txl9_z','txl10_x', 'txl10_y', 'txl10_z','txl11_x', 'txl11_y', 'txl11_z','txl12_x', 'txl12_y', 'txl12_z',
		'txl13_x', 'txl13_y', 'txl13_z','txl14_x', 'txl14_y', 'txl14_z','txl15_x', 'txl15_y', 'txl15_z','txl16_x', 'txl16_y', 'txl16_z']

		robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_finger1", "position_panda_finger2",
		"velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_finger1", "velocity_panda_finger2",
		"effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_finger1", "effort_panda_finger2",]

		print(len(self.robot_states_formated[0]))
		print(len(robot_states_col))

		T1.to_csv(folder + '/xela_sensor1.csv', header=xela_Sensor_col, index=False)
		T2.to_csv(folder + '/robot_state.csv',  header=robot_states_col, index=False)
		np.save(folder + '/robot_EE_states.npy', np.array(self.robot_EE_states))
		np.save(folder + '/color_images.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.image_color_data]))
		np.save(folder + '/depth_images.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.image_depth_data]))

if __name__ == "__main__":
	robot_reader = RobotReader()