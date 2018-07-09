#!/usr/bin/env python
#encoding = utf-8



import rospy
import roslib
import mavros
import random
import argparse
from itertools import cycle

import time, serial, datetime

from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3
from sensor_msgs.msg import Imu, BatteryState
from mavros_msgs.msg import *
from mavros_msgs.srv import *

from tf.transformations import euler_from_quaternion

import sys, signal
def signal_handler(signal, frame):
	print("\nprogram exiting gracefully")
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

SERIAL_PORT = "/dev/ttyACM0"
FIELD_NUM = 16
TIMESTAMP, VEL_X, VEL_Y, VEL_Z, ACC_X, ACC_Y, ACC_Z, ROLL, PITCH, YAW, VOL, ENERGY, ACT_VX, ACT_VY, ACT_VZ, MODE = range(FIELD_NUM)
LOG_ITEMS = [TIMESTAMP, VEL_X, VEL_Y, VEL_Z, ACC_X, ACC_Y, ACC_Z, ROLL, PITCH, YAW, ENERGY, ACT_VX, ACT_VY, ACT_VZ, MODE]

# TODO
HEADER_STRINGS = {
	TIMESTAMP: "timestamp",
	VEL_X: "vel_x",
	VEL_Y: "vel_y",
	VEL_Z: "vel_z",
	ACC_X: "acc_x",
	ACC_Y: "acc_y",
	ACC_Z: "acc_z",
	ROLL: "roll",
	PITCH: "pitch",
	YAW: "yaw",
	VOL: "vol",
	ENERGY: "energy",
	ACT_VX: "act_vx",
	ACT_VY: "act_vy",
	ACT_VZ: "act_vz",
	MODE: "mode"

}

SETPOINT_FLAG = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
				PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
				PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE

SETPOINT_IGNORE_ALL_FLAG = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
				PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
				PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
				PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE


class AeroEnergyLogger(object):

	def __init__(self, filename=None):
		self.ser = serial.Serial(SERIAL_PORT, 9600)

		if filename:
			self.log_file = open("logs/%s_log.csv" % filename, 'w')
		else:
			self.log_file = open("logs/%s_log.csv" % datetime.datetime.now().strftime('%m%d%H%M%S'), 'w')

		# self.log_file.write("timestamp,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,roll,pitch,yaw,rc0,rc1,rc2,rc3,vol,energy,act_vx,act_vy,act_vz,mode\n")
		self.log_file.write(",".join([HEADER_STRINGS[i] for i in LOG_ITEMS])+"\n")

		mavros.set_namespace()
		rospy.init_node('aero_energy_logger')

		self.cur_state = State()
		self.velocity = TwistStamped()
		self.rc_in = RCIn()
		self.imu = Imu()
		self.manual_control = ManualControl()
		self.battery_state = BatteryState()
		self.pose = PoseStamped()

		self.cur_val_list = [0] * FIELD_NUM
		self.log_val_list = [0] * FIELD_NUM

		self.last_raw_power = 0
		self.raw_power_sum = 0

		self.last_timestamp = 0



		print "initialize subscribers..."
		self.init_subscribers()
		print "initialize services..."
		self.init_services()
		print "initialize publishers..."
		self.init_publishers()

		self.start_time = time.time()

	def init_subscribers(self):
		rospy.Subscriber(mavros.get_topic('state'), State, self.state_callback)
		rospy.Subscriber(mavros.get_topic('local_position', 'velocity'), TwistStamped, self.velocity_callback)
		rospy.Subscriber(mavros.get_topic('battery'), BatteryState, self.battery_state_callback)
		rospy.Subscriber(mavros.get_topic('imu', 'data'), Imu, self.imu_callback)
		rospy.Subscriber(mavros.get_topic('manual_control', 'control'), ManualControl, self.manual_control_callback)
		rospy.Subscriber(mavros.get_topic('rc', 'in'), RCIn, self.rc_in_callback)
		rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, self.pose_callback)

	def init_services(self):
		rospy.wait_for_service(mavros.get_topic('cmd', 'arming'))
		rospy.wait_for_service(mavros.get_topic('set_mode'))
		rospy.wait_for_service(mavros.get_topic('cmd', 'takeoff'))
		self.arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
		self.set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
		self.takeoff_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'takeoff'), CommandTOL)

	def init_publishers(self):
		self.set_vel_pub = rospy.Publisher(mavros.get_topic('setpoint_raw', 'local'), PositionTarget, queue_size=10)

	def state_callback(self, state):
		self.cur_state = state

		self.cur_val_list[MODE] = self.cur_state.mode

	def velocity_callback(self, velocity):
		self.velocity = velocity

		# self.cur_val_list[VEL_X] = "{:.5f}".format(self.velocity.twist.linear.x)
		self.cur_val_list[VEL_X] = self.velocity.twist.linear.x
		self.cur_val_list[VEL_Y] = self.velocity.twist.linear.y
		self.cur_val_list[VEL_Z] = self.velocity.twist.linear.z

	def battery_state_callback(self, battery_state):
		self.battery_state = battery_state

		self.cur_val_list[VOL] = self.battery_state.voltage

	def imu_callback(self, imu):
		self.imu = imu

		self.cur_val_list[ACC_X] = self.imu.linear_acceleration.x
		self.cur_val_list[ACC_Y] = self.imu.linear_acceleration.y
		self.cur_val_list[ACC_Z] = self.imu.linear_acceleration.z

	def manual_control_callback(self, manual_control):
		self.manual_control = manual_control

	def rc_in_callback(self, rc_in):
		self.rc_in = rc_in

		self.cur_val_list[RC0] = self.rc_in.channels[0]
		self.cur_val_list[RC1] = self.rc_in.channels[1]
		self.cur_val_list[RC2] = self.rc_in.channels[2]
		self.cur_val_list[RC3] = self.rc_in.channels[3]

	def pose_callback(self, pose):
		self.pose = pose

		orientation = self.pose.pose.orientation
		qs = [orientation.x, orientation.y, orientation.z, orientation.w]
		roll, pitch, yaw = euler_from_quaternion(qs)

		self.cur_val_list[ROLL] = roll
		self.cur_val_list[PITCH] = pitch
		self.cur_val_list[YAW] = yaw

	def update_current(self, raw_value):
		self.cur_val_list[TIMESTAMP] = time.time() - self.start_time
		self.cur_val_list[CUR_RAW] = raw_value
		
		# vol = (raw_value / 1024.0) * 5000
		# amps = ((vol - 2500) / 100)
		# self.current = amps

		# self.cur_val_list[CUR] = self.current
		# self.cur_val_list[POWER] = self.cur_val_list[VOL] * self.cur_val_list[CUR]

		self.cur_val_list = [str(x) for x in self.cur_val_list]

		log_str = ",".join(self.cur_val_list)
		self.log_file.write("%s\n" % log_str)

	def update_raw_power_sum(self, raw_current_value=None):
		now = time.time()
		elapsed_time = now - self.last_timestamp
		self.last_timestamp = now


		self.raw_power_sum = self.raw_power_sum + self.last_raw_power
		# self.raw_power_sum = self.raw_power_sum + self.last_raw_power*float(elapsed_time)
		if raw_current_value:
			self.last_raw_power = float(self.cur_val_list[VOL])*float(raw_current_value)
			# self.last_raw_power = float(raw_current_value)


		# if raw_current_value:
		# 	print "raw_current_value: " + str(raw_current_value) + " raw_power_sum: " + str(self.raw_power_sum)
		# else:
		# 	print "raw_current_value: " + "none" + " raw_power_sum: " + str(self.raw_power_sum)

	def write_log(self):
		self.log_val_list[TIMESTAMP] = time.time() - self.start_time
		self.log_val_list[ENERGY] = self.convert_to_amp(self.raw_power_sum)
		# self.cur_val_list[ENERGY] = self.raw_power_sum
		

		self.log_val_list = [str(x) for x in self.log_val_list]

		log_str = ",".join([self.log_val_list[i] for i in LOG_ITEMS])
		# log_str = ",".join(self.cur_val_list)

		print "cur", self.cur_val_list
		print "log", self.log_val_list
		self.log_file.write("%s\n" % log_str)

		self.reset_raw_power_sum()

	def reset_raw_power_sum(self):
		self.raw_power_sum = 0
		self.last_raw_power = 0

	def set_body_velocity(self, vx, vy, vz=0):
		self.cur_val_list[TIMESTAMP] = time.time() - self.start_time

		self.cur_val_list[ACT_VX] = vx
		self.cur_val_list[ACT_VY] = vy
		self.cur_val_list[ACT_VZ] = vz

		self.log_val_list = list(self.cur_val_list)

		pos_target = PositionTarget()
		pos_target.coordinate_frame = PositionTarget.FRAME_BODY_NED # FRAME_BODY_NED
		pos_target.type_mask = SETPOINT_FLAG

		vel = Vector3()
		vel.x = vx
		vel.y = vy
		vel.z = vz

		pos_target.velocity = vel

		self.set_vel_pub.publish(pos_target)

	def set_empty(self):
		pos_target = PositionTarget()
		pos_target.coordinate_frame = PositionTarget.FRAME_BODY_NED # FRAME_BODY_NED
		pos_target.type_mask = SETPOINT_IGNORE_ALL_FLAG

		self.set_vel_pub.publish(pos_target)

	def pub_local_position(self, x, y, z):


		pose = PoseStamped()
		pose.pose.position.x = x
		pose.pose.position.y = y
		pose.pose.position.z = z

		self.local_pos_pub.publish(pose)

	def takeoff(self):
		self.pub_local_position(0, 0, 2)

	def set_mode(self, mode):
		mode = SetMode()
		mode.base_mode = 0
		mode.custom_mode = "OFFBOARD"

	def print_status(self):
		print "velocity", self.velocity.twist.linear.x, self.velocity.twist.linear.y, self.velocity.twist.linear.z

	def arm(self):
		self.arming_client(True)

	def disarm(self):
		self.arming_client(False)

	def convert_to_amp(self, raw_value):
		return ((raw_value / 1024.0)*5000 - 2500) / 100
