#!/usr/bin/env python
#encoding = utf-8



import rospy
import roslib

import time, serial, datetime

from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu, BatteryState
from mavros_msgs.msg import *
from tf.transformations import euler_from_quaternion

SERIAL_PORT = "/dev/ttyACM0"
FIELD_NUM = 17
TIMESTAMP, VEL_X, VEL_Y, VEL_Z, ACC_X, ACC_Y, ACC_Z, ROLL, PITCH, YAW, RC0, RC1, RC2, RC3, VOL, CUR, POWER = range(17)


class AeroEnergyLogger(object):

	def __init__(self):
		self.ser = serial.Serial(SERIAL_PORT, 9600)

		self.log_file = open("%s_log.csv" % datetime.datetime.now().strftime('%m%d%H%M%S'), 'w')
		self.log_file.write("timestamp, raw_current, velocity_x\n")

		rospy.init_node('aero_energy_logger')

		self.velocity = TwistStamped()
		self.rc_in = RCIn()
		self.imu = Imu()
		self.manual_control = ManualControl()
		self.battery_state = BatteryState()
		self.pose = PoseStamped()
		self.current = None

		self.init_subscribers()

		self.start_time = time.time()

	def init_subscribers(self):
		rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.velocity_callback)
		rospy.Subscriber("/mavros/battery", BatteryState, self.battery_state_callback)
		rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
		rospy.Subscriber("/mavros/manual_control/control", ManualControl, self.manual_control_callback)
		rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_in_callback)
		rospy.Subscriber("/mavros/local_postion/pose", PoseStamped, self.pose_callback)


	def velocity_callback(self, velocity):
		self.velocity = velocity

	def battery_state_callback(self, battery_state):
		self.battery_state = battery_state

	def imu_callback(self, imu):
		self.imu = imu

	def manual_control_callback(self, manual_control):
		self.manual_control = manual_control

	def rc_in_callback(self, rc_in):
		self.rc_in = rc_in

	def pose_callback(self, pose):
		self.pose = pose

	def update_current(self, raw_value):
		vol = (raw_value / 1024.0) * 5000
		amps = ((vol - 2500) / 100)
		self.current = amps

	def write_log(self):
		self.log_file.write("%s\n" % self.get_log_string())

	def get_log_string(self):
		s = [0] * FIELD_NUM
		s[TIMESTAMP] = time.time() - self.start_time
		s[VEL_X] = self.velocity.twist.linear.x
		s[VEL_Y] = self.velocity.twist.linear.y
		s[VEL_Z] = self.velocity.twist.linear.z
		s[ACC_Z] = self.imu.linear_acceleration.x
		s[ACC_Y] = self.imu.linear_acceleration.y
		s[ACC_Z] = self.imu.linear_acceleration.z

		orientation = self.pose.pose.orientation
		qs = [orientation.x, orientation.y, orientation.z, orientation.w]
		roll, pitch, yaw = euler_from_quaternion(qs)

		s[ROLL] = roll
		s[PITCH] = pitch
		s[YAW] = yaw

		s[RC0] = self.rc_in.channels[0]
		s[RC1] = self.rc_in.channels[1]
		s[RC2] = self.rc_in.channels[2]
		s[RC3] = self.rc_in.channels[3]

		s[VOL] = self.battery_state.voltage
		s[CUR] = self.current
		s[POWER] = s[VOL] * s[CUR]

		s = [str(x) for x in s]

		return ",".join(s)

if __name__ == '__main__':
	logger = AeroEnergyLogger()

	while True:
		current_raw = logger.ser.readline().strip()
		logger.update_current(float(current_raw))
		logger.write_log()


