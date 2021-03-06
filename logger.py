#!/usr/bin/env python
#encoding = utf-8



import rospy
import roslib

import time, serial, datetime

from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu, BatteryState
from mavros_msgs.msg import *
from tf.transformations import euler_from_quaternion

import sys, signal
def signal_handler(signal, frame):
	print("\nprogram exiting gracefully")
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

SERIAL_PORT = "/dev/ttyACM0"
FIELD_NUM = 18
TIMESTAMP, VEL_X, VEL_Y, VEL_Z, ACC_X, ACC_Y, ACC_Z, ROLL, PITCH, YAW, RC0, RC1, RC2, RC3, VOL, CUR_RAW, CUR, POWER = range(FIELD_NUM)


class AeroEnergyLogger(object):

	def __init__(self):
		self.ser = serial.Serial(SERIAL_PORT, 9600)

		self.log_file = open("%s_log.csv" % datetime.datetime.now().strftime('%m%d%H%M%S'), 'w')
		self.log_file.write("timestamp,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,roll,pitch,yaw,rc0,rc1,rc2,rc3,vol,cur_raw,cur,power\n")

		rospy.init_node('aero_energy_logger')

		self.velocity = TwistStamped()
		self.rc_in = RCIn()
		self.imu = Imu()
		self.manual_control = ManualControl()
		self.battery_state = BatteryState()
		self.pose = PoseStamped()

		self.cur_val_list = [0] * FIELD_NUM

		self.init_subscribers()
		self.init_publishers()

		self.start_time = time.time()

	def init_subscribers(self):
		rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.velocity_callback)
		rospy.Subscriber("/mavros/battery", BatteryState, self.battery_state_callback)
		rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
		rospy.Subscriber("/mavros/manual_control/control", ManualControl, self.manual_control_callback)
		rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_in_callback)
		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)

	def init_publishers(self):
		self.set_vel_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)


	def velocity_callback(self, velocity):
		self.velocity = velocity

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
		print self.pose

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

	def pub_setpoint_velocity(vx, vy, vz=0):

		pos_target = PositionTarget()
		pos_target.coordinate_frame = 8 # FRAME_BODY_NED
		pos_target.target_mask = 4039

		vel = Vector3()
		vel.x = vx
		vel.y = vy
		vel.z = vz

		pos_target.velocity = vel

		self.set_vel_publisher.publish(pos_target)


if __name__ == '__main__':
	logger = AeroEnergyLogger()
	time.sleep(3)


	while True:
		current_raw = logger.ser.readline().strip()
		try:
			logger.update_current(int(current_raw))
		except KeyboardInterrupt:
			print('interrupted!')
			break
		except:
			continue
		# logger.write_log()

	
	sys.exit()