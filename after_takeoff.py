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
FIELD_NUM = 22
TIMESTAMP, VEL_X, VEL_Y, VEL_Z, ACC_X, ACC_Y, ACC_Z, ROLL, PITCH, YAW, RC0, RC1, RC2, RC3, VOL, CUR_RAW, CUR, POWER, ACT_VX, ACT_VY, ACT_VZ, MODE = range(FIELD_NUM)

SETPOINT_FLAG = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
				PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
				PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE


class AeroEnergyLogger(object):

	def __init__(self, filename=None):
		self.ser = serial.Serial(SERIAL_PORT, 9600)

		if filename:
			self.log_file = open("logs/%s_log.csv" % filename, 'w')
		else:
			self.log_file = open("logs/%s_log.csv" % datetime.datetime.now().strftime('%m%d%H%M%S'), 'w')

		self.log_file.write("timestamp,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,roll,pitch,yaw,rc0,rc1,rc2,rc3,vol,cur_raw,cur,power,act_vx,act_vy,act_vz,mode\n")

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

		print "initialize subscribers..."
		self.init_subscribers()
		print "initialize services..."
		self.init_services()
		print "initialize publishers..."
		self.init_publishers()

		self.start_time = time.time()

	def init_subscribers(self):
		rospy.Subscriber(mavros.get_topic('state'), State, self.state_callback)
		rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.velocity_callback)
		rospy.Subscriber("/mavros/battery", BatteryState, self.battery_state_callback)
		rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
		rospy.Subscriber("/mavros/manual_control/control", ManualControl, self.manual_control_callback)
		rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_in_callback)
		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)

	def init_services(self):
		rospy.wait_for_service(mavros.get_topic('cmd', 'arming'))
		rospy.wait_for_service(mavros.get_topic('set_mode'))
		rospy.wait_for_service(mavros.get_topic('cmd', 'takeoff'))
		self.arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
		self.set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
		self.takeoff_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'takeoff'), CommandTOL)

	def init_publishers(self):
		self.set_vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

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

	def set_body_velocity(self, vx, vy, vz=0):
		self.cur_val_list[ACT_VX] = vx
		self.cur_val_list[ACT_VY] = vy
		self.cur_val_list[ACT_VZ] = vz

		pos_target = PositionTarget()
		pos_target.coordinate_frame = PositionTarget.FRAME_BODY_NED # FRAME_BODY_NED
		pos_target.type_mask = SETPOINT_FLAG

		vel = Vector3()
		vel.x = vx
		vel.y = vy
		vel.z = vz

		pos_target.velocity = vel

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



if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument("-f", "--filename", help="filename to save the log file")
	parser.add_argument("-a", "--action", help="action generation scheme")
	args = parser.parse_args()

	vx_list = []
	vy_list = []

	if args.filename:
		filename = args.filename
	if args.action:
		action_scheme = "FILE"
		action_file = args.action
		f = open("actions/"+action_file)
		lines = f.readlines()
		
		for line in lines:
			vx, vy = line.split()
			vx_list.append(vx)
			vy_list.append(vy)
	else:
		action_scheme = "RANDOM"

	print vx_list, vy_list
	vx_cycle = cycle(vx_list)
	vy_cycle = cycle(vy_list)

	logger = AeroEnergyLogger(filename)
	time.sleep(3)
	rate = rospy.Rate(20.0)
	prev_state = logger.cur_state
	
	random.seed(time.time())

	# wait for FCU connection
	print "waiting for FCU connection..."
	while not logger.cur_state.connected:
		rate.sleep()


	last_request = rospy.get_rostime()

	while not rospy.is_shutdown():
		line = logger.ser.readline().strip()

		

		if line == "#":
			print "current mode: ", logger.cur_state.mode
			now = rospy.get_rostime()
			if logger.cur_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(3.)):
				logger.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
				last_request = now 
			else:
				if not logger.cur_state.armed and (now - last_request > rospy.Duration(3.)):
					logger.arming_client(True)
					last_request = now 

			if prev_state.armed != logger.cur_state.armed:
				rospy.loginfo("Vehicle armed: %r" % logger.cur_state.armed)
			if prev_state.mode != logger.cur_state.mode: 
				rospy.loginfo("Current mode: %s" % logger.cur_state.mode)
			prev_state = logger.cur_state

			try:
				if logger.cur_state.mode == "OFFBOARD":
					if action_scheme == "FILE":
						vx = float(next(vx_cycle))
						vy = float(next(vy_cycle))
					elif action_scheme == "RANDOM":
						vx = random.randrange(-30,30)/10.0
						vy = random.randrange(-30,30)/10.0
					
					logger.set_body_velocity(vx, vy, 0.0)

			except KeyboardInterrupt:
				print('interrupted!')
				break
			except:
				continue
		else:
			try:
				# if logger.cur_state.mode == "OFFBOARD":
				# 	logger.update_current(int(line))
				logger.update_current(int(line))
				logger.cur_val_list[ACT_VX] = '-'
				logger.cur_val_list[ACT_VY] = '-'
				logger.cur_val_list[ACT_VZ] = '-'
			except KeyboardInterrupt:
				print('interrupted!')
				break
			except:
				continue


	logger.set_body_velocity(0.0, 0.0, 0.0)
	sys.exit()
