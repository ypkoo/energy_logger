#!/usr/bin/env python
#encoding = utf-8



import random
import argparse
from itertools import cycle
import time
from aero_energy_logger import AeroEnergyLogger
import sys, signal
import rospy
import roslib


def signal_handler(signal, frame):
	print("\nprogram exiting gracefully")
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)




if __name__ == '__main__':


	logger = AeroEnergyLogger('empty')
	time.sleep(3)
	rate = rospy.Rate(50.0)
	prev_state = logger.cur_state
	
	random.seed(time.time())

	# wait for FCU connection
	print "waiting for FCU connection..."
	while not logger.cur_state.connected:
		rate.sleep()	

	print "FCU connected!"

	last_request = rospy.get_rostime()

	for i in range(10):

		logger.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
		logger.set_empty()
		rate.sleep()


	while not rospy.is_shutdown():

		try:
			print "current mode: ", logger.cur_state.mode
			print "armed: ", logger.cur_state.armed
			logger.set_empty()
			now = rospy.get_rostime()
			if logger.cur_state.mode != "OFFBOARD" or (now - last_request > rospy.Duration(3.)):
				print "request offboard"
				logger.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
				logger.set_empty()
				last_request = now 
			# if not logger.cur_state.armed and (now - last_request > rospy.Duration(3.)):
			# 	logger.arming_client(True)
			# 	last_request = now 
				
			if logger.cur_state.mode == "OFFBOARD":
				logger.set_empty()
				# vx = random.randrange(-30,30)/10.0
				# vy = random.randrange(-30,30)/10.0
				# logger.set_body_velocity(vx, vy, 0)

		except KeyboardInterrupt:
			print('interrupted!')
			break
		except:
			continue

		time.sleep(1)



	logger.set_body_velocity(0.0, 0.0, 0.0)
	sys.exit()
