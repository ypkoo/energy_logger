#!/usr/bin/env python
#encoding = utf-8



import random
import argparse
from itertools import cycle
import time
from aero_energy_logger import *
import rospy
import sys, signal


def signal_handler(signal, frame):
	print("\nprogram exiting gracefully")
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)




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
	rate = rospy.Rate(50.0)
	prev_state = logger.cur_state
	
	random.seed(time.time())

	# wait for FCU connection
	print "waiting for FCU connection..."
	while not logger.cur_state.connected:
		rate.sleep()	

	print "FCU connected!"

	last_request = rospy.get_rostime()


	while not rospy.is_shutdown():
		line = logger.ser.readline().strip()

		
		# print "current mode: ", logger.cur_state.mode
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
		
		if line == "#":
			

			try:
				# if logger.cur_state.mode == "OFFBOARD":
				logger.update_raw_power_sum()
				print "write log"
				logger.write_log()
				print "give action"
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
			# except:
			# 	print "exception occured #"
		else:
			try:
				# if logger.cur_state.mode == "OFFBOARD":
				# 	logger.update_current(int(line))
				logger.update_raw_power_sum(int(line))
			except KeyboardInterrupt:
				print('interrupted!')
				break
			# except:
			# 	print "exception occured raw"


	logger.set_body_velocity(0.0, 0.0, 0.0)
	sys.exit()
