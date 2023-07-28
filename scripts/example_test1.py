#!/usr/bin/env python3

import sys
sys.path.append("../ketirobotsdk")
from ketirobotsdk.sdk import *
from time import *
import threading
import math
sys.path.append("../include/gripper")
from keti_zimmer_gripper import KetiZimmer
from utils import *




class State:
	Wait = 1
	Moving = 2

class Cmd:
	RecvRobotState = 1
	RecvGripperWidth = 2
	RobotMoveJ = 3
	RobotMoveL = 4
	RobotMoveB = 5
	GripperMoveGrip = 6
	GripperMoveRelease = 7
	MoveLCommand = 11
	MoveBCommand = 12

robot_connected = False
state = 0
cmd = 0
current_joint = []
current_T_matrix = []
gripper = KetiZimmer()

def key_input_func():
	global cmd
	key_value = 0

	while robot_connected is True:
		print("\n Enter character and press \"Enter\"")
		print(" 1 : Receive robot current state")
		print(" 2 : Receive gripper current width")
		print(" 3 : Robot move joint motion")
		print(" 4 : Robot move Cartesian motion")
		print(" 5 : Robot move Cartesian motion with blend")
		print(" 6 : Gripper move(grip)")
		print(" 7 : Gripper move(release)")
		print("######################")
		print(" 11 : MoveL Motion Command")
		print(" 12 : MoveB Motion Command")

		key_value = input()

		if key_value == '1':
			cmd = Cmd.RecvRobotState
		elif key_value == '2':
			cmd = Cmd.RecvGripperWidth
		elif key_value == '3':
			cmd = Cmd.RobotMoveJ
		elif key_value == '4':
			cmd = Cmd.RobotMoveL
		elif key_value == '5':
			cmd = Cmd.RobotMoveB
		elif key_value == '6':
			cmd = Cmd.GripperMoveGrip
		elif key_value == '7':
			cmd = Cmd.GripperMoveRelease
   
		elif key_value == '11':
			cmd = Cmd.MoveLCommand
		elif key_value == '12':
			cmd = Cmd.MoveBCommand

		while cmd != 0:
			sleep(0.001)

def data_update_func():
	global robot_connected, state, cmd, current_joint, current_T_matrix
	while robot_connected is True:
		robotInfor = RobotInfo()

		current_joint = [robotInfor.Jnt[0], robotInfor.Jnt[1], robotInfor.Jnt[2], robotInfor.Jnt[3], robotInfor.Jnt[4], robotInfor.Jnt[5]]

		current_T_matrix = [robotInfor.Mat[0], robotInfor.Mat[1], robotInfor.Mat[2], robotInfor.Mat[3],
                      robotInfor.Mat[4], robotInfor.Mat[5], robotInfor.Mat[6], robotInfor.Mat[7],
                      robotInfor.Mat[8], robotInfor.Mat[9], robotInfor.Mat[10], robotInfor.Mat[11],
                      robotInfor.Mat[12], robotInfor.Mat[13], robotInfor.Mat[14], robotInfor.Mat[15]]

		if robotInfor.State == 2:
			state = State.Moving
			cmd = 0
		elif robotInfor.State == 1:
			state = State.Wait

		sleep(0.01)



if __name__ == '__main__':
	if len(sys.argv) != 4:
		print("\n\nPlease check the input arguments!!\n\n")
		exit(1)

	setLibPath(f'{os.getcwd()}/../ketirobotsdk/ketirobotsdk/librobotsdkv2.so')

	robot_ip = sys.argv[1]
	gripper_ip = sys.argv[2]
	gripper_port = sys.argv[3]

	SetRobotConf(RB10, robot_ip,5000)
	robot_connected = RobotConnect()
	
	if gripper_port == 502:
		gripper.connect(gripper_ip, gripper_port)
		gripper_connected = gripper.isConnected()
		print("wait...")
		if gripper_connected is True:
			gripper.gripper_init()
  
	key_input_thraed = threading.Thread(target=key_input_func, daemon=True)
	key_input_thraed.start()
	data_update_thread = threading.Thread(target=data_update_func, daemon=True)
	data_update_thread.start()


	cnt_joint = 0
	cnt_pose = 0

	SetVelocity(5)

	try:
		while robot_connected is True:
			if state == State.Wait:
				if cmd == Cmd.RecvRobotState:
					robotInfor = RobotInfo()
					print("current_state : {0}".format(robotInfor.State))
					print("current_joint : {0}".format(current_joint))
					print("current_T_matrix : ")
					print(current_T_matrix[0:4])
					print(current_T_matrix[4:8])
					print(current_T_matrix[8:12])
					print(current_T_matrix[12:16])
					cmd = 0
				elif cmd == Cmd.RecvGripperWidth:
					print("current width : {0}".format(gripper.grip_get_pos()))
					cmd = 0
				
				elif cmd == Cmd.GripperMoveGrip:
					if gripper_port == 502:
						gripper.gripper_grip()
					cmd = 0
				elif cmd == Cmd.GripperMoveRelease:
					if gripper_port == 502:
						gripper.gripper_release()
					cmd = 0
     
				elif cmd == Cmd.MoveLCommand:
					moveL_command([[1,0,0],[0,0,1],[0,1,0]], [(-0.20827999711036682 - 0.019257799873702513), (-0.733189046382904 - 0.04160633109030661), (0.5071385502815247)])
					cmd = 0
     
				elif cmd == Cmd.MoveBCommand:
					moveB_command(2, [[[1,0,0],[0,1,0],[0,0,1]], [[1,0,0],[0,1,0],[0,0,1]]], [[-0.2, -0.4, 0.2], [-0.6, -0.1, 0.3]])
					cmd = 0
        

			sleep(0.001)
   
   
	except KeyboardInterrupt:
		RobotDisconnect()

	print("finish")