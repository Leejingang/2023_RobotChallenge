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
import rospy
from geometry_msgs.msg import Point



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
	MoveBCommand = 12 #detect -> grasp -> center -> place -> center
	MoveCenter = 13
	MoveGraspDetect = 14
	MovePlaseDetect = 15
	MoveUpPosition = 16

class Camera(object):
	def __init__(self, topic="/topic"):
		self.sub = rospy.Subscriber(topic, Point, callback=self.callback)

		self.point = None

	def callback(self, msg):
		self.point = [round(msg.x / 1000, 5), round(msg.y / 1000, 5), round(msg.z / 1000, 8)]
			

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
		print(" 13 :  MoveCenter Command")
		print(" 14 :  MoveGraspDetect Command")
		print(" 15 :  MovePlaseDetect Command")
		print(" 16 :  MoveUpPosition Command")


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
   
		elif key_value == '13':
			cmd = Cmd.MoveCenter

		elif key_value == '14':
			cmd = Cmd.MoveGraspDetect	

		elif key_value == '15':
			cmd = Cmd.MovePlaseDetect	
   
		elif key_value == '16':
			cmd = Cmd.MoveUpPosition
   
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

def pointcallback(msg):
    global camera_points
    
    x = None
    x = msg.x
    y = msg.y
    z = msg.z
    
    if x == None:
        camera_points = [0, 0, 0]
    
    else :
        camera_points = (x, y, z)	
    
if __name__ == '__main__':
    
	cali = [0.02, 0.05, 0.0]
    
	if len(sys.argv) != 4:
		print("\n\nPlease check the input arguments!!\n\n")
		exit(1)

	setLibPath(f'{os.getcwd()}/../ketirobotsdk/ketirobotsdk/librobotsdkv2.so')

	robot_ip = sys.argv[1]
	gripper_ip = sys.argv[2]
	gripper_port = sys.argv[3]

	SetRobotConf(RB10, robot_ip,5000)
	robot_connected = RobotConnect()
 
	
 
	
	rospy.init_node('move_robot')  


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

	SetVelocity(10)
 
 
	camera = Camera(topic="/topic")

	try:
		while robot_connected is True and not rospy.is_shutdown():
			# if state == State.Wait and camera.point is not None:
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
					print([(current_T_matrix[3] - camera.point[0]), (current_T_matrix[7] + camera.point[1] + 0.4), (camera.point[2] - current_T_matrix[11])])

					moveL_command([[1,0,0],[0,0,1],[0,1,0]], [(current_T_matrix[3] - camera.point[0] + cali[0]), (current_T_matrix[7] + camera.point[1] + cali[1]), (camera.point[2] - current_T_matrix[11]) - 0.08])

					cmd = 0
     
				elif cmd == Cmd.MoveBCommand:
					moveB_command(5,
                   [[[1,0,0],[0,0,1],[0,1,0]],
                    [[1,0,0],[0,0,1],[0,1,0]],
                    [[1,0,0],[0,0,1],[0,1,0]],
                    [[1,0,0],[0,0,1],[0,1,0]],
                    [[1,0,0],[0,0,1],[0,1,0]]],
                   
                   [[0.2187105119228363, -0.8066880106925964, 0.5406314730644226],
                    [(current_T_matrix[3] - camera.point[0] + cali[0]), (current_T_matrix[7] + camera.point[1] + cali[1]), 0.28287017047405243],
                    [0.025800511240959167, -0.6534833908081055, 0.5406314730644226],
                    [-0.15508690476417542, -0.6533833146095276, 0.021425902843475342],
                    [0.025800511240959167, -0.6534833908081055, 0.5406314730644226]
                    ])
					cmd = 0
     
				elif cmd == Cmd.MoveCenter:
					moveL_command([[1,0,0],[0,0,1],[0,1,0]], [0.025800511240959167, -0.6534833908081055, 0.5406314730644226])

					cmd = 0
				elif cmd == Cmd.MoveGraspDetect:
					moveL_command([[1,0,0],[0,0,1],[0,1,0]], [0.2187105119228363, -0.8066880106925964, 0.5406314730644226])

					cmd = 0
     
				elif cmd == Cmd.MovePlaseDetect:
					moveL_command([[1,0,0],[0,0,1],[0,1,0]], [-0.1326087862253189, -0.6559941172599792, 0.5406314730644226])

					cmd = 0
     
     
				elif cmd == Cmd.MoveUpPosition:
					moveL_command([[1,0,0],[0,0,1],[0,1,0]], [current_T_matrix[3], current_T_matrix[7], 0.5])

					cmd = 0

			sleep(1)
   
   
	except KeyboardInterrupt:
		RobotDisconnect()

	print("finish")