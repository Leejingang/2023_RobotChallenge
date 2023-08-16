from signal import signal, SIGPIPE, SIG_DFL
# Ignore SIG_PIPE and don't throw exceptions on it... (http://docs.python.org/library/signal.html)
signal(SIGPIPE, SIG_DFL)
import socket
import cv2
import time
import datetime
import numpy
import numpy as np
import copy
import base64
import time

import numpy as np
import json
from ketisdk.sensor.realsense_sensor import RSSensor
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
from geometry_msgs.msg import Point
import rospy

class GraspDetector:
    def __init__(self, ip, port):

        self.wts = np.array([[387, 198], [968, 202], [970, 538], [374, 536]]).flatten()
        self.threshold = 0.8
        self.w_range=np.array([[30, 40], [10, 15], [80,90]]).flatten()
        self.min_value = 70
        self.max_value = 80
        self.npose = 5000
        self.top_n = 200
        self.angle_step = 10
        self.set_param_new()

        self.TCP_SERVER_IP = ip
        self.TCP_SERVER_PORT = port
        self.connectCount = 0
        self.rgb_data=[]
        self.connectServer()
    def set_parameter(self,param):
        self.param=copy.deepcopy(param)

    def set_param_new(self):
        param=int(self.wts.shape[0] / 2)
        param = np.append(param, self.wts)
        param=np.append(param,self.threshold)
        param=np.append(param,int(self.w_range.shape[0] / 2))
        param=np.append(param,self.w_range)
        param=np.append(param,self.npose)
        param=np.append(param,self.top_n)
        param=np.append(param,self.angle_step)
        self.param=copy.deepcopy(param)

    def set_param(self):
        param=copy.deepcopy(self.wts)
        param=np.append(param,self.threshold)
        param=np.append(param,self.min_value)
        param=np.append(param,self.max_value)
        param=np.append(param,self.npose)
        param=np.append(param,self.top_n)
        param=np.append(param,self.angle_step)
        self.param=copy.deepcopy(param)

    def set_workspace(self,wt_array):
        self.wts=wt_array.flatten()

    def set_threshold(self,threshold):
        self.threshold=threshold

    def set_width_range(self,w_range):
        self.w_range=w_range

    def set_npose(self,npose):
        self.npose=npose

    def set_top_n(self,top_n):
        self.top_n=top_n

    def set_angle_set(self,angle_step):
        self.angle_step=angle_step

    def get_data(self,rgb_data, depth):
        start_time = time.perf_counter()


        self.sock.send("/grab".encode('utf-8'))
        # send param
        stringData = base64.b64encode(np.array(self.param))
        length = str(len(stringData))
        # image data
        self.sock.sendall(length.encode('utf-8').ljust(64))
        self.sock.send(stringData)

        # send image
        send_data = numpy.dstack((rgb_data, depth))
        stime = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        stringData = base64.b64encode(send_data)
        length = str(len(stringData))

        # image data
        self.sock.sendall(length.encode('utf-8').ljust(64))
        self.sock.send(stringData)
        self.sock.send(stime.encode('utf-8').ljust(64))

        # image size
        stringData = base64.b64encode(np.array(rgb_data.shape))
        length = str(len(stringData))
        self.sock.sendall(length.encode('utf-8').ljust(64))
        self.sock.send(stringData)

        #recv
        #grips
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        grips = np.frombuffer(base64.b64decode(rec_data), float)
        grips=grips.reshape([int(grips.size / 11), 11])
        detected = {'grip': grips}

        # image
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        img_data = np.frombuffer(base64.b64decode(rec_data), np.uint8)
        # size
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        img_size = np.frombuffer(base64.b64decode(rec_data), int)

        result_image = img_data.reshape(np.append(img_size[0:2], [3]))
        detected.update({'im': result_image})

        # best_index
        rec_data = self.sock.recv(64)
        best_index = rec_data.decode('utf-8')
        detected.update({'best_ind': best_index})

        # best_n_inds
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        best_n_inds = np.frombuffer(base64.b64decode(rec_data), int)
        detected.update({'best_n_inds': best_n_inds})

        # best_grip
        rec_data = self.sock.recv(64)
        length1 = rec_data.decode('utf-8')
        rec_data = self.recvall(self.sock, int(length1))
        best_grip = np.frombuffer(base64.b64decode(rec_data), float)
        detected.update({'best': best_grip})
        
        print("best grip is :  ", best_grip)

        rec_data = self.sock.recv(64)
        print(rec_data)
        end_time = time.process_time()
        end_time = time.perf_counter()
        print(f"time elapsed : {int(round((end_time - start_time) * 1000))}ms")
        return detected

    def connectServer(self):
        print("connectServer")
        try:
            self.sock = socket.socket()
            self.sock.connect((self.TCP_SERVER_IP, self.TCP_SERVER_PORT))
            print(
                u'Client socket is connected with Server socket [ TCP_SERVER_IP: ' + self.TCP_SERVER_IP + ', TCP_SERVER_PORT: ' + str(
                    self.TCP_SERVER_PORT) + ' ]')
            self.connectCount = 0

        except Exception as e:
            print(e)
            self.connectCount += 1
            if self.connectCount == 10:
                print(u'Connect fail %d times. exit program' % (self.connectCount))
                sys.exit()
            print(u'%d times try to connect with server' % (self.connectCount))
            self.connectServer()

    def recvall(self, sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf

if __name__=='__main__':
    import sys

    for arg in sys.argv:
        print(arg)
        
    sensor = RSSensor()
    sensor.start()

    # rgb_data = cv2.imread("/home/lee/Desktop/robotchallenge/robochalle_demo/data/Grasp_RGB.png")
    # depth_data = cv2.imread("/home/lee/Desktop/robotchallenge/robochalle_demo/data/Grasp_depth.png", -1)
    
    
    print("read image")
    TCP_IP = '192.168.0.200'
    TCP_PORT = 5000
    GraspPointDetector=GraspDetector(TCP_IP, TCP_PORT)
    print("1")
    GraspPointDetector.set_workspace(np.array([[387, 198], [968, 202], [970, 538], [374, 536]]).flatten())
    print("2")
    GraspPointDetector.set_threshold(0.8)
    GraspPointDetector.set_width_range(np.array([[30, 40], [10, 15], [80,90]]).flatten())
    GraspPointDetector.set_npose(500)
    GraspPointDetector.set_top_n(50)
    GraspPointDetector.set_angle_set(10)
    count=0
    try:
        print("input msg1")
        msg = input('')
        if msg=="1":
            while 1:
                rgb_data, depth_data = sensor.get_data()  # realsense data

                count+=1
                cur_rgb_data=copy.deepcopy(rgb_data)

                cur_depth_data=copy.deepcopy(depth_data)

                disp_text=str(count)

                cv2.putText(cur_rgb_data, disp_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)

                # GraspPointDetector.set_param()
                GraspPointDetector.set_param_new()

                ret=GraspPointDetector.get_data(cur_rgb_data, cur_depth_data)

                print(count)
                cv2.imshow("rgb",ret["im"])
                # time.sleep()
                cv2.waitKey(1000)
        else:
            while 1:
                count += 1
                print("input msg2")
                msg = input('')
                if msg=="s":
                    
                    rgb_data, depth_data = sensor.get_data()  # realsense data

                    cur_rgb_data = copy.deepcopy(rgb_data)
                    cur_depth_data = copy.deepcopy(depth_data)
                    disp_text = str(count)
                    cv2.putText(cur_rgb_data, disp_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)

                    GraspPointDetector.set_param()
                    ret = GraspPointDetector.get_data(cur_rgb_data, cur_depth_data)
                    print(count)
                    cv2.imshow("rgb", ret["im"])
                    cv2.waitKey(10)
    except Exception as e:
        print(e)
        cv2.destroyAllWindows()
