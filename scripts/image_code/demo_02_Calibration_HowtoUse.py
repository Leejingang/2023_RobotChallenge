import sys
try:
    import cv2
except:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    import cv2

from calibration import HandEyeCalib

import numpy as np
import json
from ketisdk.sensor.realsense_sensor import RSSensor
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
from geometry_msgs.msg import Point
import rospy

def mouse_event(event, x, y, flags, param):
    global gx, gy
    global display_img

    if event == cv2.EVENT_FLAG_LBUTTON:
        Xc = x
        Yc = y
        Zc=depth_data[y,x]
        print ("2D points",Xc,Yc)
        print ("depth",Zc)
        point_x=(Xc-ppx)/fx*Zc
        point_y=(Yc-ppy)/fy*Zc
        point_z=float(Zc)
        print("robot 3d point trans",point_x,point_y,point_z)
        robot_x_calib = 0
        robot_y_calib = 0
        robot_z_calib = 0
        print("robot trans, Calibration 적용", robot_x_calib,robot_y_calib,robot_z_calib)
        
        msg = Point()
        msg.x , msg.y, msg.z = point_x, point_y, point_z
        
        pointpub = rospy.Publisher('topic', Point, queue_size=1)
        pointpub.publish(msg)
        
        

if __name__=='__main__':
    global  ppx, ppy, fx, fy
    global Calib
    pointpub = rospy.Publisher('topic', Point, queue_size=1)
    cv2.namedWindow('result')
    cv2.setMouseCallback('result', mouse_event)

    rospy.init_node("camera_point_pub")
    msg = Point()
    
    HandEyeCal=HandEyeCalib()
    Calib=HandEyeCal.load_calibration(filename="../calibration/calibration2.json")
    print(Calib)

    sensor_enable=True
    if sensor_enable:
        sensor = RSSensor()
        sensor.start()
        ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy

    else:
        ppx, ppy, fx, fy = np.load("intr_param.npy")
    # set workspace
    try:
        with open("../configs/workspace/config_roi.json", "r") as st_json:
            ws_pts = json.load(st_json)
    except:
            ws_pts = [(152, 85), (1081, 84), (1036, 663), (870, 659), (768, 557), (716, 577), (753, 675), (235, 667)]

    while True:
        if sensor_enable:
            rgb_data, depth_data = sensor.get_data()  # realsense data
        else:
            rgb_data = cv2.imread("../data/Grasp_RGB.png")
            depth_data = cv2.imread("../data/Grasp_depth.png", -1)
        rgbd = RGBD(rgb_data, depth_data, depth_min=400,depth_max=600)
        if ws_pts is not None:
            rgbd.set_workspace(pts=ws_pts)
        rgbimg = rgbd.draw_workspace(rgbd.rgb)
        # msg.x, msg.y, msg.z = xc, yc, zc
        # pointpub.publish(msg)
        cv2.circle(rgbimg, (1280//2, 720//2), 1, (0,0,200))
        cv2.imshow("result", rgbimg)
        
        key = cv2.waitKey(100)

        if key == ord('q'):
            print("set workspace")
            break
