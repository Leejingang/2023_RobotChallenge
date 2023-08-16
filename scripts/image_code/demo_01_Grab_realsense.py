import numpy as np
import cv2
import json
from ketisdk.sensor.realsense_sensor import RSSensor
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD

if __name__=='__main__':
    cv2.namedWindow('result')

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
        rgb_data = rgb_data[:, :, ::-1] #RGB -> BGR
        rgbd = RGBD(rgb_data, depth_data, depth_min=400,depth_max=600)
        if ws_pts is not None:
            rgbd.set_workspace(pts=ws_pts)
        rgbimg = rgbd.draw_workspace(rgbd.rgb)
        cv2.imshow("result", rgbimg)
        key = cv2.waitKey(100)
        if key == ord('q'):
            print("set workspace")
            break