import sys
import numpy as np
import json
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
from ketisdk.import_basic_utils import *
import os
try:
    import cv2
except:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    import cv2
def print_grips(grips):
    print("X,Y,Z", grips[0:3], end=" / ")
    print("box size", grips[3:5], end=" / ")
    print("angle", grips[5], end=" / ")
    print("left point", grips[6:8], end=" / ")
    print("right point", grips[8:10], end=" / ")
    print("score", grips[10])

def grasp2TiltedRect(xc, yc, w, h, angle):
    rx, ry = w/2, h/2
    rect = []
    for dd in [(-rx, -ry), (rx, -ry), (rx, ry), (-rx, ry)]:
        dx, dy = ProcUtils().rotateXY(dd[0], dd[1], angle)
        rect.append((int(xc + dx), int(yc + dy)))
    return rect
def get_high_score_inds(Score, thresh):
    return np.where(Score>thresh)[0].flatten().tolist()
def get_low_score_inds(Score, thresh):
    return np.where(Score<=thresh)[0].flatten().tolist()

def show_grips(rgbd, Grip, best_ind, best_n_inds, disp_mode='rgb', score_thresh=0.7,show_steps=True, show_score=True,out=None):
    if out is None: out = rgbd.disp(mode=disp_mode)
    # if show_steps:
    #     low_score_inds = get_low_score_inds(Grip[:, -1], score_thresh)
    #     for grip in Grip[low_score_inds, :]:
    #         x0, y0, x1, y1 = grip[-5:-1].astype('int')
    #         cv2.line(out, (x0, y0), (x1, y1), (0, 0, 255), 1)

        # high_score_inds = get_high_score_inds(Grip[:, -1], score_thresh)
        # for grip in Grip[high_score_inds, :]:
        #     x0, y0, x1, y1 = grip[-5:-1].astype('int')
        #     cv2.line(out, (x0, y0), (x1, y1), (0, 0, 255), 1)

    # for grip in Grip[best_n_inds, :]:
    #     x0, y0, x1, y1 = grip[-5:-1].astype('int')
    #     cv2.line(out, (x0, y0), (x1, y1), (255, 0, 100), 1)

    # grip = Grip[best_ind, :]
    # rect = grasp2TiltedRect(grip[0], grip[1], grip[3], grip[4], grip[5])
    # for j, pt in enumerate(rect):
    #     next_pt = rect[j + 1] if j < 3 else rect[0]
    #     cv2.line(out, pt, next_pt, (255, 0, 0), 2)

    # if show_score:
    #     text_scale = 0.8
    #     text_thick = 2
    #     x, y = rgbd.workspace.bbox[:2]
    #     score = Grip[(best_ind, -1)]
    #     cv2.putText(out, f'Grip: {score:>0.3f}', (x, y), cv2.FONT_HERSHEY_COMPLEX, text_scale,
    #                 (255, 0, 0), text_thick)
    for grip in Grip:
        x0, y0, x1, y1 = grip[-5:-1].astype('int')
        cv2.line(out, (x0, y0), (x1, y1), (255, 0, 100), 3)


    return out


if __name__=='__main__':
    # print("Demo GraspPoint")
    # print("====================")
    rgb_data = cv2.imread("../data/Grasp_RGB.png")
    depth_data = cv2.imread("../data/Grasp_depth.png", -1)
    input_RGBD = RGBD(rgb_data, depth_data, depth_min=400, depth_max=800)
    ppx, ppy, fx, fy = np.load("intr_param.npy")
    # print("====================")
    filename="../data/Result_Data.json"
    with open(filename) as json_file:
        json_data = json.load(json_file)
        grips=np.array(json_data["grip"])
        result_image=np.array(json_data['im']).astype(np.uint8)
        best_index=json_data['best_ind']
        best_n_inds=np.array(json_data['best_n_inds'])

    # print("best_index", best_index)
    best_grip = grips[best_index,:]
    # print_grips(best_grip)
    # print("====================")
    # print("best_n_inds", best_n_inds.size)
    best_n_grips=grips[best_n_inds]
    # print("best top n grips")
    index=0
    # print(best_n_grips)
    for cur_grip in best_n_grips:
        # print("#",str(index)," : ", end=" / ")
        # print_grips(cur_grip)
        index+=1
    # print("====================")
    grips = sorted(grips, key=lambda x: x[10])
    grips = np.array(grips)
    print(grips[-6:-1])
    out=show_grips(input_RGBD, grips[-6:-1], best_index, best_n_inds, disp_mode='rgb', score_thresh=0.7, show_steps=True,
                   show_score=True, out=None)
    cv2.imshow("result_image",out)
    cv2.waitKey(0)

    # Mission
    # 상위 score 5개의 grips을 원본 rgb_data에 표시하기
    # opencv의 line 함수 이용
