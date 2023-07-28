import numpy as np
import math as m

class selecting():
    
    
    def __init__(self, width, hight, depth_data, intr_params):
        self.width_param = width
        self.hight_param = hight
        self.depth_data = depth_data
        self.intr_params = intr_params
        
    def distance_fiter(self, grips):
        #depth_data : rgb_data, depth_data = sensor.get_data()
        
        # sensor = RSSensor()
        # sensor.start()
        # ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy
        
        spatial_grips = []
        
        for i in grips:
            
            #center
            center_Xc = i[0]
            center_Yc = i[1]
            center_Zc = self.depth_data[center_Yc,center_Xc]
            center_point_x = (center_Xc-self.intr_params[0])/self.intr_params[2]*center_Zc
            center_point_y = (center_Yc-self.intr_params[1])/self.intr_params[3]*center_Zc
            center_point_z = float(center_Zc)
            
            #left
            left_Xc = i[6]
            left_Yc = i[7]
            left_Zc = self.depth_data[left_Yc,left_Xc]
            left_point_x = (left_Xc-self.intr_params[0])/self.intr_params[2]*left_Zc
            left_point_y = (left_Yc-self.intr_params[1])/self.intr_params[3]*left_Zc
            left_point_z = float(left_Zc)
            
            #right
            right_Xc = i[8]
            right_Yc = i[9]
            right_Zc = self.depth_data[right_Yc,right_Xc]
            right_point_x = (right_Xc-self.intr_params[0])/self.intr_params[2]*right_Zc
            right_point_y = (right_Yc-self.intr_params[1])/self.intr_params[3]*right_Zc
            right_point_z = float(right_Zc)
            
            #girp width
            width = m.sqrt((left_point_x - right_point_x)**2 + (left_point_y - right_point_y) ** 2 + (left_point_z - right_point_z) ** 2)
            hight = abs(left_point_z = right_point_x)
            
            
            #filtering
            
            if self.width_param < width and self.hight_param < hight:
            # [[center], [left], [right], [width, hight], score]
                spatial_grips.append([[center_point_x, center_point_y, center_point_z],
                                      [left_point_x, left_point_y, left_point_z],
                                      [right_point_x, right_point_y, right_point_z],
                                      [width, hight],
                                      i[10]])
            
        if len(spatial_grips) > 0:
            spatial_grips = sorted(spatial_grips, key=lambda x: x[10])
            return spatial_grips
        
        else :
            return 0
        
    def optimal_select(self, grips):
        
        grips = self.distance_fiter(grips)
        
        if grips == 0:
            return -1
        
        else :
            grips = sorted(grips, key=lambda x: x[4])

            return grips[-1]
        
        
        
            
            
