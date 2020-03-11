#!/usr/bin/env python
# --------------------------------------
# file:      uv_transformation.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     subprogram in profile_detection_v1_08.py
#            adapt to the different camera view size
#            for [test]
# --------------------------------------------------
import numpy as np

class camera_parameter:
    def __init__(self, name):
        self.name = name

    def get_transformation_matrixes(self):
        if self.name == "c18tod70":
            # intrinsic parameters color camera 18
            camera_cx_c = 639.999
            camera_cy_c = 356.782
            camera_fx_c = 923.577
            camera_fy_c = 924.282
            # intrinsic parameters depth camera 70
            camera_cx_d = 641.227
            camera_cy_d = 354.445
            camera_fx_d = 650.822
            camera_fy_d = 650.822
            # get from rs-sensor-control
            # from 18 Color: RGB8 1280*720 30Hz
            # to   70 Depth: Z16  1280*720 30Hz
            T_color2depth = np.array([[0.999976, -0.000363598, -0.00694777, -0.0147219],
                                      [0.000364764, 1, 0.000166607, -0.000306018],
                                      [0.00694771, -0.000169137, 0.999976, -0.000312607],
                                      [0, 0, 0, 1]])
        elif self.name == "c70tod75":
            # intrinsic parameters color camera 70
            camera_cx_c = 424
            camera_cy_c = 237.855
            camera_fx_c = 615.718
            camera_fy_c = 616.188
            # intrinsic parameters depth camera 75
            camera_cx_d = 424.813
            camera_cy_d = 236.32
            camera_fx_d = 431.169
            camera_fy_d = 431.169
            # get from rs-sensor-control
            # from 70 Color: BGR8 848x480 30Hz
            # to   75 Depth: Z16  848x480 30Hz
            T_color2depth = np.array([[0.999976, -0.000363598, -0.00694777, -0.0147219],
                                      [0.000364764, 1, 0.000166607, -0.000306018],
                                      [0.00694771, -0.000169137, 0.999976, -0.000312607],
                                      [0, 0, 0, 1]])
        else:
            print("False Input for camera parameters")

        T_camer2sensor_c = np.array([[camera_fx_c, 0, camera_cx_c], [0, camera_fy_c, camera_cy_c], [0, 0, 1]])
        T_camer2sensor_d = np.array([[camera_fx_d, 0, camera_cx_d], [0, camera_fy_d, camera_cy_d], [0, 0, 1]])
        return T_camer2sensor_c,T_camer2sensor_d,T_color2depth

def uv_transformation(u_c, v_c,name):
    cameras1 = camera_parameter(name)
    T_camer2sensor_c,T_camer2sensor_d,T_color2depth = cameras1.get_transformation_matrixes()
    # # intrinsic parameters color camera
    # camera_cx_c = 639.999
    # camera_cy_c = 356.782
    # camera_fx_c = 923.577
    # camera_fy_c = 924.282
    #
    # T_camer2sensor_c = np.array([[camera_fx_c, 0, camera_cx_c], [0, camera_fy_c, camera_cy_c], [0, 0, 1]])
    p_uv_c = np.array([[u_c], [v_c], [1]])
    p_c = np.dot(np.linalg.inv(T_camer2sensor_c), p_uv_c)

    # get from rs-sensor-control
    # from 18 Color: RGB8 1280*720 30Hz
    # to   70 Depth: Z16  1280*720 30Hz
    # T_color2depth = np.array([[0.999976, -0.000363598, -0.00694777, -0.0147219],
    #                           [0.000364764, 1, 0.000166607, -0.000306018],
    #                           [0.00694771, -0.000169137, 0.999976, -0.000312607],
    #                           [0, 0, 0, 1]])
    p_c = np.append(p_c, 1)
    p_c = p_c.reshape(p_c.shape[0], 1) # transport

    p_d = np.dot(T_color2depth, p_c)
    p_d = p_d[0:3]

    # # intrinsic parameters depth camera
    # camera_cx_d = 641.227
    # camera_cy_d = 354.445
    # camera_fx_d = 650.822
    # camera_fy_d = 650.822

    # T_camer2sensor_d = np.array([[camera_fx_d, 0, camera_cx_d], [0, camera_fy_d, camera_cy_d], [0, 0, 1]])
    p_uv_d = np.dot(T_camer2sensor_d, p_d) / p_d[2]
    u_d = round_to_int(p_uv_d[0])
    v_d = round_to_int(p_uv_d[1])
    # print("point in color camera's sensor ({},{}) is the point ({},{}) in depth camera's sensor".format(p_uv_c[0],
    #                                                                                                     p_uv_c[1],
    #                                                                                                     u_d, v_d))
    return u_d, v_d


def round_to_int(x):
    if x - int(x) > 0.5:
        x = int(x) + 1
    else:
        x = int(x)
    return x


if __name__ == '__main__':
    u_in_color_camera = 553
    v_in_color_camera = 152
    uv_transformation(u_in_color_camera, v_in_color_camera,"c70tod75")
