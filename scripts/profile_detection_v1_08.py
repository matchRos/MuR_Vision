#!/usr/bin/env python
# --------------------------------------
# file:      profile_detection_v1_08.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     detect an object from the image and estimate 6D-pose of the object
#   main function: object_detector(color_img, depth_img, aligned_depth_img)
#      parameters: color_img ---> color image (original image from color module in camera)
#                  depth_img ---> depth image (original image from depth module in camera)
#                  aligned_depth_img ---> processed depth image, it results from the alignment of depth image to color image
#         returns: origin_CS_object, draw_img, theta_z, length_vector
#                  origin_CS_object ---> the origin of object coordinate system
#                  draw_img ---> color image with a detected object which is marked by a rectangle
#                  theta_z ---> inclined angle of the rectangle in the image
#                  length_vector ---> length vector of the object
# --------------------------------------------------
# process:
# 1  -- read images
# 2  -- convert into grayscale and reduce noise
# 3  -- extract gradients from images
# 4  -- secondary noise supression
# 5  -- morphological transformation and enlarge the profile
# 6  -- find contours
# 7  -- draw contours
# 8  -- capture depth of ROD by applying an affine transformation to depth image
# 9  -- save images (see v1_06)
# 10 -- return coordinate of point and image
## step 8 is irrelevant to the 3D-position estimation of point


import cv2
import numpy as np
import pyrealsense2 as rs
import sys

from functions_module import get_distance, coordinates_3D_calculation, \
    parameter_adjustment
from uv_transformation import uv_transformation

#---------- read images from recorded video----------------
def read_images_from_video(video_path):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(video_path)
    profile = pipe.start(cfg)

    for x in range(5):
        pipe.wait_for_frames()

    frameset_captured = pipe.wait_for_frames()
    if frameset_captured is not None:
        print("Frame Captured")
        # color_frame_captured = frameset_captured.get_color_frame()
        # depth_frame_captured = frameset_captured.get_depth_frame()
        # return color_frame_captured, depth_frame_captured
        return profile, frameset_captured
    else:
        print("Error: No Frame Found!")
    pipe.stop()

#---------- image processing direct from an available image----------------
def get_image(path):
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img, gray


def Sobel_gradient(blurred):
    # calculate gradX and gradY using Sobel operator
    gradX = cv2.Sobel(blurred, ddepth=cv2.CV_32F, dx=1, dy=0)
    gradY = cv2.Sobel(blurred, ddepth=cv2.CV_32F, dx=0, dy=1)
    # gradient = cv2.subtract(gradX, gradY)
    # gradient = cv2.convertScaleAbs(gradient)
    abs_gradX = cv2.convertScaleAbs(gradX)
    abs_gradY = cv2.convertScaleAbs(gradY)
    gradient = cv2.addWeighted(abs_gradX, 1, abs_gradY, 1, 0)
    # cv2.imshow("gradX-gradY", gradient)
    # cv2.imshow("gradX/2+gradY/2", gradient2)
    return gradX, gradY, gradient


def Thresh_and_blur(gradient):
    blurred = cv2.GaussianBlur(gradient, (9, 9), 2)
    # thresh = parameter_adjustment(blurred, 1, "threshold")
    ret, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
    return thresh


def image_morphology(thresh):
    # refer to the link
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    opening = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    #  enlarge the profile
    opening = cv2.erode(opening, None, iterations=4)
    opening = cv2.dilate(opening, None, iterations=4)

    return opening


def findcnts_and_box_point(closed):
    # find contours and sort them according their areas
    # obtain the minimal rectangle of the contour with largest area
    # in python3.5 only two outputs cnts, hierarchy
    # in python2.7 three outputs: img, cnts, hierarchy
    img,cnts, hierarchy = cv2.findContours(closed.copy(),
                                       cv2.RETR_LIST,
                                       cv2.CHAIN_APPROX_SIMPLE)
    sorted_contours = sorted(cnts, key=cv2.contourArea, reverse=True)
    return sorted_contours


def drawcnts_and_cut(original_img, box):
    # because drawContours will leave the contours on the image to draw, it must use a copy to keep the image unchanged.
    # draw a bounding box arounded the detected barcode and display the image
    draw_img = cv2.drawContours(original_img.copy(), [box], -1, (0, 0, 255), 3)

    Xs = [i[0] for i in box]
    Ys = [i[1] for i in box]
    x1 = min(Xs)
    x2 = max(Xs)
    y1 = min(Ys)
    y2 = max(Ys)
    hight = y2 - y1
    width = x2 - x1
    crop_img = original_img[y1:y1 + hight, x1:x1 + width]

    if x1 < 0 or y1 < 0:
        print("Error: x1 < 0, y1<0, please check it again before cropping the image!")
        return draw_img, draw_img
    else:
        return draw_img, crop_img


def alignment_depth2color(frameset_to_align):
    stream_to_align = rs.stream.color
    align = rs.align(stream_to_align)
    aligned_frames = align.process(frameset_to_align)
    return aligned_frames


def remove_black_points(array):
    counter = 0
    if array.ndim == 2:
        row_num, col_num = array.shape
        for i in range(row_num):
            for j in range(col_num):
                if array[i][j] < 0.01:
                    counter = counter + 1
        return counter
    else:
        pass


def affine_transformation(image_to_rotate, rotation_matrix, points_rect, img_type):
    image_temp = image_to_rotate.copy()  # in alignment problem use box2rect(box_d)
    # image_to_rotate = original_img.copy() # alternative to see rotated color image
    dst_size = (image_temp.shape[1], image_temp.shape[0])  # (y,x)
    rot_img = cv2.warpAffine(image_temp, rotation_matrix, dst_size)
    rot_img_temp = rot_img.copy()
    test_img = cv2.drawContours(rot_img_temp, [points_rect], -1, (0, 0, 255), 3)
    # if image_to_rotate.ndim == 3:  # image is color image
    ## cv2.imshow("Rotated Image", rot_img)
    # cv2.imshow("Test of Feasiblity of Affine Transformation of " + img_type, test_img)
    ## elif image_to_rotate.ndim == 2:  # image is depth image and not colorized

    return rot_img


def object_detector(color_img, depth_img, aligned_depth_img):
    # 0-- initilization
    depth_scale = 0.0010000000474974513
    flag = 0
    distance_array = []

    # 1-- read images and align depth images to color images--

    depth_image_in_meters = aligned_depth_img * depth_scale
    raw_depth_image_in_meters = depth_img * depth_scale

    # 2-- convert into grayscale and reduce noise--

    gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    # blurred_img = parameter_adjustment(gray_img, 4, "GaussianBlur")
    blurred_img = cv2.GaussianBlur(gray_img, (5, 5), 1)

    # 3-- extract gradients from images--

    gradX, gradY, gradient = Sobel_gradient(blurred_img)

    # 4-- secondary noise supression--

    thresh = Thresh_and_blur(gradient)  # thresh is a binary image

    # 5-- morphological transformation and enlarge the profile--

    opening = image_morphology(thresh)  # closed is a bw image

    # 6-- find contours--

    sorted_cnts = findcnts_and_box_point(opening)

    # 6PULS-- verify the object is profile according to its length ratio--

    cnts_idx = 0
    # compute the minimal rectangle of contours
    while True:
        # determination, if list index is out of range
        if cnts_idx == len(sorted_cnts): # including cnt_idx = 0, cnts_idx == len(sorted_cnts) and cnts_idx > 0
            if cnts_idx == 0:
                flag = 3
                print("Error 3: no contours catched due to too fast movement of camera!")
            else:
                flag = 2
                print("Error 2: all contours don't satisfy the profile definition!")
            break
        print("  index of detected contours = " + str(cnts_idx) + ", length of list = " + str(len(sorted_cnts)))
        minrect_obj = cv2.minAreaRect(sorted_cnts[cnts_idx])
        # int0 is alias of intp: Integer used for indexing (same as C ssize_t; normally either int32 or int64)
        box = np.int0(cv2.boxPoints(minrect_obj))
        print("  box in uv coordinates: [{},{},{},{}]".format(box[0],box[1],box[2],box[3]))
        box_d = box.copy()
        # sensor coordinates transformation from color camera to unaligned depth camera
        # Region of Interest is the detected object

        # in livestream there is an approch to obtain the intrinsics for the aligned depth images
        # Therefore, it is no need to transform u,v coordinates from CSc to CSd (raw depth)
        ## for i in range(len(box_d)):
        ##     box_d[i] = (uv_transformation(box_d[i][0], box_d[i][1],"c70tod75"))
        ## 3D coordinates in real world but in camera coordinate system
        points = []
        dimensions = []
        for point_idx in range(4):
            p_3d = coordinates_3D_calculation(box_d[point_idx][0], box_d[point_idx][1],
                                              depth_image_in_meters, "aligned_depth")  # these two items to change
            if p_3d == [0, 0, 0]:
                print("         detected object has currently no depth information!")
                break
            else:
                points.append(p_3d)
        if len(points) == 4:  # depth information of 4 ponits are available
            d12 = int(get_distance(points[0], points[1]) * 1000)
            d23 = int(get_distance(points[1], points[2]) * 1000)
            d34 = int(get_distance(points[2], points[3]) * 1000)
            d41 = int(get_distance(points[3], points[0]) * 1000)
            dimensions = [d12, d23, d34, d41]
            dimensions.sort()
        else:  # no enough points of available depth information
            dimensions = [11111]  # nonsensical value, just get the loop step into the next
        print("  edge length of the detected rectangle: {}".format(dimensions))
        # print(points)
        # loop2 break -- length ratio: long profile (>5:1)  short profile (>3:1)
        if dimensions[0] == 11111:
            flag = 1
            print("Error 1: this image lacks depth infomation")
            break
        else:
            if dimensions[0] < 100 and dimensions[2] > 3 * dimensions[0]:
                print("\n!!A profil of size {}x{}x{} mm3 with {}. contours is detected!!".format(dimensions[2],
                                                                                        dimensions[0],
                                                                                        dimensions[0],
                                                                                        cnts_idx))
                break
            else:
                cnts_idx += 1
    # print("flag =" + str(flag))
    if flag == 3:
        return [-11, -11, -11], color_img, 0.0,[-11, -11, -11]

    # 7-- draw contours--

    # 7.1-- show all of images in the process
    draw_img, crop_img = drawcnts_and_cut(color_img, box)
    # draw_img_depth, crop_img_depth = drawcnts_and_cut(depth_image_before_alignment_colorized, box_d)
    # cv2.imshow('Figure 1: Original Image', color_img)
    # cv2.imshow('Figure 2: Blurred Image', blurred_img)
    # # cv2.imshow('gradX', gradX)
    # # cv2.imshow('gradY', gradY)
    # cv2.imshow('Figure 3: Sobel Edge Detection', gradient)
    # cv2.imshow('Figure 4: Thresholding', thresh)
    # cv2.imshow('Figure 5: Opening (Erosion and Dilation)', opening)

    # 7.2-- draw all of contours(see v1_06)

    # 7.3-- add note into final image
    font = cv2.FONT_HERSHEY_SIMPLEX
    if flag == 1:
        cv2.putText(draw_img, "No Depth Information!", (10, 350), font, 1, (255, 255, 255), 2,
                    cv2.LINE_AA)
    elif flag == 2:
        cv2.putText(draw_img, "Oversized!", (10, 350), font, 1, (255, 255, 255), 2,
                    cv2.LINE_AA)
    # cv2.namedWindow("Figure 6: Detected Object in Color", cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('Figure 6: Detected Object in Color', draw_img)
    # cv2.imshow('Figure 7: Detected Object', crop_img)
    # cv2.namedWindow("Figure 8: Detected Object in RAW Depth", cv2.WINDOW_AUTOSIZE)
    # cv2.imshow("Figure 8: Detected Object in RAW Depth", draw_img_depth)

    # 8--capture depth of ROD by applying an affine transformation to depth image--

    # depth data (z) will be obtained from aligned depth image,
    # while dimensions data (x,y,z) will be obtained from raw depth image
    # The reason is that the calculation of x and y needs the intrinsics of depth camera.
    # However, due to the alignment, the principal points of depth camera cx and cy do change.
    # Therefore, the only access to dimensions data is from unaligned (raw) depth image, where intrinsics are unchanged.
    # In the followings, it is so as "alignment problem" mention.

    # 8.1-- get the min rectangle of the detected object and rotation matrix of affine transformation
    rect_center, (width, length), theta = minrect_obj  # in alignment problem use box2rect(box_d)
    # print(minrect_obj)
    x_c, y_c = rect_center
    rot_mat = cv2.getRotationMatrix2D(rect_center, theta, 1.0)

    # 8.2-- get 4 points of rotated rectangle (straight)
    rect_box = np.asanyarray([[x_c - width / 2, y_c - length / 2],
                              [x_c - width / 2, y_c + length / 2],
                              [x_c + width / 2, y_c + length / 2],
                              [x_c + width / 2, y_c - length / 2]]).astype(np.int32)
    x_max = max([i[0] for i in rect_box])
    x_min = min([i[0] for i in rect_box])
    y_max = max([i[1] for i in rect_box])
    y_min = min([i[1] for i in rect_box])

    # 8.3-- validation of correctness of rotated coordinates
    # rotated_colorized_depth_image = affine_transformation(depth_image_colorized, rot_mat, rect_box,
    #                                                       "Colorized Depth Image")
    rotated_color_image = affine_transformation(color_img, rot_mat, rect_box, "Color Image")

    # 8.4-- get the data of RIO in unaligned depth image
    rotated_uncolorized_depth_image = affine_transformation(aligned_depth_img, rot_mat, rect_box,
                                                            "Raw Depth Image")
    if x_min < 0 or y_min < 0:
        # if x_min < 0:
        #     x_min = 0
        # if y_min < 0:
        #     y_min = 0
        print("Error: x_min = {} < 0, y_min = {} <0, please check it again before cropping the image!".format(
            x_min,
            y_min))
    else:
        depth_image_obj = rotated_uncolorized_depth_image[y_min:y_max, x_min:x_max].astype(float)
        # cv2.imshow("Figure 9: Cropped Object in Depth Image", depth_image_obj)
        depth_obj = depth_image_obj * depth_scale
        # remove the black points where no depth information is available
        distance = np.sum(depth_obj) / (depth_obj.size - remove_black_points(depth_obj))
        print("       The object is {:.4} meters away\n".format(distance))
        # distance_array.append(distance)
        distance_array = np.append(distance_array, distance)

    # 9 -- save images (see v1_06)

    # 10 -- return coordinate of point and image
    if flag:
        return [-1, -1, -1], draw_img, 0.0,[-1, -1, -1]
    # zmin = 10
    # p_i_zmin = 0
    # for p_i in range(4):
    #     if points[p_i][2] < zmin:
    #         zmin = points[p_i][2]
    #         p_i_zmin = p_i

    # get the bottom corner point on object in the view of camera
    # ------- Point Of Interest ------
    vmax = 0
    index_POI = 0
    for point_idx in range(4):
        if box[point_idx][1] > vmax:
            vmax = box[point_idx][1]
            index_POI = point_idx
    uv_PIO = box[index_POI]
    xyz_POI = points[index_POI]
    if index_POI == 3:
        index_POI_left = 0
    else:
        index_POI_left = index_POI +1
    if index_POI == 0:
        index_POI_right = 3
    else:
        index_POI_right = index_POI -1
    p12 = [points[index_POI_left][0] - points[index_POI][0], points[index_POI_left][1] - points[index_POI][1],
           points[index_POI_left][2] - points[index_POI][2]]
    p14 = [points[index_POI_right][0] - points[index_POI][0],points[index_POI_right][1] - points[index_POI][1],
           points[index_POI_right][2] - points[index_POI][2]]
    p12_norm = get_distance(points[index_POI_left],points[index_POI])
    p14_norm = get_distance(points[index_POI_right],points[index_POI])
    # origin_CS_object is always the point in the lower left corner in the view of camera
    # length_vector shows the direction of y axis of object coordinate system
    if p12_norm < p14_norm:  #right edge is longer
        # before rotation of z, first rotate on x for 180 Grad
        theta_z = 90 + theta
        origin_CS_object = points[index_POI_left]
        length_vector = p14
    else:                    #left edge is longer
        theta_z = theta
        origin_CS_object = xyz_POI
        length_vector = p12
    print("  pose of profile is {:.6} degree".format(theta_z))
    # return xyz_POI, draw_img, theta_z
    return origin_CS_object, draw_img, theta_z, length_vector


if __name__ == '__main__':
    # imgs from ros has unit [mm], but program is written according to framset where unit is [m]
    color_img = np.load("pics/color_img_eg1.npy")
    depth_img = np.load("pics/depth_img_eg1.npy")  # [mm]/1000 = [m]
    aligned_depth_img = np.load("pics/aligned_depth_img_eg1.npy")
    point = object_detector(color_img, depth_img, aligned_depth_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
