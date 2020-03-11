#!/usr/bin/env python
# --------------------------------------
# file:      StraightDownZ.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     subprogram for the first step in grasp_object.py
#            adjust the z-axis straight down
# --------------------------------------------------
import math
import numpy as np
import tf
from RPY2RotationVector import rpy2rv


# convert roation vector to quaternion
def rv2quat(rotation_vector):
    rv_norm = np.linalg.norm(rotation_vector)
    if rv_norm < 0.0001:  # rv = [0,0,0]
        u = rotation_vector * 0
    else:
        u = rotation_vector / rv_norm
    theta = rv_norm
    quaternion_array = axisangle2quaternion(u, theta)
    quaternion_list = list(quaternion_array)

    return quaternion_list


# convert axis angle into quaternion
def axisangle2quaternion(u, theta):
    s = math.cos(theta / 2)
    v = u * math.sin(theta / 2)
    quaternion_array = np.array([s, v[0], v[1], v[2]])
    return quaternion_array


def quaternion_multiply(quaternion1, quaternion2):
    s1, x1, y1, z1 = quaternion1
    s2, x2, y2, z2 = quaternion2
    return np.array([s1 * s2 - x1 * x2 - y1 * y2 - z1 * z2,
                     s1 * x2 + s2 * x1 + y1 * z2 - z1 * y2,
                     s1 * y2 + s2 * y1 + z1 * x2 - x1 * z2,
                     s1 * z2 + s2 * z1 + x1 * y2 - y1 * x2], dtype=np.float64)


def z_straight_down(current_pose):
    z_0 = np.array([0, 0, 1])
    #  rasise a random pose to one whose z-axis towards down
    pose_1 = current_pose
    pos = current_pose[0:3]
    rv = pose_1[3:6]
    quat_b1 = rv2quat(rv)  # sxyz
    quat_b1_xyzw = [quat_b1[1], quat_b1[2], quat_b1[3], quat_b1[0]]
    
    T_matrix_1 = tf.transformations.quaternion_matrix(quat_b1_xyzw)
    z_1 = T_matrix_1[0:3, 2]
    n_vector = np.cross(z_0, z_1)
    n_norm = np.linalg.norm(n_vector)
    p_vector = np.dot(z_0, z_1)
    norm_product = np.linalg.norm(z_0) * np.linalg.norm(z_1)
    # rotation vector in base coordinate system
    if n_norm < 0.0001:  # z_0 and z_1 are collinear
        rotation_vector_12_CS0 = T_matrix_1[0:3, 1]
    else:
        rotation_vector_12_CS0 = n_vector / n_norm
    rotation_angle_12 = math.pi - math.acos(p_vector / norm_product)
    # rotation vector is to be converted into 1. coordinate system
    R_matrix_1 = T_matrix_1[0:3, 0:3]
    R_inv = np.linalg.inv(R_matrix_1)
    rotation_vector_12_CS1 = np.dot(R_inv, rotation_vector_12_CS0.reshape(3, 1))

    quat_12 = axisangle2quaternion(rotation_vector_12_CS1, rotation_angle_12)
    quat_b2 = quaternion_multiply(quat_b1, quat_12)
    quat_b2_xyzw = [quat_b2[1][0], quat_b2[2][0], quat_b2[3][0], quat_b2[0][0]]
    
    # validate the result
    T_matrix_2 = tf.transformations.quaternion_matrix(quat_b2_xyzw)
    # print("T matix1:\n"+str(T_matrix_1))
    # print("T matix2:\n"+str(T_matrix_2))

    euler = tf.transformations.euler_from_quaternion(quat_b2_xyzw)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    (rx, ry, rz) = rpy2rv(roll, pitch, yaw)
    rv2 = np.array([rx, ry, rz])
    # rv2_direct = math.pi*rotation_vector_12_CS0
    # print(rv2)
    pose_2 = np.append(pos, rv2)

    return pose_2


if __name__ == '__main__':
    pose_original = np.array([0.2294, 0.5267, 0.05995, -1.684, -0.5462, -0.0912])
    pose_z_down = z_straight_down(pose_original)

# if __name__ == '__main__':
#     z_0 = np.array([0, 0, 1])
#
#     # test data
#     pose_1 = np.array([0.286958, 0.296590, 0.068019, -0.708585, -3.050590, 0.003502])
#     rv = pose_1[3:6]
#     # rv = np.array([3, 0.5, 2.5])
#     # rv = np.array([0.2,0.5,0.8426])*2.0
#     # rv = np.array([0, 0, 0])
#     # rv = np.array([1, 1, 1]) * 1.20919
#     quat_b1 = rv2quat(rv)  # sxyz
#     quat_b1_xyzw = [quat_b1[1], quat_b1[2], quat_b1[3], quat_b1[0]]
#
#     T_matrix_1 = tf.transformations.quaternion_matrix(quat_b1_xyzw)
#     z_1 = T_matrix_1[0:3, 2]
#     n_vector = np.cross(z_0, z_1)
#     n_norm = np.linalg.norm(n_vector)
#     p_vector = np.dot(z_0, z_1)
#     norm_product = np.linalg.norm(z_0) * np.linalg.norm(z_1)
#     # rotation vector in base coordinate system
#     if n_norm < 0.0001:  # z_0 and z_1 are collinear
#         rotation_vector_12 = T_matrix_1[0:3, 1]
#     else:
#         rotation_vector_12 = n_vector / n_norm
#     rotation_angle_12 = math.pi - math.acos(p_vector / norm_product)
#     # rotation vector is to be converted into 1. coordinate system
#     R_matrix_1 = T_matrix_1[0:3, 0:3]
#     R_inv = np.linalg.inv(R_matrix_1)
#     rotation_vector_12 = np.dot(R_inv, rotation_vector_12.reshape(3, 1))
#
#     quat_12 = axisangle2quaternion(rotation_vector_12, rotation_angle_12)
#     quat_b2 = quaternion_multiply(quat_b1, quat_12)
#     quat_b2_xyzw = [quat_b2[1], quat_b2[2], quat_b2[3], quat_b2[0]]
#
#     # validate the result
#     T_matrix_2 = tf.transformations.quaternion_matrix(quat_b2_xyzw)
#     print(T_matrix_1)
#     print(T_matrix_2)
#     rv2 = rotation_vector_12*rotation_angle_12
#     print(rv2)
