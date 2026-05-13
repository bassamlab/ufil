# Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import numpy as np
import copy

from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs as tf2g


def project_point_to_image_plane(point_camera, K, distortion_coeffs):
    """
    Projects a 3D point in the camera coordinate system onto the 2D image plane,
    taking into account intrinsic camera parameters and lens distortion.
    Parameters:
    point_camera (tuple): A tuple (X_c, Y_c, Z_c) representing the 3D point in the camera coordinate system.
    K (numpy.ndarray): A 3x3 intrinsic camera matrix.
    distortion_coeffs (tuple): A tuple (k1, k2, p1, p2, k3) containing the distortion coefficients.
    Returns:
    tuple: A tuple (u, v) representing the 2D pixel coordinates on the image plane.
    """

    # Unpack distortion coefficients
    k1, k2, p1, p2, k3 = distortion_coeffs

    # Perspective projection
    X_c, Y_c, Z_c = point_camera
    x_normalized = X_c / Z_c
    y_normalized = Y_c / Z_c

    # Compute r^2
    r2 = x_normalized**2 + y_normalized**2

    # Radial distortion
    x_distorted = x_normalized * (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3)
    y_distorted = y_normalized * (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3)

    # Tangential distortion
    x_distorted += 2 * p1 * x_normalized * y_normalized + p2 * (r2 + 2 * x_normalized**2)
    y_distorted += p1 * (r2 + 2 * y_normalized**2) + 2 * p2 * x_normalized * y_normalized

    # Map to pixel coordinates
    #3x3 Intrinsic
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    u = fx * x_distorted + cx
    v = fy * y_distorted + cy

    

    return int(u), int(v)

# Roll Pich Yaw to Rotation Matrix
def rpy_to_rotation_matrix(roll, pitch, yaw):
    """
    Converts roll, pitch, and yaw angles to a rotation matrix.
    Parameters:
    roll (float): Rotation angle around the x-axis in radians.
    pitch (float): Rotation angle around the y-axis in radians.
    yaw (float): Rotation angle around the z-axis in radians.
    Returns:
    numpy.ndarray: A 3x3 rotation matrix representing the combined rotation.
    """
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    return R_z @ R_y @ R_x


def transform_point_to_pixel(track_object_org, m1_point_transform, intrinsic, distortion, projection, T_camera_lidar):
    """
    Transforms a 3D point from the lidar coordinate system to the pixel coordinate system of a camera.
    Args:
        track_object_org (object): The original track object containing the state and dimension information.
        m1_point_transform (Transform): The transformation to be applied to the base point.
        intrinsic (np.ndarray): The intrinsic camera matrix.
        distortion (np.ndarray): The distortion coefficients of the camera.
        projection (np.ndarray): The projection matrix.
        T_camera_lidar (np.ndarray): The transformation matrix from the lidar coordinate system to the camera coordinate system.
    Returns:
        tuple: A tuple (u, v, x, y) where (u, v) are the pixel coordinates and (x, y) are the transformed coordinates.
    """

    track_object = copy.deepcopy(track_object_org)
    base_point = PointStamped()
    base_point.point.x = track_object.state.state.x
    base_point.point.y = track_object.state.state.y
    base_point.point.z = track_object.dimension.dimension.height/2

    m1_point = tf2g.do_transform_point(base_point, copy.deepcopy(m1_point_transform))
    m1_point = np.array([m1_point.point.x, m1_point.point.y, m1_point.point.z, 1])

    
    u,v,x,y = transform(m1_point, distortion, intrinsic, projection, T_camera_lidar)
    return u,v,x,y


# Project point to Pixel Cordinates
def to_cam(track_object_org, transform_to_cam, intrinsic, distortion, projection):
    """
    Transforms a track object from its original frame to the camera frame and projects it to pixel coordinates.

    Args:
        track_object_org (object): The original track object containing state and dimension information.
        transform_to_cam (Transform): The transformation to apply from the track frame to the camera frame.
        intrinsic (array-like): The intrinsic parameters of the camera.
        distortion (array-like): The distortion coefficients of the camera.
        projection (array-like): The projection matrix of the camera.

    Returns:
        tuple: A tuple (u, v, x, y) representing the pixel coordinates (u, v) and the transformed coordinates (x, y).
    """

    # Safe position of object in point
    track_object = copy.deepcopy(track_object_org)
    base_point = PointStamped()
    base_point.point.x = track_object.state.state.x
    base_point.point.y = track_object.state.state.y
    base_point.point.z = track_object.dimension.dimension.height/2

    # Transform from Track Frame to Camera Frame
    m1_point = tf2g.do_transform_point(base_point, transform_to_cam)
    m1_point = np.array([m1_point.point.x, m1_point.point.y, m1_point.point.z, 1])

    # Project to Pixel Cordinates
    u,v,x,y = to_pixel(m1_point, distortion, intrinsic, projection)
    return u,v,x,y

# Project Camera Cordinate Point to Pixel
def to_pixel(point_cam, distortion, intrinsic, projection):
    """
    Projects a 3D point from camera coordinates to pixel coordinates in the image plane.
    Args:
        point_cam (numpy.ndarray): A 3D point in camera coordinates.
        distortion (numpy.ndarray): Distortion coefficients for the camera.
        intrinsic (numpy.ndarray): Intrinsic matrix of the camera.
        projection (numpy.ndarray): Projection matrix for the camera.
    Returns:
        tuple: A tuple containing:
            - u (float): The u-coordinate (horizontal) in the image plane.
            - v (float): The v-coordinate (vertical) in the image plane.
            - x (float): The x-coordinate in the image plane after projection.
            - y (float): The y-coordinate in the image plane after projection.
    """
   
    point_cam = point_cam[0:3]
    point_cam = np.squeeze(np.asarray(point_cam))
    u, v = project_point_to_image_plane(point_cam, intrinsic.reshape(3, 3), distortion)
    point_cam = np.append(point_cam, 1)
    x, y = project_point_to_image(point_cam, projection.reshape(3,4))
    u,v = x,y
    return u,v, x,y


# Project to Pixel using Projection Matrix
def project_point_to_image(point_m1, projection):
    """
    Projects a 3D point onto a 2D image plane using a given projection matrix.
    Args:
        point_m1 (numpy.ndarray): A 3D point in homogeneous coordinates (shape: (4,)).
        projection (numpy.ndarray): A 3x4 projection matrix.
    Returns:
        tuple: The (x, y) coordinates of the projected point on the 2D image plane.
    """

    point = np.dot(projection, point_m1)
    x = point[0]/point[2]
    y = point[1]/point[2]
    return int(x),int(y)

# Project M1 Point to Pixel Cordinates Using Projection Matrix T_camera_lidar
def transform(point_m1, distortion, intrinsic, projection, T_camera_lidar):
    """
    Transforms a point from one coordinate system to another and projects it onto an image plane.
    Args:
        point_m1 (numpy.ndarray): The point in the original coordinate system.
        distortion (numpy.ndarray): The distortion coefficients for the camera.
        intrinsic (numpy.ndarray): The intrinsic matrix of the camera.
        projection (numpy.ndarray): The projection matrix.
        T_camera_lidar (numpy.ndarray): The transformation matrix from the LiDAR coordinate system to the camera coordinate system.
    Returns:
        tuple: A tuple containing:
            - u (float): The u-coordinate of the projected point on the image plane.
            - v (float): The v-coordinate of the projected point on the image plane.
            - x (float): The x-coordinate of the projected point in the image.
            - y (float): The y-coordinate of the projected point in the image.
    """
    
    point = np.transpose(point_m1)
    point_camera = np.dot(T_camera_lidar,point).transpose()
    point_camera = point_camera[0:3]
    point_camera = np.squeeze(np.asarray(point_camera))


    u, v = project_point_to_image_plane(point_camera, intrinsic.reshape(3, 3), distortion)
    point_camera = np.append(point_camera, 1)
    x, y = project_point_to_image(point_camera, projection.reshape(3,4))
    return u,v, x,y
