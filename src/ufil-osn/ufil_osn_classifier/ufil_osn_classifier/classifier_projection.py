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


"""Projection of tracked objects into camera images."""

import copy

from geometry_msgs.msg import PointStamped
import numpy as np
import tf2_geometry_msgs as tf2g


def project_point_to_image_plane(point_camera, K, distortion_coeffs):
    """
    Project a 3D point in camera coordinates onto the 2D image plane.

    Takes the intrinsic camera parameters and the lens distortion into account.

    :param point_camera: Tuple (X_c, Y_c, Z_c), the 3D point in camera coordinates.
    :param K: 3x3 intrinsic camera matrix.
    :param distortion_coeffs: Distortion coefficients (k1, k2, p1, p2, k3).
    :return: Tuple (u, v) of 2D pixel coordinates on the image plane.
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

    # Map to pixel coordinates using the 3x3 intrinsic matrix
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    u = fx * x_distorted + cx
    v = fy * y_distorted + cy

    return int(u), int(v)


def rpy_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert roll, pitch and yaw angles to a rotation matrix.

    :param roll: Rotation angle around the x-axis in radians.
    :param pitch: Rotation angle around the y-axis in radians.
    :param yaw: Rotation angle around the z-axis in radians.
    :return: 3x3 rotation matrix R_z @ R_y @ R_x.
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


def transform_point_to_pixel(track_object_org, m1_point_transform, intrinsic, distortion,
                             projection, T_camera_lidar):
    """
    Transform the center of a track from the lidar frame to pixel coordinates of a camera.

    :param track_object_org: Track object containing state and dimension.
    :param m1_point_transform: Transform applied to the track center first.
    :param intrinsic: Intrinsic camera matrix.
    :param distortion: Distortion coefficients of the camera.
    :param projection: Projection matrix of the camera.
    :param T_camera_lidar: Homogeneous transformation from the lidar to the camera frame.
    :return: Tuple (u, v, x, y) of pixel coordinates with (u, v) using the intrinsic
        matrix and distortion and (x, y) using the projection matrix.
    """
    track_object = copy.deepcopy(track_object_org)
    base_point = PointStamped()
    base_point.point.x = track_object.state.state.x
    base_point.point.y = track_object.state.state.y
    base_point.point.z = track_object.dimension.dimension.height / 2

    m1_point = tf2g.do_transform_point(base_point, copy.deepcopy(m1_point_transform))
    m1_point = np.array([m1_point.point.x, m1_point.point.y, m1_point.point.z, 1])

    u, v, x, y = transform(m1_point, distortion, intrinsic, projection, T_camera_lidar)
    return u, v, x, y


def to_cam(track_object_org, transform_to_cam, intrinsic, distortion, projection):
    """
    Project the center of a track to pixel coordinates of a camera.

    :param track_object_org: Track object containing state and dimension.
    :param transform_to_cam: Transform from the track frame to the camera frame.
    :param intrinsic: Intrinsic parameters of the camera.
    :param distortion: Distortion coefficients of the camera.
    :param projection: Projection matrix of the camera.
    :return: Tuple (u, v, x, y), both pairs are the pixel coordinates computed with
        the projection matrix.
    """
    # Save position of object in point
    track_object = copy.deepcopy(track_object_org)
    base_point = PointStamped()
    base_point.point.x = track_object.state.state.x
    base_point.point.y = track_object.state.state.y
    base_point.point.z = track_object.dimension.dimension.height / 2

    # Transform from track frame to camera frame
    m1_point = tf2g.do_transform_point(base_point, transform_to_cam)
    m1_point = np.array([m1_point.point.x, m1_point.point.y, m1_point.point.z, 1])

    # Project to pixel coordinates
    u, v, x, y = to_pixel(m1_point, distortion, intrinsic, projection)
    return u, v, x, y


def to_pixel(point_cam, distortion, intrinsic, projection):
    """
    Project a 3D point in camera coordinates to pixel coordinates.

    :param point_cam: 3D point in camera coordinates.
    :param distortion: Distortion coefficients of the camera.
    :param intrinsic: Intrinsic matrix of the camera.
    :param projection: Projection matrix of the camera.
    :return: Tuple (u, v, x, y), both pairs are the pixel coordinates computed with
        the projection matrix.
    """
    point_cam = point_cam[0:3]
    point_cam = np.squeeze(np.asarray(point_cam))
    u, v = project_point_to_image_plane(point_cam, intrinsic.reshape(3, 3), distortion)
    point_cam = np.append(point_cam, 1)
    x, y = project_point_to_image(point_cam, projection.reshape(3, 4))
    u, v = x, y
    return u, v, x, y


def project_point_to_image(point_m1, projection):
    """
    Project a 3D point onto the 2D image plane using a projection matrix.

    :param point_m1: 3D point in homogeneous coordinates (shape (4,)).
    :param projection: 3x4 projection matrix.
    :return: Tuple (x, y) of pixel coordinates.
    """
    point = np.dot(projection, point_m1)
    x = point[0] / point[2]
    y = point[1] / point[2]
    return int(x), int(y)


def transform(point_m1, distortion, intrinsic, projection, T_camera_lidar):
    """
    Transform a lidar point to the camera frame and project it onto the image plane.

    :param point_m1: Point in homogeneous lidar coordinates.
    :param distortion: Distortion coefficients of the camera.
    :param intrinsic: Intrinsic matrix of the camera.
    :param projection: Projection matrix of the camera.
    :param T_camera_lidar: Homogeneous transformation from the lidar to the camera frame.
    :return: Tuple (u, v, x, y) of pixel coordinates with (u, v) using the intrinsic
        matrix and distortion and (x, y) using the projection matrix.
    """
    point = np.transpose(point_m1)
    point_camera = np.dot(T_camera_lidar, point).transpose()
    point_camera = point_camera[0:3]
    point_camera = np.squeeze(np.asarray(point_camera))

    u, v = project_point_to_image_plane(point_camera, intrinsic.reshape(3, 3), distortion)
    point_camera = np.append(point_camera, 1)
    x, y = project_point_to_image(point_camera, projection.reshape(3, 4))
    return u, v, x, y
