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


import nav_msgs.msg
import h5py

import numpy as np


def flatten_odometry(msg: nav_msgs.msg.Odometry, timestamp: np.float64, root_group: h5py.Group):
    """Conversion function for a NavSatFix message.
    Pushes a single NavSatFix messages into a hdf5 group.
    If the group does not contain the necessary attributes and datasets they will be created as well.
    """

    # Create datasets and attributes if necessary
    ensure_group_integrity(root_group)

    # Push new data to datasets. The datasets are resized accordingly
    append_empty_rows_to_datasets(root_group)
    fill_new_data_into_last_row(msg, root_group)


def ensure_group_integrity(root_group: h5py.Group):
    """Ensures the hdf5 group integrity.
    Creates required datasets and attributes if they are missing.
    """
    if not root_group.attrs.__contains__("type"):
        create_required_group_parameter(root_group)
        create_dataset_stamp(root_group)
        create_dataset_pose_position(root_group)
        create_dataset_pose_orientation(root_group)
        create_dataset_pose_covariance(root_group)
        create_dataset_twist_linear(root_group)
        create_dataset_twist_angular(root_group)
        create_dataset_twist_covariance(root_group)
        create_dataset_message_time(root_group)


def create_dataset_message_time(root_group: h5py.Group):
    """! Creates the message reception time dataset

    The dataset is initialised with all required and optional parameter provided in the SFB
    handbook.
    """
    root_group.create_dataset("message_time", (0, 1), np.float64, chunks=True,
                              maxshape=(None, 1))
    root_group['message_time'].attrs.create("name", "message creation time")
    root_group['message_time'].attrs.create("description", "time stamp in ns at wiche this "
                                                     "message was created")
    root_group['message_time'].attrs.create("dim", "[creation time]")
    root_group['message_time'].attrs.create("axis", "x")
    root_group['message_time'].attrs.create("unit", "ns")
    root_group['message_time'].attrs.create("quantity", "-")


def create_required_group_parameter(root_group: h5py.Group):
    """Create the required parameter for the hdf5 group.
    Parameter definitions are provided in the SFB handbook.
    """
    root_group.attrs.create("type", "TODO")
    root_group.attrs.create("description", "TODO")


def create_dataset_stamp(root_group: h5py.Group):
    """Creates the stamp dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("stamp", (0, 2), int, chunks=True, maxshape=(None, 2))
    root_group['stamp'].attrs.create("name", "TODO")
    root_group['stamp'].attrs.create("description", "TODO")
    root_group['stamp'].attrs.create("dim", "TODO")
    root_group['stamp'].attrs.create("axis", "TODO")
    root_group['stamp'].attrs.create("unit", "TODO")
    root_group['stamp'].attrs.create("quantity", "TODO")


def create_dataset_pose_position(root_group):
    """Creates the pose position dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("pose_position", (0, 3), float, chunks=True, maxshape=(None, 3))
    root_group['pose_position'].attrs.create("name", "TODO")
    root_group['pose_position'].attrs.create("description", "TODO")
    root_group['pose_position'].attrs.create("dim", "TODO")
    root_group['pose_position'].attrs.create("axis", "TODO")
    root_group['pose_position'].attrs.create("unit", "TODO")
    root_group['pose_position'].attrs.create("quantity", "TODO")


def create_dataset_pose_orientation(root_group: h5py.Group):
    """Creates the pose orientation dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("pose_orientation", (0, 3), float, chunks=True, maxshape=(None, 3))
    root_group['pose_orientation'].attrs.create("name", "TODO")
    root_group['pose_orientation'].attrs.create("description", "TODO")
    root_group['pose_orientation'].attrs.create("dim", "TODO")
    root_group['pose_orientation'].attrs.create("axis", "TODO")
    root_group['pose_orientation'].attrs.create("unit", "TODO")
    root_group['pose_orientation'].attrs.create("quantity", "TODO")


def create_dataset_pose_covariance(root_group: h5py.Group):
    """Creates the pose covariance dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("pose_covariance", (0, 36), float, chunks=True, maxshape=(None, 36))
    root_group['pose_covariance'].attrs.create("name", "TODO")
    root_group['pose_covariance'].attrs.create("description", "TODO")
    root_group['pose_covariance'].attrs.create("dim", "TODO")
    root_group['pose_covariance'].attrs.create("axis", "TODO")
    root_group['pose_covariance'].attrs.create("unit", "TODO")
    root_group['pose_covariance'].attrs.create("quantity", "TODO")


def create_dataset_twist_linear(root_group: h5py.Group):
    """Creates the twist linear dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("twist_linear", (0, 3), float, chunks=True, maxshape=(None, 3))
    root_group['twist_linear'].attrs.create("name", "TODO")
    root_group['twist_linear'].attrs.create("description", "TODO")
    root_group['twist_linear'].attrs.create("dim", "TODO")
    root_group['twist_linear'].attrs.create("axis", "TODO")
    root_group['twist_linear'].attrs.create("unit", "TODO")
    root_group['twist_linear'].attrs.create("quantity", "TODO")


def create_dataset_twist_angular(root_group: h5py.Group):
    """Creates the twist angular dataset
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("twist_angular", (0, 3), float, chunks=True, maxshape=(None, 3))
    root_group['twist_angular'].attrs.create("name", "TODO")
    root_group['twist_angular'].attrs.create("description", "TODO")
    root_group['twist_angular'].attrs.create("dim", "TODO")
    root_group['twist_angular'].attrs.create("axis", "TODO")
    root_group['twist_angular'].attrs.create("unit", "TODO")
    root_group['twist_angular'].attrs.create("quantity", "TODO")


def create_dataset_twist_covariance(root_group: h5py.Group):
    """Creates the twist covariance dataset
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("twist_covariance", (0, 36), float, chunks=True, maxshape=(None, 36))
    root_group['twist_covariance'].attrs.create("name", "TODO")
    root_group['twist_covariance'].attrs.create("description", "TODO")
    root_group['twist_covariance'].attrs.create("dim", "TODO")
    root_group['twist_covariance'].attrs.create("axis", "TODO")
    root_group['twist_covariance'].attrs.create("unit", "TODO")
    root_group['twist_covariance'].attrs.create("quantity", "TODO")


def append_empty_rows_to_datasets(root_group):
    """Appends a single empty row to all datasets
    Resizes the buffer to make room for a new line of data.
    """
    number_of_messages = root_group['stamp'].len()

    root_group['stamp'].resize(number_of_messages + 1, axis=0)
    root_group['pose_position'].resize(number_of_messages + 1, axis=0)
    root_group['pose_orientation'].resize(number_of_messages + 1, axis=0)
    root_group['pose_covariance'].resize(number_of_messages + 1, axis=0)
    root_group['twist_linear'].resize(number_of_messages + 1, axis=0)
    root_group['twist_angular'].resize(number_of_messages + 1, axis=0)
    root_group['twist_covariance'].resize(number_of_messages + 1, axis=0)
    root_group['message_time'].resize(number_of_messages + 1, axis=0)


def fill_new_data_into_last_row(msg: nav_msgs.msg.Odometry, root_group: h5py.Group):
    """Pushed new data into the last field of the buffer
    Assumes that the buffer was resized already.
    If not, the last row will be overwritten.
    """
    root_group['stamp'][-1] = [[msg.header.stamp.sec, msg.header.stamp.nanosec]]
    root_group['pose_position'][-1] = [[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]]
    root_group['pose_orientation'][-1] = [
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]]
    root_group['pose_covariance'][-1] = [msg.pose.covariance.tolist()]
    root_group['twist_linear'][-1] = [
        [msg.twist.twist.linear.x, msg.twist.twist.twist.linear.y, msg.twist.twist.twist.linear.z]]
    root_group['twist_angular'][-1] = [
        [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]]
    root_group['twist_covariance'][-1] = [msg.twist.covariance.tolist()]
    root_group['message_time'][-1, 0] = timestamp
