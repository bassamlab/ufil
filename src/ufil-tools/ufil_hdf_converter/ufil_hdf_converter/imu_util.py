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


import sensor_msgs.msg
import h5py

import numpy as np

def flatten_imu(msg: sensor_msgs.msg.Imu, timestamp: np.float64, root_group: h5py.Group):
    """Conversion function for an IMU message.
    Pushes a single IMU messages into a hdf5 group.
    If the group does not contain the necessary attributes and datasets they will be created as well.
    """

    # Create datasets and attributes if necessary
    ensure_group_integrity(root_group)

    # Push new data to datasets. The datasets are resized accordingly
    append_empty_rows_to_datasets(root_group)
    fill_new_data_into_last_row(msg, timestamp, root_group)


def ensure_group_integrity(root_group: h5py.Group):
    """Ensures the hdf5 group integrity.
    Creates required datasets and attributes if they are missing.
    """
    if not root_group.attrs.__contains__("type"):
        create_required_group_parameter(root_group)
        create_dataset_orientation(root_group)
        create_dataset_angular_velocity(root_group)
        create_dataset_linear_acceleration(root_group)
        create_dataset_message_time(root_group)


def create_required_group_parameter(root_group: h5py.Group):
    """Create the required parameter for the hdf5 group.
    Parameter definitions are provided in the SFB handbook.
    """
    root_group.attrs.create("type", "experiment")
    root_group.attrs.create("description", "data recorded from an Inertial Measurement Unit (IMU)"
                            "mounted to some form of vehicle")


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
    root_group['message_time'].attrs.create("dim", "[message  time]")
    root_group['message_time'].attrs.create("axis", "x")
    root_group['message_time'].attrs.create("unit", "ns")
    root_group['message_time'].attrs.create("quantity", "-")


def create_dataset_orientation(root_group: h5py.Group):
    """Creates the orientation dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("orientation", (0, 4), float, chunks=True, maxshape=(None, 4))
    root_group['orientation'].attrs.create("name", "orientation")
    root_group['orientation'].attrs.create("description", "orientation of the sensor in quaternion")
    root_group['orientation'].attrs.create("dim", "[x quaternion, y quaternion, z quaternion, w quaternion]")
    root_group['orientation'].attrs.create("axis", "y")
    root_group['orientation'].attrs.create("unit", "quaternion units")
    root_group['orientation'].attrs.create("quantity", "orientation")

def create_dataset_angular_velocity(root_group: h5py.Group):
    """Creates the angular velocity dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("angular_velocity", (0, 3), float, chunks=True, maxshape=(None, 3))
    root_group['angular_velocity'].attrs.create("name", "angular velocity")
    root_group['angular_velocity'].attrs.create("description", "angular velocity ")
    root_group['angular_velocity'].attrs.create("dim", "[x angular velocity, y angular velocity, z angular velocity]")
    root_group['angular_velocity'].attrs.create("axis", "y")
    root_group['angular_velocity'].attrs.create("unit", "def/s")
    root_group['angular_velocity'].attrs.create("quantity", "angular velocity")


def create_dataset_linear_acceleration(root_group: h5py.Group):
    """Creates the linear acceleration dataset.
    The dataset is initialised with all required and optional parameter prmsg.linear_acceleration.xovided in the SFB handbook.
    """
    root_group.create_dataset("linear_acceleration", (0, 3), float, chunks=True, maxshape=(None, 3))
    root_group['linear_acceleration'].attrs.create("name", "acceleration")
    root_group['linear_acceleration'].attrs.create("description", "linear acceleration given at point in time")
    root_group['linear_acceleration'].attrs.create("dim", "[x acceleration, y acceleration, z acceleration]")
    root_group['linear_acceleration'].attrs.create("axis", "y")
    root_group['linear_acceleration'].attrs.create("unit", "m/s^2")
    root_group['linear_acceleration'].attrs.create("quantity", "acceleration")


def append_empty_rows_to_datasets(root_group):
    """Appends a single empty row to all datasets.
    Resizes the buffer to make room for a new line of data.
    """
    number_of_messages = root_group['orientation'].len()

    root_group['orientation'].resize(number_of_messages + 1, axis=0)
    root_group['angular_velocity'].resize(number_of_messages + 1, axis=0)
    root_group['linear_acceleration'].resize(number_of_messages + 1, axis=0)
    root_group['message_time'].resize(number_of_messages + 1, axis=0)


def fill_new_data_into_last_row(msg: sensor_msgs.msg.Imu, timestamp: np.float64, root_group: h5py.Group):
    """Pushed new data into the last field of the buffer.
    Assumes that the buffer was resized already.
    If not, the last row will be overwritten.
    """
    root_group['orientation'][-1] = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    root_group['angular_velocity'][-1] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    root_group['linear_acceleration'][-1] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
    root_group['message_time'][-1, 0] = timestamp

