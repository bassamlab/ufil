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


def flatten_navsatfix(msg: sensor_msgs.msg.NavSatFix, timestamp: np.float64, root_group: h5py.Group):
    """Conversion function for a NavSatFix message.
    Pushes a single NavSatFix messages into a hdf5 group.
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
        create_dataset_gnss_time(root_group)
        create_dataset_status(root_group)
        create_dataset_position(root_group)
        create_dataset_position_covariance(root_group)
        create_dataset_position_covariance_type(root_group)
        create_dataset_message_time(root_group)


def create_required_group_parameter(root_group: h5py.Group):
    """Create the required parameter for the hdf5 group.
    Parameter definitions are provided in the SFB handbook.
    """
    root_group.attrs.create("type", "experiment")
    root_group.attrs.create("description", "data recorded from an Global Navigation Satellite System (GNSS)"
                            "mounted to some form of vehicle")

def create_dataset_message_time(root_group: h5py.Group):
    """! Creates the message reception time dataset

    The dataset is initialised with all required and optional parameter provided in the SFB
    handbook.
    """
    root_group.create_dataset("message_time", (0, 1), np.float64, chunks=True,
                              maxshape=(None, 1))
    root_group['message_time'].attrs.create("name", "message time")
    root_group['message_time'].attrs.create("description", "time stamp in ns at wiche this "
                                                     "message was created")
    root_group['message_time'].attrs.create("dim", "[message time]")
    root_group['message_time'].attrs.create("axis", "x")
    root_group['message_time'].attrs.create("unit", "ns")
    root_group['message_time'].attrs.create("quantity", "-")

def create_dataset_gnss_time(root_group: h5py.Group):
    """Creates the stamp dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("gnss_time", (0, 2), int, chunks=True, maxshape=(None, 2))
    root_group['gnss_time'].attrs.create("name", "gnns time stamp")
    root_group['gnss_time'].attrs.create("description", "time stamp in ns provided by gnss satellites")
    root_group['gnss_time'].attrs.create("dim", "[gnss time]]")
    root_group['gnss_time'].attrs.create("axis", "x")
    root_group['gnss_time'].attrs.create("unit", "ns")
    root_group['gnss_time'].attrs.create("quantity", "-")


def create_dataset_status(root_group: h5py.Group):
    """Creates the status dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("status", (0, 2), int, chunks=True, maxshape=(None, 2))
    root_group['status'].attrs.create("name", "status of gnss")
    root_group['status'].attrs.create("description", "status of gnss including the number of satellite available")
    root_group['status'].attrs.create("dim", "[status, number of satellites]")
    root_group['status'].attrs.create("axis", "y")
    root_group['status'].attrs.create("unit", "[none, count]]")
    root_group['status'].attrs.create("quantity", "[enum, count]]")


def create_dataset_position(root_group: h5py.Group):
    """Creates the longitude dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("position", (0, 3), float, chunks=True, maxshape=(None, 3))
    root_group['position'].attrs.create("name", "position")
    root_group['position'].attrs.create("description", "position of gnss receiver")
    root_group['position'].attrs.create("dim", "[latitude, longitude, altitude]")
    root_group['position'].attrs.create("axis", "x, y")
    root_group['position'].attrs.create("unit", "[deg, deg, m]")
    root_group['position'].attrs.create("quantity", "position")


def create_dataset_position_covariance(root_group: h5py.Group):
    """Creates the position covariance dataset.
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("position_covariance", (0, 9), float, chunks=True, maxshape=(None, 9))
    root_group['position_covariance'].attrs.create("name", "covariance of position")
    root_group['position_covariance'].attrs.create("description", "covariance of position measurements")
    root_group['position_covariance'].attrs.create("dim", "row major covariance matrix/dilution of precision (DOP) on first value")
    root_group['position_covariance'].attrs.create("axis", "-")
    root_group['position_covariance'].attrs.create("unit", "covariance")
    root_group['position_covariance'].attrs.create("quantity", "-")


def create_dataset_position_covariance_type(root_group: h5py.Group):
    """Creates the position covariance type dataset
    The dataset is initialised with all required and optional parameter provided in the SFB handbook.
    """
    root_group.create_dataset("position_covariance_type", (0, 1), np.uint8, chunks=True, maxshape=(None, 1))
    root_group['position_covariance_type'].attrs.create("name", "type of position covariance")
    root_group['position_covariance_type'].attrs.create("description", "indicates of ow major covariance matrix or dilution of precision (DOP) on first value is provided")
    root_group['position_covariance_type'].attrs.create("dim", "[type]")
    root_group['position_covariance_type'].attrs.create("axis", "y")
    root_group['position_covariance_type'].attrs.create("unit", "[enum]]")
    root_group['position_covariance_type'].attrs.create("quantity", "-")


def append_empty_rows_to_datasets(root_group):
    """Appends a single empty row to all datasets
    Resizes the buffer to make room for a new line of data.
    """
    number_of_messages = root_group['gnss_time'].len()

    root_group['gnss_time'].resize(number_of_messages + 1, axis=0)
    root_group['status'].resize(number_of_messages + 1, axis=0)
    root_group['position'].resize(number_of_messages + 1, axis=0)
    root_group['position_covariance'].resize(number_of_messages + 1, axis=0)
    root_group['position_covariance_type'].resize(number_of_messages + 1, axis=0)
    root_group['message_time'].resize(number_of_messages + 1, axis=0)


def fill_new_data_into_last_row(msg: sensor_msgs.msg.NavSatFix, timestamp: np.float64, root_group: h5py.Group):
    """Pushed new data into the last field of the buffer
    Assumes that the buffer was resized already.
    If not, the last row will be overwritten.
    """
    root_group['gnss_time'][-1] = [[msg.header.stamp.sec, msg.header.stamp.nanosec]]
    root_group['status'][-1] = [[msg.status.status, msg.status.service]]
    root_group['position'][-1] = [msg.latitude, msg.longitude, msg.altitude]
    root_group['position_covariance'][-1] = msg.position_covariance
    root_group['position_covariance_type'][-1] = msg.position_covariance_type
    root_group['message_time'][-1, 0] = timestamp
