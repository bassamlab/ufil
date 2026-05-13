#!/usr/bin/env python

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


import progressbar
import os

import argparse
import h5py

import rosbag2_py
import rclpy.logging
import rclpy.serialization
import rosidl_runtime_py.utilities

from tf2_msgs.msg import TFMessage  # ensure dependency
from .tf_util import append_tf_transform
from datetime import datetime
from typing import Any, List
from .imu_util import flatten_imu
from .navsatfix_util import flatten_navsatfix
from .odometry_util import flatten_odometry
from .mag_util import flatten_magnetic_field

from .sensor_msgs_pointcloud2 import flatten_pointcloud2
from .ufil_msgs_object_list import flatten_object_list
from .ufil_msgs_object import flatten_object
from .occupancy_grid_util import flatten_occ_grid

def add_meta_data(file: h5py.File) -> None:
    today = datetime.today()
    file_name: str = file.filename
    if(file_name.count('/') > 0):
        file_name_split = file_name.split('/')
        file_name = file_name_split[len(file_name_split)-1]

    file.attrs.create("id", file_name.split(".")[0])
    file.attrs.create("author", "Simon Schäfer")
    file.attrs.create("contact", "schaefer@embedded.rwth-aachen.de")
    file.attrs.create("date.creation", today.strftime('%Y-%m-%d'))
    file.attrs.create("subproject", "B03")
    file.attrs.create("university", "RWTH")
    file.attrs.create("version", 1)
    file.attrs.create("generator", "Python")
    file.attrs.create("description", "This is a test description.")

    file.attrs.create("date.modification", today.strftime('%Y-%m-%d'))
    file.attrs.create("spatial_reference", "Local")

def flatten_msg(file: h5py.File, msg: Any, topic_name: str, topic_type: str, timestamp) -> None:
    if topic_type == "sensor_msgs/msg/Imu":
        root_group = file.require_group('imu' + topic_name)
        flatten_imu(msg, timestamp, root_group)
    elif topic_type == "sensor_msgs/msg/NavSatFix":
        root_group = file.require_group('navsatfix' + topic_name)
        flatten_navsatfix(msg, timestamp, root_group)
    elif topic_type == 'nav_msgs/msg/Odometry':
        root_group = file.require_group('odometry' + topic_name)
        flatten_odometry(msg, timestamp, root_group)
    elif topic_type == 'sensor_msgs/msg/MagneticField':
        root_group = file.require_group('mag' + topic_name)
        flatten_magnetic_field(msg, timestamp, root_group)
    elif topic_type == "sensor_msgs/msg/PointCloud2":
        root_group = file.require_group('pointcloud' + topic_name)
        flatten_pointcloud2(msg, timestamp, root_group)
    elif topic_type == 'ufil_msgs/msg/ObjectList':
        root_group = file.require_group('objects' + topic_name)
        flatten_object_list(msg, timestamp, root_group)
        msg_time: np.float_64 = msg.header.stamp.sec*1e9 + msg.header.stamp.nanosec;
        for object in msg.objects:
            sub_group = root_group.require_group(str(object.id))
            flatten_object(object, timestamp, msg_time, sub_group)
    elif topic_type == "nav_msgs/msg/OccupancyGrid":
        root_group = file.require_group('occgrid' + topic_name)
        flatten_occ_grid(root_group, msg, float(timestamp))
    elif topic_type == "tf2_msgs/msg/TFMessage":
        root_group = file.require_group('tf')
        tfmsg = msg
        for tfs in tfmsg.transforms:
            parent = tfs.header.frame_id.strip()
            child  = tfs.child_frame_id.strip()
            ts = tfs.header.stamp.sec + 1e-9 * tfs.header.stamp.nanosec
            tr = tfs.transform.translation
            q  = tfs.transform.rotation
            append_tf_transform(root_group, parent, child, ts, tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
    else:
        raise RuntimeError('Topic not supported: %s' % topic_type)

def bag2hdf5(fname: str, out_fname: str,  topic_filter=[]):
    rclpy.logging.get_logger('rosbag2hdf_converter').info(
        'Reading rosbag from %s.' % fname)
    rclpy.logging.get_logger('rosbag2hdf_converter').info(
        'Writing hdf5 to %s.' % out_fname)

    # Open rosbag
    metadata = rosbag2_py.Info().read_metadata(fname, "mcap")

    reader = rosbag2_py.SequentialReader()
    options = rosbag2_py.StorageOptions(fname, metadata.storage_identifier)
    converter = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader.open(options, converter)

    # Apply selected topics as a filter
    if topic_filter is not None and len(topic_filter) != 0:
        filter = rosbag2_py.StorageFilter(topic_filter)
        reader.set_filter(filter)
        topics = topic_filter
    else:
        topics_metadata = reader.get_all_topics_and_types()
        topics = [topic_meta.name for topic_meta in topics_metadata]

    number_of_message = 0
    topic_types = {}
    for topic in topics:
        for topic_info in metadata.topics_with_message_count:
            # print(topic_info.topic_metadata.name)
            if not (topic_info.topic_metadata.name == topic):
                continue

            topic_types[topic_info.topic_metadata.name] = topic_info.topic_metadata.type
            number_of_message = number_of_message + topic_info.message_count

    if number_of_message == 0 and topic_filter != None and len(topic_filter) != 0:
        raise RuntimeError(
            'Topic filter active but no topics found in rosbag.')

    bar_widget = ['Converting %s: ' %
                  fname, progressbar.Percentage(), progressbar.Bar()]
    bar = progressbar.ProgressBar(
        widgets=bar_widget, maxval=number_of_message).start()
    counter = 0
    try:
        with h5py.File(out_fname, mode='a') as out_hdf5:
            add_meta_data(out_hdf5)
            while reader.has_next():
                topic, rawdata, timestamp = reader.read_next()
                msg_type = rosidl_runtime_py.utilities.get_message(topic_types[topic])
                msg = rclpy.serialization.deserialize_message(rawdata, msg_type)
                flatten_msg(out_hdf5, msg, topic, topic_types[topic], timestamp)
                counter = counter+1
                bar.update(counter)

            out_hdf5.flush()
    except:
        os.unlink(out_fname)
        raise
    finally:
        bar.finish()


def check_if_input_file_exists_exit_on_error(filename: str):
    # Check if input file exists of not show error
    if not os.path.exists(filename):
        rclpy.logging.get_logger('rosbag2hdf_converter').error(
            'Rosbag %s not found. Is the path correct?' % filename)
        raise SystemExit(1)


def check_if_output_file_exists_exit_on_error(filename: str, force_flag: bool):
    if not force_flag and os.path.exists(filename):
        rclpy.logging.get_logger('rosbag2hdf_converter').error(
            'HDF5 %s already exist. Please remove the existing file or provide the -f flag to overwrite.' % args.filename)
        raise SystemExit(2)


def generate_outout_filename(key: str, output_filename: str, input_filename: str) -> str:
    date_string = datetime.today().strftime('%y%m%d')

    # Use key, if no key is set use the output flag, if not flag is set use the filename of the input file
    if key is not None:
        rclpy.logging.get_logger('rosbag2hdf_converter').info(
            'Key %s detected. Creating target file name from key.' % key)
        return "B03_SSC_%s_%s.hdf5" % (date_string, key)
    if output_filename is not None:
        rclpy.logging.get_logger('rosbag2hdf_converter').info(
            'Output filename %s detected. Creating output file name from file name.' % output_filename)
        return output_filename

    rclpy.logging.get_logger('rosbag2hdf_converter').info(
        'No option detected to set output file name. Creating target file name from input file name.')
    return input_filename + "/B03_SSC_%s_%s.hdf5" % (date_string, 'NOKEY')


def main(args=None):
    rclpy.init(args=args)

    # Get paramteter from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str, help="the .bag file")
    parser.add_argument('-o', '--out', type=str,
                        help="name of output file")
    parser.add_argument('-k', '--key', type=str,
                        help="key to create SFB output file name")
    parser.add_argument('-t', '--topic', nargs='+',
                        help="topic name to convert. defaults to all. "
                        "multiple may be specified.")
    parser.add_argument('-f', '--force',
                        help="allows to overwrite existing files.",
                        action='store_true')
    args = parser.parse_args()

    input_filename = args.filename
    output_filename = args.out
    name_key = args.key
    force_flag = args.force
    topics = args.topic

    # Process arguments
    check_if_input_file_exists_exit_on_error(input_filename)
    output_filename = generate_outout_filename(name_key, output_filename, input_filename)
    check_if_output_file_exists_exit_on_error(output_filename, force_flag)

    # Start conversion
    bag2hdf5(input_filename, output_filename, topics)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
