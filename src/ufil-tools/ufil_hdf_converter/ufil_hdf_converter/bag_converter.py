# Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

"""Reads a rosbag2 bag and writes its supported messages into an HDF5 file."""

import os
from typing import Callable, Optional, Sequence

import h5py
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from .hdf_writer import ConversionStats, FileMetadata, HdfWriter, write_metadata
from .ros_conversion import CONVERTERS, to_records

ProgressCallback = Callable[[int, int], None]
"""Called with (processed messages, total messages)."""


def convert_bag(bag: str, output: str, topics: Optional[Sequence[str]] = None,
                metadata: FileMetadata = FileMetadata(),
                on_progress: Optional[ProgressCallback] = None) -> ConversionStats:
    """
    Convert all messages of a bag, or only those on ``topics``, into a new HDF5 file.

    Messages of unsupported types are skipped and counted in the returned statistics.
    """
    if not os.path.exists(bag):
        raise FileNotFoundError(f"Bag '{bag}' does not exist.")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))

    message_counts = {info.topic_metadata.name: info.message_count
                      for info in reader.get_metadata().topics_with_message_count}
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topics:
        missing = sorted(set(topics) - topic_types.keys())
        if missing:
            raise ValueError(f"Topics {missing} are not in bag '{bag}'.")
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
        selected = set(topics)
    else:
        selected = set(topic_types)
    total = sum(message_counts.get(topic, 0) for topic in selected)

    message_classes = {topic: get_message(topic_types[topic]) for topic in selected
                       if topic_types[topic] in CONVERTERS}
    stats = ConversionStats()
    processed = 0
    with h5py.File(output, mode='w') as file:
        write_metadata(file, metadata)
        writer = HdfWriter(file)
        while reader.has_next():
            topic, data, receive_time = reader.read_next()
            processed += 1
            message_class = message_classes.get(topic)
            if message_class is None:
                stats.skipped += 1
                stats.unsupported_topics[(topic, topic_types[topic])] += 1
            else:
                msg = deserialize_message(data, message_class)
                writer.write_all(to_records(msg, topic, topic_types[topic], receive_time))
                stats.written += 1
            if on_progress is not None:
                on_progress(processed, total)
    return stats
