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

"""rosbag2 adapter: reads and writes bags as streams of ``BagMessage``."""

import os
from typing import Dict, Iterable, Iterator, List, Sequence

import rosbag2_py

from .bag_operations import BagMessage, merge_streams, merge_topic_types, MergeStats, \
    trim_and_shift, TrimStats

DEFAULT_STORAGE_ID = 'mcap'

# Messages are copied without deserialization, so no conversion is requested.
_CONVERTER_OPTIONS = rosbag2_py.ConverterOptions('', '')


def open_reader(uri: str) -> rosbag2_py.SequentialReader:
    """Open a bag for reading. The storage plugin (mcap, sqlite3) is detected automatically."""
    if not os.path.exists(uri):
        raise FileNotFoundError(f"Bag '{uri}' does not exist.")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''), _CONVERTER_OPTIONS)
    return reader


def read_messages(reader: rosbag2_py.SequentialReader) -> Iterator[BagMessage]:
    while reader.has_next():
        yield BagMessage(*reader.read_next())


def open_writer(uri: str, storage_id: str,
                topics: Iterable[rosbag2_py.TopicMetadata]) -> rosbag2_py.SequentialWriter:
    """Create a new bag with the given topics. Refuses to overwrite an existing path."""
    if os.path.exists(uri):
        raise FileExistsError(f"Output '{uri}' already exists. Remove it or choose another path.")
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id), _CONVERTER_OPTIONS)
    for topic in topics:
        writer.create_topic(topic)
    return writer


def write_messages(writer: rosbag2_py.SequentialWriter, messages: Iterable[BagMessage]) -> None:
    for message in messages:
        writer.write(message.topic, message.data, message.timestamp)


def merge_bags(inputs: Sequence[str], output: str,
               storage_id: str = DEFAULT_STORAGE_ID) -> MergeStats:
    """Merge several bags into one bag ordered by receive time."""
    readers = [open_reader(uri) for uri in inputs]
    topic_lists = [reader.get_all_topics_and_types() for reader in readers]
    merge_topic_types([{t.name: t.type for t in topics} for topics in topic_lists])

    unique_topics: Dict[str, rosbag2_py.TopicMetadata] = {}
    for topics in topic_lists:
        for topic in topics:
            unique_topics.setdefault(topic.name, topic)

    writer = open_writer(output, storage_id, unique_topics.values())
    stats = MergeStats()
    write_messages(writer, merge_streams([read_messages(r) for r in readers], stats))
    # Closing the writer flushes and finalizes the bag metadata.
    del writer
    return stats


def trim_bag(input_uri: str, output: str, trim_seconds: float, shift: bool = True,
             storage_id: str = DEFAULT_STORAGE_ID) -> TrimStats:
    """Copy a bag without its first ``trim_seconds`` seconds, optionally shifting time to 0."""
    reader = open_reader(input_uri)
    topics: List[rosbag2_py.TopicMetadata] = reader.get_all_topics_and_types()
    writer = open_writer(output, storage_id, topics)
    stats = TrimStats()
    write_messages(writer, trim_and_shift(read_messages(reader), round(trim_seconds * 1e9),
                                          shift, stats))
    del writer
    return stats
