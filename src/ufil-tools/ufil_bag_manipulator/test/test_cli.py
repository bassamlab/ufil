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

"""End-to-end tests of the ufil_bag_manipulator command on real rosbag2 bags."""

from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from std_msgs.msg import String
from ufil_bag_manipulator.cli import main


def _write_bag(path, storage_id, topic, messages):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(0, topic, 'std_msgs/msg/String', 'cdr'))
    for timestamp, text in messages:
        writer.write(topic, serialize_message(String(data=text)), timestamp)


def _read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    messages = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        messages.append((topic, deserialize_message(data, String).data, timestamp))
    return topics, messages


def test_merge_combines_bags_of_different_storage(tmp_path):
    _write_bag(tmp_path / 'a', 'mcap', '/a', [(10, 'a1'), (30, 'a2')])
    _write_bag(tmp_path / 'b', 'sqlite3', '/b', [(20, 'b1')])

    assert main(['merge', str(tmp_path / 'a'), str(tmp_path / 'b'),
                 '-o', str(tmp_path / 'out')]) == 0

    topics, messages = _read_bag(tmp_path / 'out')
    assert topics == {'/a': 'std_msgs/msg/String', '/b': 'std_msgs/msg/String'}
    assert messages == [('/a', 'a1', 10), ('/b', 'b1', 20), ('/a', 'a2', 30)]


def test_trim_drops_start_and_shifts(tmp_path):
    second = 1_000_000_000
    _write_bag(tmp_path / 'in', 'mcap', '/a',
               [(5 * second, 'drop'), (6 * second, 'keep1'), (7 * second, 'keep2')])

    assert main(['trim', str(tmp_path / 'in'), '-o', str(tmp_path / 'out'),
                 '--trim', '0.5']) == 0

    _, messages = _read_bag(tmp_path / 'out')
    assert messages == [('/a', 'keep1', second // 2), ('/a', 'keep2', 3 * second // 2)]


def test_refuses_existing_output_and_missing_input(tmp_path):
    _write_bag(tmp_path / 'in', 'mcap', '/a', [(1, 'x')])
    (tmp_path / 'out').mkdir()
    assert main(['trim', str(tmp_path / 'in'), '-o', str(tmp_path / 'out')]) == 1
    assert main(['trim', str(tmp_path / 'missing'), '-o', str(tmp_path / 'new')]) == 1
    assert main(['merge', str(tmp_path / 'in'), '-o', str(tmp_path / 'new')]) == 1
