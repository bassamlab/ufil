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

"""Unit tests of the ROS-independent bag operations."""

import pytest
from ufil_bag_manipulator.bag_operations import BagMessage, merge_streams, \
    merge_topic_types, MergeStats, TopicConflictError, trim_and_shift, TrimStats


def _messages(topic, timestamps):
    return [BagMessage(topic, f'{topic}{t}'.encode(), t) for t in timestamps]


def test_merge_orders_by_timestamp_and_counts_per_input():
    stats = MergeStats()
    merged = list(merge_streams(
        [_messages('/a', [1, 4, 6]), _messages('/b', [2, 3, 7]), _messages('/c', [5])], stats))
    assert [m.timestamp for m in merged] == [1, 2, 3, 4, 5, 6, 7]
    assert stats.messages_per_input == [3, 3, 1]
    assert stats.total == 7


def test_merge_prefers_earlier_input_on_equal_timestamps():
    merged = list(merge_streams([_messages('/a', [5, 5]), _messages('/b', [5])]))
    assert [m.topic for m in merged] == ['/a', '/a', '/b']


def test_merge_handles_empty_inputs():
    assert list(merge_streams([[], _messages('/a', [1])])) == _messages('/a', [1])
    assert list(merge_streams([[], []])) == []


def test_merge_topic_types_unites_and_detects_conflicts():
    assert merge_topic_types([{'/a': 'x/msg/A'}, {'/a': 'x/msg/A', '/b': 'x/msg/B'}]) == {
        '/a': 'x/msg/A', '/b': 'x/msg/B'}
    with pytest.raises(TopicConflictError, match="'/a'"):
        merge_topic_types([{'/a': 'x/msg/A'}, {'/a': 'x/msg/B'}])


def test_trim_drops_start_and_shifts_cutoff_to_zero():
    stats = TrimStats()
    kept = list(trim_and_shift(_messages('/a', [100, 150, 200, 250]), 100, stats=stats))
    assert [m.timestamp for m in kept] == [0, 50]
    assert [m.data for m in kept] == [b'/a200', b'/a250']
    assert (stats.start_timestamp, stats.cutoff_timestamp) == (100, 200)
    assert (stats.kept, stats.dropped) == (2, 2)


def test_trim_without_shift_keeps_timestamps():
    kept = list(trim_and_shift(_messages('/a', [100, 150, 200]), 50, shift=False))
    assert [m.timestamp for m in kept] == [150, 200]


def test_trim_zero_shifts_first_message_to_zero():
    kept = list(trim_and_shift(_messages('/a', [100, 150]), 0))
    assert [m.timestamp for m in kept] == [0, 50]


def test_trim_of_empty_stream_reports_no_start():
    stats = TrimStats()
    assert list(trim_and_shift([], 10, stats=stats)) == []
    assert stats.start_timestamp is None


def test_trim_rejects_negative_duration():
    with pytest.raises(ValueError):
        list(trim_and_shift(_messages('/a', [1]), -1))
