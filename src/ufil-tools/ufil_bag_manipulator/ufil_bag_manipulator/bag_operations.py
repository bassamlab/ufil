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

"""
ROS-independent operations on streams of serialized bag messages.

The functions only see ``BagMessage`` tuples (topic, serialized payload, receive time in ns),
so they neither deserialize messages nor depend on rosbag2.
"""

from dataclasses import dataclass, field
import heapq
from typing import Dict, Iterable, Iterator, List, Mapping, NamedTuple, Optional, Sequence


class BagMessage(NamedTuple):
    """One serialized message as stored in a bag."""

    topic: str
    data: bytes
    timestamp: int
    """Receive time in nanoseconds."""


class TopicConflictError(ValueError):
    """The same topic name is used with different message types in the merged bags."""


@dataclass
class MergeStats:
    """Number of messages taken from each input of ``merge_streams``."""

    messages_per_input: List[int] = field(default_factory=list)

    @property
    def total(self) -> int:
        return sum(self.messages_per_input)


@dataclass
class TrimStats:
    """Outcome of ``trim_and_shift``. Timestamps are in nanoseconds."""

    start_timestamp: Optional[int] = None
    cutoff_timestamp: Optional[int] = None
    kept: int = 0
    dropped: int = 0


def merge_topic_types(topic_types: Sequence[Mapping[str, str]]) -> Dict[str, str]:
    """
    Combine the topic name to message type mappings of several bags.

    Raises ``TopicConflictError`` if a topic name appears with different types, because the
    merged bag could not be deserialized consistently.
    """
    merged: Dict[str, str] = {}
    for mapping in topic_types:
        for name, msg_type in mapping.items():
            known = merged.setdefault(name, msg_type)
            if known != msg_type:
                raise TopicConflictError(
                    f"Topic '{name}' has type '{known}' in one bag and '{msg_type}' in another.")
    return merged


def _tag(index: int, stream: Iterable[BagMessage]) -> Iterator[tuple]:
    for message in stream:
        yield index, message


def merge_streams(streams: Sequence[Iterable[BagMessage]],
                  stats: Optional[MergeStats] = None) -> Iterator[BagMessage]:
    """
    Merge time-ordered message streams into one stream ordered by receive time.

    On equal timestamps the message of the earlier stream comes first. Every stream must be
    sorted by timestamp, which holds for messages read sequentially from a bag.
    """
    if stats is not None:
        stats.messages_per_input = [0] * len(streams)
    tagged = [_tag(index, stream) for index, stream in enumerate(streams)]
    for index, message in heapq.merge(*tagged, key=lambda item: item[1].timestamp):
        if stats is not None:
            stats.messages_per_input[index] += 1
        yield message


def trim_and_shift(messages: Iterable[BagMessage], trim_ns: int, shift: bool = True,
                   stats: Optional[TrimStats] = None) -> Iterator[BagMessage]:
    """
    Drop all messages received within ``trim_ns`` after the first message.

    With ``shift`` the timestamps of the kept messages are moved so that the cutoff becomes
    time zero; otherwise they are kept unchanged.
    """
    if trim_ns < 0:
        raise ValueError(f'Trim duration must not be negative, got {trim_ns} ns.')
    stats = stats if stats is not None else TrimStats()
    for message in messages:
        if stats.start_timestamp is None:
            stats.start_timestamp = message.timestamp
            stats.cutoff_timestamp = message.timestamp + trim_ns
        if message.timestamp < stats.cutoff_timestamp:
            stats.dropped += 1
            continue
        stats.kept += 1
        if shift:
            message = message._replace(timestamp=message.timestamp - stats.cutoff_timestamp)
        yield message
