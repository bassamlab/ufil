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
from collections.abc import Iterator
import datetime

from stonesoup.buffered_generator import BufferedGenerator
from stonesoup.reader import DetectionReader
from stonesoup.types.detection import Detection


class OnlineDetectionReader(DetectionReader):
    """Custom DetectionReader implementation to set new value in online applications. This implementation requires a
    synchronization of tracker and calling `set_value` to avoid a lack of values for the tracker."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__()
        self.next_value = None
        self.next_time = None

    def set_value(self, time, value) -> None:
        """Sets the new Detections for the tracker"""
        self.next_time = time
        self.next_value = value

    @BufferedGenerator.generator_method
    def detections_gen(self) -> Iterator[tuple[datetime.datetime, set[Detection]]]:
        """On request of the tracker, this method yields the set value"""
        time = self.next_time
        value = self.next_value
        self.next_time = None
        self.next_value = None
        yield time, value
