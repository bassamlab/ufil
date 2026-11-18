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

"""ROS-independent map of typed landmark points, stored as Lanelet2 OSM file."""

from dataclasses import dataclass
from typing import List, Sequence

import lanelet2.core as l2_core
import lanelet2.io as l2_io
import lanelet2.projection as l2_proj


@dataclass(frozen=True)
class PointMapConfig:
    """Configuration of a ``PointMap``."""

    origin_latitude: float
    origin_longitude: float
    point_types: Sequence[str] = ('curb', 'tree', 'sign')

    def validate(self) -> None:
        """Raise ValueError if the configuration is invalid."""
        if not -90.0 <= self.origin_latitude <= 90.0:
            raise ValueError(f'origin latitude {self.origin_latitude} is outside [-90, 90].')
        if not -180.0 <= self.origin_longitude <= 180.0:
            raise ValueError(
                f'origin longitude {self.origin_longitude} is outside [-180, 180].')
        if not self.point_types:
            raise ValueError('at least one point type is required.')
        if any(not point_type for point_type in self.point_types):
            raise ValueError('point types must not be empty strings.')
        if len(set(self.point_types)) != len(self.point_types):
            raise ValueError(f'point types {list(self.point_types)} contain duplicates.')


class PointMap:
    """
    Collects points given in a local metric frame and writes them as OSM nodes.

    The local frame is a UTM projection centered at the configured origin. Every point carries
    its type in the ``type`` tag.
    """

    def __init__(self, config: PointMapConfig):
        config.validate()
        self._config = config
        self._projector = l2_proj.UtmProjector(
            l2_io.Origin(config.origin_latitude, config.origin_longitude))
        self._map = l2_core.LaneletMap()
        self._points: List[l2_core.Point3d] = []

    @property
    def point_types(self) -> Sequence[str]:
        return self._config.point_types

    def __len__(self) -> int:
        return len(self._points)

    def add_point(self, x: float, y: float, point_type: str) -> int:
        """Add a point at ground level and return its id."""
        if point_type not in self._config.point_types:
            raise ValueError(
                f"unknown point type '{point_type}', expected one of "
                f'{list(self._config.point_types)}.')
        point = l2_core.Point3d(len(self._points) + 1, x, y, 0.0)
        point.attributes['type'] = point_type
        self._map.add(point)
        self._points.append(point)
        return point.id

    def save(self, path: str) -> int:
        """Write all points to an OSM file and return the number of written points."""
        if self._points:
            l2_io.write(path, self._map, self._projector)
        return len(self._points)
