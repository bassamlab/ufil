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
ROS-independent HDF5 writer.

The writer appends ``Row``, ``Transform`` and ``GridFrame`` records to an open ``h5py.File``.
Converting messages into these records is the job of ``ros_conversion``.
"""

from collections import Counter
from dataclasses import dataclass, field
import datetime
import math
import os
from typing import Any, Mapping, Optional, Sequence, Tuple, Union

import h5py
import numpy as np

from .hdf_layout import GroupLayout


@dataclass(frozen=True)
class Row:
    """One message worth of values for a time series group."""

    group: str
    layout: GroupLayout
    values: Mapping[str, Any]


@dataclass(frozen=True)
class Transform:
    """One transform between two coordinate frames."""

    parent: str
    child: str
    stamp: float
    """Header time in seconds."""
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]
    """Quaternion as (x, y, z, w)."""


@dataclass(frozen=True)
class GridFrame:
    """One occupancy grid. ``cells`` holds the occupancy (-1 unknown, 0..100) per cell."""

    group: str
    receive_time: float
    """Bag receive time in nanoseconds."""
    resolution: float
    origin: Tuple[float, float]
    frame_id: str
    cells: np.ndarray
    """Array of shape (height, width)."""


Record = Union[Row, Transform, GridFrame]


@dataclass(frozen=True)
class FileMetadata:
    """File attributes required by the SFB data handbook."""

    author: str = 'Simon Schäfer'
    contact: str = 'schaefer@embedded.rwth-aachen.de'
    description: str = 'Data converted from a rosbag2 recording.'
    subproject: str = 'B03'
    university: str = 'RWTH'
    version: int = 1
    generator: str = 'ufil_hdf_converter'
    spatial_reference: str = 'Local'


@dataclass
class ConversionStats:
    """Number of converted and skipped messages, and which topics were skipped."""

    written: int = 0
    skipped: int = 0
    unsupported_topics: Counter = field(default_factory=Counter)
    """Skipped message count per (topic, type)."""


def write_metadata(file: h5py.File, metadata: FileMetadata,
                   date: Optional[datetime.date] = None) -> None:
    date_string = (date or datetime.date.today()).strftime('%Y-%m-%d')
    file.attrs['id'] = os.path.splitext(os.path.basename(file.filename))[0]
    file.attrs['author'] = metadata.author
    file.attrs['contact'] = metadata.contact
    file.attrs['date.creation'] = date_string
    file.attrs['date.modification'] = date_string
    file.attrs['subproject'] = metadata.subproject
    file.attrs['university'] = metadata.university
    file.attrs['version'] = metadata.version
    file.attrs['generator'] = metadata.generator
    file.attrs['description'] = metadata.description
    file.attrs['spatial_reference'] = metadata.spatial_reference


def _append_row(dataset: h5py.Dataset, value: Any) -> None:
    index = dataset.shape[0]
    dataset.resize(index + 1, axis=0)
    dataset[index] = value


class HdfWriter:
    """Appends records to an HDF5 file, creating groups and datasets on first use."""

    def __init__(self, file: h5py.File):
        self._file = file

    def write(self, record: Record) -> None:
        if isinstance(record, Row):
            self._write_row(record)
        elif isinstance(record, Transform):
            self._write_transform(record)
        elif isinstance(record, GridFrame):
            self._write_grid(record)
        else:
            raise TypeError(f'Unsupported record type {type(record).__name__}.')

    def write_all(self, records: Sequence[Record]) -> None:
        for record in records:
            self.write(record)

    def _write_row(self, row: Row) -> None:
        group = self._file.require_group(row.group)
        if 'type' not in group.attrs:
            group.attrs['type'] = row.layout.group_type
            group.attrs['description'] = row.layout.description
            for name, layout in row.layout.datasets.items():
                dataset = group.create_dataset(name, (0, *layout.shape), layout.dtype,
                                               chunks=True, maxshape=(None, *layout.shape))
                dataset.attrs.update(layout.attrs)
        missing = row.layout.datasets.keys() - row.values.keys()
        if missing:
            raise ValueError(f"Row for '{row.group}' misses values for {sorted(missing)}.")
        for name in row.layout.datasets:
            _append_row(group[name], row.values[name])

    def _write_transform(self, transform: Transform) -> None:
        group = self._file.require_group(f'tf/{transform.parent}__to__{transform.child}')
        if 'times' not in group:
            for name, size in (('xytheta', 3), ('quat', 4), ('trans', 3)):
                group.create_dataset(name, (0, size), np.float64, chunks=True,
                                     maxshape=(None, size))
            group.create_dataset('times', (0,), np.float64, chunks=True, maxshape=(None,))
            group['times'].attrs['unit'] = 's'
            group.attrs['parent_frame'] = transform.parent
            group.attrs['child_frame'] = transform.child
        tx, ty, _ = transform.translation
        _append_row(group['times'], transform.stamp)
        _append_row(group['xytheta'], (tx, ty, yaw_from_quaternion(*transform.rotation)))
        _append_row(group['quat'], transform.rotation)
        _append_row(group['trans'], transform.translation)

    def _write_grid(self, grid: GridFrame) -> None:
        group = self._file.require_group(grid.group)
        height, width = grid.cells.shape
        if 'value' not in group:
            group.create_dataset('times', (0,), np.float64, chunks=True, maxshape=(None,))
            group['times'].attrs['unit'] = 'ns'
            for name, dtype in (('value', np.int8), ('mask', np.uint8)):
                group.create_dataset(name, (0, height, width), dtype,
                                     maxshape=(None, height, width), chunks=(1, height, width),
                                     compression='gzip', shuffle=True)
        if group['value'].shape[1:] != (height, width):
            raise ValueError(
                f"Occupancy grid '{grid.group}' changed size from {group['value'].shape[1:]} "
                f'to {(height, width)}. Convert the parts with different sizes separately.')
        info = group.require_group('info')
        info.attrs.update({
            'resolution': grid.resolution, 'width': width, 'height': height,
            'origin_x': grid.origin[0], 'origin_y': grid.origin[1], 'frame_id': grid.frame_id})
        _append_row(group['value'], grid.cells.astype(np.int8))
        _append_row(group['mask'], (grid.cells > 0).astype(np.uint8))
        _append_row(group['times'], grid.receive_time)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
