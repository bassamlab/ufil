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

"""Unit tests of the ROS-independent HDF5 writer and output naming."""

import datetime
import math

import h5py
import numpy as np
import pytest
from ufil_hdf_converter import hdf_layout
from ufil_hdf_converter.cli import output_filename
from ufil_hdf_converter.hdf_writer import FileMetadata, GridFrame, HdfWriter, Row, \
    Transform, write_metadata, yaw_from_quaternion


@pytest.fixture
def h5file(tmp_path):
    with h5py.File(tmp_path / 'B03_test.hdf5', 'w') as file:
        yield file


def _mag_row(value, time):
    return Row('mag/mag', hdf_layout.MAGNETIC_FIELD,
               {'magnetic_field': (value, 0.0, 0.0), 'message_time': time})


def test_rows_create_group_once_and_append(h5file):
    writer = HdfWriter(h5file)
    writer.write_all([_mag_row(1.0, 10), _mag_row(2.0, 20)])

    group = h5file['mag/mag']
    assert group.attrs['type'] == 'experiment'
    assert group['magnetic_field'].shape == (2, 3)
    assert group['message_time'][:, 0].tolist() == [10.0, 20.0]
    assert group['magnetic_field'][1, 0] == 2.0
    assert group['magnetic_field'].attrs['unit'] == 'Tesla'


def test_every_layout_accepts_a_complete_row(h5file):
    writer = HdfWriter(h5file)
    for name, layout in vars(hdf_layout).items():
        if isinstance(layout, hdf_layout.GroupLayout):
            values = {key: np.zeros(spec.shape) for key, spec in layout.datasets.items()}
            writer.write(Row(name, layout, values))
            assert all(h5file[name][key].shape[0] == 1 for key in layout.datasets)


def test_row_with_missing_values_is_rejected(h5file):
    with pytest.raises(ValueError, match='message_time'):
        HdfWriter(h5file).write(
            Row('mag/x', hdf_layout.MAGNETIC_FIELD, {'magnetic_field': (0, 0, 0)}))


def test_transform_is_stored_with_yaw(h5file):
    yaw = 0.3
    rotation = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    HdfWriter(h5file).write(Transform('map', 'base', 1.5, (1.0, 2.0, 3.0), rotation))

    group = h5file['tf/map__to__base']
    assert group.attrs['parent_frame'] == 'map'
    assert group['times'][:].tolist() == [1.5]
    assert group['xytheta'][0] == pytest.approx([1.0, 2.0, yaw])
    assert group['trans'][0].tolist() == [1.0, 2.0, 3.0]
    assert yaw_from_quaternion(*rotation) == pytest.approx(yaw)


def test_grid_frames_are_stacked_and_size_changes_rejected(h5file):
    writer = HdfWriter(h5file)
    cells = np.array([[-1, 0], [50, 100]], dtype=np.int8)
    writer.write(GridFrame('occgrid/map', 5.0, 0.1, (1.0, 2.0), 'map', cells))
    writer.write(GridFrame('occgrid/map', 6.0, 0.1, (1.0, 2.0), 'map', cells))

    group = h5file['occgrid/map']
    assert group['value'].shape == (2, 2, 2)
    assert group['mask'][0].tolist() == [[0, 0], [1, 1]]
    assert group['info'].attrs['width'] == 2
    with pytest.raises(ValueError, match='changed size'):
        writer.write(GridFrame('occgrid/map', 7.0, 0.1, (0, 0), 'map', np.zeros((3, 2))))


def test_metadata_attributes(h5file):
    write_metadata(h5file, FileMetadata(description='test run'), datetime.date(2026, 1, 2))
    assert h5file.attrs['id'] == 'B03_test'
    assert h5file.attrs['description'] == 'test run'
    assert h5file.attrs['date.creation'] == '2026-01-02'


def test_output_filename(tmp_path):
    day = datetime.date(2026, 1, 2)
    assert output_filename(str(tmp_path), 'x.hdf5', None, day) == 'x.hdf5'
    assert output_filename(str(tmp_path), None, 'run1', day) == 'B03_SSC_260102_run1.hdf5'
    assert output_filename(str(tmp_path), None, None, day) == str(
        tmp_path / 'B03_SSC_260102_NOKEY.hdf5')
    assert output_filename(str(tmp_path / 'bag.mcap'), None, None, day) == str(
        tmp_path / 'B03_SSC_260102_NOKEY.hdf5')
