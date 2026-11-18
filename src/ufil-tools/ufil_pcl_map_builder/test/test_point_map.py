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

"""Unit tests of the ROS-independent point map."""

import xml.etree.ElementTree as ElementTree

import pytest
from ufil_pcl_map_builder.point_map import PointMap, PointMapConfig

ORIGIN = (50.78, 6.06)


def _tags(osm_file):
    return [{tag.get('k'): tag.get('v') for tag in node.iter('tag')}
            for node in ElementTree.parse(osm_file).getroot().iter('node')]


def test_points_are_written_with_type_tag(tmp_path):
    point_map = PointMap(PointMapConfig(*ORIGIN))
    assert point_map.add_point(1.0, 2.0, 'curb') == 1
    assert point_map.add_point(3.0, 4.0, 'sign') == 2

    output = tmp_path / 'map.osm'
    assert point_map.save(str(output)) == 2
    assert [tags['type'] for tags in _tags(output)] == ['curb', 'sign']


def test_point_at_origin_is_georeferenced_at_origin(tmp_path):
    point_map = PointMap(PointMapConfig(*ORIGIN))
    point_map.add_point(0.0, 0.0, 'tree')
    point_map.save(str(tmp_path / 'map.osm'))
    node = next(ElementTree.parse(tmp_path / 'map.osm').getroot().iter('node'))
    assert float(node.get('lat')) == pytest.approx(ORIGIN[0])
    assert float(node.get('lon')) == pytest.approx(ORIGIN[1])


def test_empty_map_writes_nothing(tmp_path):
    assert PointMap(PointMapConfig(*ORIGIN)).save(str(tmp_path / 'map.osm')) == 0
    assert not (tmp_path / 'map.osm').exists()


def test_unknown_point_type_is_rejected():
    with pytest.raises(ValueError, match='unknown point type'):
        PointMap(PointMapConfig(*ORIGIN)).add_point(0.0, 0.0, 'lamp')


@pytest.mark.parametrize('config', [
    PointMapConfig(91.0, 0.0),
    PointMapConfig(0.0, -181.0),
    PointMapConfig(0.0, 0.0, ()),
    PointMapConfig(0.0, 0.0, ('curb', '')),
    PointMapConfig(0.0, 0.0, ('curb', 'curb')),
])
def test_invalid_config_is_rejected(config):
    with pytest.raises(ValueError):
        config.validate()
