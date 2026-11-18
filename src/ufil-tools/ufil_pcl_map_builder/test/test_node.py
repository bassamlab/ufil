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

"""Tests of the ROS node: parameters, default.yaml and point handling."""

import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
import pytest
import rclpy
from rclpy.parameter import Parameter
from ufil_pcl_map_builder.pcl_map_builder_node import PclMapBuilderNode
import yaml


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(tmp_path, **overrides):
    parameters = {'output_file': str(tmp_path / 'map.osm'), **overrides}
    return PclMapBuilderNode(parameter_overrides=[
        Parameter(name, value=value) for name, value in parameters.items()])


def _click(node, x, y):
    msg = PointStamped()
    msg.point.x, msg.point.y = x, y
    node._on_point(msg)


def test_default_yaml_matches_declared_parameters(tmp_path):
    share = get_package_share_directory('ufil_pcl_map_builder')
    with open(os.path.join(share, 'config', 'default.yaml')) as file:
        defaults = yaml.safe_load(file)['/**']['ros__parameters']
    flat = {'origin.latitude': defaults['origin']['latitude'],
            'origin.longitude': defaults['origin']['longitude'],
            **{k: v for k, v in defaults.items() if k != 'origin'}}

    node = PclMapBuilderNode()
    declared = set(node.list_parameters([], 0).names)
    assert set(flat) == declared - {'use_sim_time', 'start_type_description_service'}
    for name, value in flat.items():
        assert node.get_parameter(name).value == value
    node.destroy_node()


def test_clicked_points_are_saved_with_current_type(tmp_path):
    node = _node(tmp_path)
    _click(node, 1.0, 2.0)
    node.select_point_type('3')
    _click(node, 3.0, 4.0)
    node.select_point_type('9')
    assert node.point_type == 'sign'
    assert len(node.point_map) == 2
    assert (tmp_path / 'map.osm').exists()
    node.destroy_node()


def test_map_is_saved_on_shutdown_without_autosave(tmp_path):
    node = _node(tmp_path, autosave=False)
    _click(node, 1.0, 2.0)
    assert not (tmp_path / 'map.osm').exists()
    node.destroy_node()
    assert (tmp_path / 'map.osm').exists()


def test_invalid_point_type_is_rejected_at_runtime(tmp_path):
    node = _node(tmp_path)
    result = node.set_parameters([Parameter('point_type', value='lamp')])[0]
    assert not result.successful
    assert node.point_type == 'curb'
    assert node.set_parameters([Parameter('point_type', value='tree')])[0].successful
    assert node.point_type == 'tree'
    node.destroy_node()


@pytest.mark.parametrize('overrides', [
    {'point_type': 'lamp'},
    {'origin.latitude': 100.0},
    {'point_types': ['curb', 'curb'], 'point_type': 'curb'},
    {'output_file': ''},
])
def test_invalid_startup_parameters_are_rejected(tmp_path, overrides):
    with pytest.raises(ValueError):
        _node(tmp_path, **overrides)
