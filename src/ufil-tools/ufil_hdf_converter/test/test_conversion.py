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

"""End-to-end test: write a bag with every supported type and convert it with the CLI."""

from geometry_msgs.msg import TransformStamped
import h5py
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from ufil_hdf_converter.cli import main
from ufil_hdf_converter.ros_conversion import CONVERTERS
from ufil_msgs.msg import Object, ObjectList


def _messages():
    imu = Imu()
    imu.orientation.w = 1.0
    imu.linear_acceleration.z = 9.81

    mag = MagneticField()
    mag.magnetic_field.x = 2e-5

    fix = NavSatFix(latitude=50.78, longitude=6.06, altitude=200.0)
    fix.header.stamp.sec = 7
    fix.status.status, fix.status.service = 0, 1

    odometry = Odometry()
    odometry.pose.pose.position.x = 3.0
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.y = 4.0
    odometry.pose.covariance[0] = 0.5

    cloud = PointCloud2(height=1, width=42)

    grid = OccupancyGrid()
    grid.info.width, grid.info.height, grid.info.resolution = 3, 2, 0.5
    grid.data = [-1, 0, 100, 0, 50, 0]

    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0

    obj = Object(id=17, existence_probability=0.9)
    obj.state.state.x, obj.state.state.v_y, obj.state.state.yaw = 1.0, 2.0, 0.5
    obj.state.covariance[9] = 0.25
    obj.state.covariance[63] = 0.75
    obj.dimension.dimension.length = 4.5
    obj.dimension.covariance[8] = 0.1
    obj.classification.classification[0] = 1.0
    objects = ObjectList(objects=[obj])
    objects.header.stamp.nanosec = 500

    return [
        ('/imu', 'sensor_msgs/msg/Imu', imu),
        ('/mag', 'sensor_msgs/msg/MagneticField', mag),
        ('/fix', 'sensor_msgs/msg/NavSatFix', fix),
        ('/odom', 'nav_msgs/msg/Odometry', odometry),
        ('/cloud', 'sensor_msgs/msg/PointCloud2', cloud),
        ('/map', 'nav_msgs/msg/OccupancyGrid', grid),
        ('/tf', 'tf2_msgs/msg/TFMessage', TFMessage(transforms=[transform])),
        ('/objects', 'ufil_msgs/msg/ObjectList', objects),
        ('/chatter', 'std_msgs/msg/String', String(data='unsupported')),
    ]


@pytest.fixture
def bag(tmp_path):
    path = tmp_path / 'bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for index, (topic, msg_type, msg) in enumerate(_messages()):
        writer.create_topic(rosbag2_py.TopicMetadata(index, topic, msg_type, 'cdr'))
        writer.write(topic, serialize_message(msg), 1000 + index)
    del writer
    return path


def test_test_bag_covers_all_supported_types():
    assert {msg_type for _, msg_type, _ in _messages()} >= CONVERTERS.keys()


def test_converts_every_supported_type(bag, tmp_path):
    output = tmp_path / 'out.hdf5'
    assert main([str(bag), '-o', str(output), '-d', 'unit test']) == 0

    with h5py.File(output, 'r') as file:
        assert file.attrs['description'] == 'unit test'
        assert file['imu/imu/linear_acceleration'][0].tolist() == [0.0, 0.0, 9.81]
        assert file['imu/imu/message_time'][0, 0] == 1000
        assert file['mag/mag/magnetic_field'][0, 0] == pytest.approx(2e-5)
        assert file['navsatfix/fix/position'][0].tolist() == [50.78, 6.06, 200.0]
        assert file['navsatfix/fix/gnss_time'][0].tolist() == [7, 0]
        assert file['navsatfix/fix/status'][0].tolist() == [0, 1]
        assert file['odometry/odom/pose_orientation'][0].tolist() == [0.0, 0.0, 0.0, 1.0]
        assert file['odometry/odom/twist_linear'][0].tolist() == [0.0, 4.0, 0.0]
        assert file['odometry/odom/pose_covariance'][0, 0] == 0.5
        assert file['pointcloud/cloud/number_of_points'][0, 0] == 42
        assert file['occgrid/map/value'][0].tolist() == [[-1, 0, 100], [0, 50, 0]]
        assert file['tf/map__to__base_link/trans'][0].tolist() == [1.0, 0.0, 0.0]
        assert file['objects/objects/number_of_objects'][0, 0] == 1
        assert file['objects/objects/header_time'][0, 0] == 500

        obj = file['objects/objects/17']
        assert obj['state/position'][0].tolist() == [1.0, 0.0, 0.0]
        assert obj['state/linear_velocity'][0].tolist() == [0.0, 2.0, 0.0]
        assert obj['state/position_variance'][0].tolist() == [0.0, 0.25, -1.0]
        assert obj['state/angular_velocity_variance'][0].tolist() == [-1.0, -1.0, 0.75]
        assert obj['state/orientation'][0, 2] == 0.5
        assert obj['dimension/dimension_variance'][0, 2] == pytest.approx(0.1)
        assert obj['classification'][0, 0] == 1.0
        assert 'chatter' not in str(list(file.keys()))


def test_topic_filter(bag, tmp_path):
    output = tmp_path / 'out.hdf5'
    assert main([str(bag), '-o', str(output), '-t', '/imu', '/fix']) == 0
    with h5py.File(output, 'r') as file:
        assert sorted(file.keys()) == ['imu', 'navsatfix']

    assert main([str(bag), '-o', str(tmp_path / 'x.hdf5'), '-t', '/missing']) == 1


def test_existing_output_needs_force(bag, tmp_path):
    output = tmp_path / 'out.hdf5'
    output.write_bytes(b'')
    assert main([str(bag), '-o', str(output)]) == 2
    assert main([str(bag), '-o', str(output), '--force']) == 0


def test_missing_bag_fails(tmp_path):
    assert main([str(tmp_path / 'missing'), '-o', str(tmp_path / 'out.hdf5')]) == 1
