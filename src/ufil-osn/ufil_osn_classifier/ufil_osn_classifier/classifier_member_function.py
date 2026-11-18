# Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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


"""ROS 2 node fusing camera-based YOLO classifications into the OSN object list."""

import copy
import os
import traceback

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Transform
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ufil_msgs.msg import ObjectList
from ufil_osn_classifier.classifier_projection import to_cam
from ufil_osn_classifier.conf_fusion import conf_fusion, ConfidenceStorage
from ufil_osn_classifier.matrix_parameter import parse_matrix
from ultralytics import YOLO

PACKAGE_NAME = 'ufil_osn_classifier'

DEFAULT_T_CAMERA_RIGHT_LIDAR = """[
            [-0.26207, 0.96484, 0.01995, -0.29462],
            [0.07057, -0.00146, 0.99751, 0.20912],
            [0.96247, 0.26282, -0.06771, 0.97677],
            [0.00000, 0.00000, 0.00000, 1.00000]
        ]"""

DEFAULT_T_CAMERA_LEFT_LIDAR = """[
            [0.26207, 0.96484, -0.01995, -0.29462],
            [0.07056, 0.00146, 0.99751, 0.20912],
            [0.96247, -0.26283, -0.06770, 0.97677],
            [0.00000, 0.00000, 0.00000, 1.00000]
        ]"""

# Distance in pixels between a projected track and a YOLO box center to match them.
ASSIGNMENT_THRESHOLD = 80


def default_model_file():
    """Return the path of the YOLO TensorRT engine installed with this package."""
    return os.path.join(get_package_share_directory(PACKAGE_NAME), 'models', 'yolo11s.engine')


class YoloClassifier(Node):
    """
    ROS 2 node that refines the classification of tracks using YOLO on camera images.

    Subscribes to the object list of the OSN tracker and to compressed camera images and
    camera infos. Projects every track into the images, matches it with the YOLO
    detections and fuses the camera classification with the tracker classification.
    Publishes the annotated images and the fused object list.
    """

    def __init__(self):
        """Declare the parameters, load the YOLO model and set up the ROS interfaces."""
        super().__init__('yolo_classifier')

        # Declare parameters for input and output topics and camera info
        self.track_input = self.declare_parameter(
            'track_input', 'osn/object_list').get_parameter_value().string_value
        self.track_output = self.declare_parameter(
            'track_output', 'osn/classified_object_list').get_parameter_value().string_value

        self.declare_parameter(
            'camera_input_topics',
            ['/camera_right/image_rect/compressed', '/camera_left/image_rect/compressed'])
        self.declare_parameter(
            'camera_output_topics',
            ['/annot_right/image_rect/compressed', '/annot_left/image_rect/compressed'])
        self.declare_parameter(
            'camera_info_topics', ['/camera_right/camera_info', '/camera_left/camera_info'])

        self.target_frame = self.declare_parameter(
            'target_frame', 'm1').get_parameter_value().string_value

        self.declare_parameter('tracker_trust', 0.2)
        self.declare_parameter('camera_trust', 0.4)
        self.declare_parameter('smoothing_factor', 0.1)
        self.declare_parameter('model_file', default_model_file())
        self.declare_parameter('task', 'classify')

        self.declare_parameter('T_camera_right_lidar', DEFAULT_T_CAMERA_RIGHT_LIDAR)
        self.declare_parameter('T_camera_left_lidar', DEFAULT_T_CAMERA_LEFT_LIDAR)

        # Initialize camera parameters
        self.cameras = [
            {
                'object_list': ObjectList(),
                'frame': 'camera_right_image',
                'transform': Transform,
                'image': CompressedImage,
                'distortion': [],
                'intrinsic': [],
                'projection': [],
                'camera_info': CameraInfo,
                'T_camera_lidar': self.get_matrix_parameter('T_camera_right_lidar'),
            },
            {
                'object_list': ObjectList(),
                'frame': 'camera_left_image',
                'transform': Transform,
                'image': CompressedImage,
                'distortion': [],
                'intrinsic': [],
                'projection': [],
                'camera_info': CameraInfo,
                'T_camera_lidar': self.get_matrix_parameter('T_camera_left_lidar'),
            }
        ]

        # YOLO classes to detect
        self.yolo_classes = [0, 1, 2, 3, 5, 7, 15, 16, 17, 36]

        # Initialize confidence storage
        self.conf_storage = ConfidenceStorage()

        # Initialize CvBridge and YOLO model
        self.br = CvBridge()
        model_file = self.get_parameter('model_file').get_parameter_value().string_value
        task = self.get_parameter('task').get_parameter_value().string_value
        self.get_logger().info(f'Loading YOLO model {model_file} (task: {task})')
        self.yolo = YOLO(model_file, task)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscriptions for object list, images, and camera info
        self.object_subscription = self.create_subscription(
            ObjectList,
            self.track_input,
            self.listener_callback_object,
            QoSPresetProfiles.SENSOR_DATA.value)

        camera_input_topics = self.get_parameter(
            'camera_input_topics').get_parameter_value().string_array_value
        camera_info_topics = self.get_parameter(
            'camera_info_topics').get_parameter_value().string_array_value
        camera_output_topics = self.get_parameter(
            'camera_output_topics').get_parameter_value().string_array_value

        self.image_subscriptions = []
        self.info_subscriptions = []
        for i in range(len(self.cameras)):
            image_subscription = self.create_subscription(
                CompressedImage,
                camera_input_topics[i],
                lambda msg, i=i: self.listener_callback_image(msg, i),
                QoSPresetProfiles.SENSOR_DATA.value)
            self.image_subscriptions.append(image_subscription)

            info_subscription = self.create_subscription(
                CameraInfo,
                camera_info_topics[i],
                lambda msg, i=i: self.listener_callback_info(msg, i),
                QoSPresetProfiles.SENSOR_DATA.value)
            self.info_subscriptions.append(info_subscription)

        # Create publishers for annotated images and the classified object list
        self.image_publishers = []
        for i in range(len(self.cameras)):
            publisher = self.create_publisher(CompressedImage, camera_output_topics[i], 10)
            self.image_publishers.append(publisher)

        self.fused_track_publisher_ = self.create_publisher(ObjectList, self.track_output, 10)

    def get_double_parameter(self, name):
        """Return the current value of the double parameter ``name``."""
        return self.get_parameter(name).get_parameter_value().double_value

    def get_matrix_parameter(self, name):
        """Parse the 4x4 matrix given as string in parameter ``name``."""
        text = self.get_parameter(name).get_parameter_value().string_value
        try:
            return parse_matrix(text, (4, 4))
        except ValueError as error:
            raise ValueError(f'Invalid parameter {name}: {error}') from error

    def cls_yolo_to_tracker(self, yolo_cls, yolo_conf):
        """Map a YOLO class index and confidence to a UFIL classification vector."""
        track_cls = np.zeros((7,), dtype='f')

        if yolo_cls == 0:  # Pedestrian
            track_cls[4] = yolo_conf
            self.fill_conf_intervall(4, track_cls, yolo_conf)

        elif yolo_cls == 1:  # Bicycle
            track_cls[3] = yolo_conf
            self.fill_conf_intervall(3, track_cls, yolo_conf)

        elif yolo_cls == 2:  # Car
            track_cls[0] = yolo_conf
            self.fill_conf_intervall(0, track_cls, yolo_conf)

        elif yolo_cls == 3:  # Motorcycle
            track_cls[2] = yolo_conf
            self.fill_conf_intervall(2, track_cls, yolo_conf)

        elif yolo_cls == 5 or yolo_cls == 7:  # Bus or Truck
            track_cls[1] = yolo_conf
            self.fill_conf_intervall(1, track_cls, yolo_conf)

        else:  # Other
            track_cls[6] = yolo_conf
            self.fill_conf_intervall(6, track_cls, yolo_conf)

        return track_cls

    def fill_conf_intervall(self, index, track_cls, conf):
        """
        Distribute the remaining confidence equally over all classes except ``index``.

        This ensures that the classification vector sums up to 1.
        """
        for i in range(0, 7):
            if i == index:
                continue
            else:
                track_cls[i] = (1 - conf) / 6
        return track_cls

    def track_object_to_string(self, track):
        """Return the name of the most likely class of a UFIL classification vector."""
        if np.argmax(track) == 0:
            return 'CAR'
        elif np.argmax(track) == 1:
            return 'TRUCK'
        elif np.argmax(track) == 2:
            return 'MOTORCYCLE'
        elif np.argmax(track) == 3:
            return 'BICYCLE'
        elif np.argmax(track) == 4:
            return 'PEDESTRIAN'
        else:
            return 'OTHER'

    def assign_yolo_to_point(self, track_object, image, boxes_yolo, u, v, threshold,
                             color_rotation, counter):
        """
        Assign the closest YOLO box to the projected track position (u, v).

        Sets the classification of ``track_object`` to the one of the assigned box and
        draws the box into the image.

        :return: Tuple (in_yolo, image), in_yolo is True if a box was assigned.
        """
        fit = []
        yolo_conf = []
        yolo_cls = []
        box_e_dist = []
        for box_yolo in boxes_yolo:
            box = box_yolo.xyxy[0]

            box_center = (box[0] + ((box[2] - box[0]) / 2), box[1] + ((box[3] - box[1]) / 2))

            if box_yolo.cls[0] == 1:
                box_center = (box_center[0], box[3])

            e_dist = np.sqrt((u - box_center[0])**2 + (v - box_center[1])**2)

            if e_dist < threshold:
                fit.append(box)
                yolo_conf.append(box_yolo.conf)
                yolo_cls.append(box_yolo.cls)
                box_e_dist.append(e_dist)

        if len(yolo_cls) == 0:
            image = cv2.circle(image, (u, v), 3, (0, 0, 255), -1)
            return False, image

        if 1 in yolo_cls and 0 in yolo_cls:
            box_e_dist[yolo_cls.index(0)] = 64

        if 3 in yolo_cls and 0 in yolo_cls:
            box_e_dist[yolo_cls.index(0)] = 64

        bfi = box_e_dist.index(min(box_e_dist))
        fit = fit[bfi]
        color = color_rotation[counter]
        classification = self.cls_yolo_to_tracker(yolo_cls[bfi], yolo_conf[bfi])
        image = cv2.rectangle(
            image, (int(fit[0]), int(fit[1])), (int(fit[2]), int(fit[3])), color, 2)
        cv2.putText(
            image, str(self.track_object_to_string(classification)),
            (int(fit[0]), int(fit[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)

        track_object.classification.classification = self.cls_yolo_to_tracker(
            yolo_cls[bfi], yolo_conf[bfi])
        return True, image

    def listener_callback_info(self, msg, index):
        """Store the calibration of camera ``index`` and destroy the info subscription."""
        self.cameras[index]['camera_info'] = msg
        self.cameras[index]['distortion'] = np.array(msg.d)
        self.cameras[index]['intrinsic'] = np.array(msg.k)
        self.cameras[index]['projection'] = np.array(msg.p)
        self.get_logger().info(f'Received camera info of {self.cameras[index]["frame"]}')
        self.destroy_subscription(self.info_subscriptions[index])

    def listener_callback_image(self, msg, index):
        """Store the last received image of camera ``index``."""
        self.cameras[index]['image'] = msg

    def listener_callback_object(self, msg):
        """Classify the received tracks using the last images and publish the result."""
        self.objectlist = msg

        try:
            # Convert the received compressed images to OpenCV images
            images = [self.br.compressed_imgmsg_to_cv2(camera['image'], 'passthrough')
                      for camera in self.cameras]
        except Exception:
            self.get_logger().info(
                'Waiting for images of all cameras', throttle_duration_sec=5.0)
            return

        # Get the transform from the object list frame to each camera frame
        for camera in self.cameras:
            camera['transform'] = self.tf_buffer.lookup_transform(
                camera['frame'],
                self.objectlist.header.frame_id,
                rclpy.time.Time())

        # Perform YOLO object detection on each image
        yolo_objects = [self.yolo(image, classes=self.yolo_classes, verbose=True)
                        for image in images]
        boxes_yolo = [yolo_object[0].boxes.cpu().numpy() if len(yolo_object) > 0 else []
                      for yolo_object in yolo_objects]

        # Initialize the fused track message
        fused_track_msg = ObjectList()
        fused_track_msg.header = self.objectlist.header

        counter = 0  # Reset the counter to cycle through color_rotation
        color_rotation = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
        try:
            for track_object in self.objectlist.objects:
                # Use a uniform classification if the track has none
                classification_sum = sum(track_object.classification.classification)
                if classification_sum < 0.99999 or classification_sum > 1.00001:
                    if classification_sum == 0.0:
                        track_object.classification.classification = np.full(7, 1 / 7)

                # Project the track object to each camera's image plane
                projections = [
                    to_cam(track_object, camera['transform'], camera['intrinsic'],
                           camera['distortion'], camera['projection'])
                    for camera in self.cameras]

                # Check if the projections are within the image boundaries
                img_size = (self.cameras[0]['camera_info'].width,
                            self.cameras[0]['camera_info'].height)
                in_frames = [(u >= 10 and v >= 0 and u <= img_size[0] - 10 and v <= img_size[1])
                             for u, v, _, _ in projections]

                in_yolos = [False] * len(self.cameras)

                fused_object = copy.deepcopy(track_object)

                if not any(in_frames):
                    # If the object is not in any frame, keep the stored classification
                    if self.conf_storage.check_for_id(fused_object.id):
                        fused_object.classification.classification = \
                            self.conf_storage.keep_fusion(fused_object.id)
                        fused_track_msg.objects = np.append(fused_track_msg.objects,
                                                            fused_object)
                else:
                    confidences = []
                    # Save confidences for all detections of the object
                    for i, (in_frame, (u, v, _, _), boxes) in enumerate(
                            zip(in_frames, projections, boxes_yolo)):
                        if in_frame:
                            track_copy = copy.deepcopy(track_object)
                            in_yolos[i], images[i] = self.assign_yolo_to_point(
                                track_copy, images[i], boxes, u, v, ASSIGNMENT_THRESHOLD,
                                color_rotation, counter)
                            images[i] = cv2.circle(
                                images[i], (u, v), 2, color_rotation[counter], -1)
                            if in_yolos[i]:
                                confidences.append(track_copy.classification.classification)
                            else:
                                confidences.append(None)

                    if not any(in_yolos):
                        # If the object is not detected by YOLO in any frame,
                        # keep the stored classification
                        if self.conf_storage.check_for_id(fused_object.id):
                            fused_object.classification.classification = \
                                self.conf_storage.keep_fusion(fused_object.id)
                            fused_track_msg.objects = np.append(fused_track_msg.objects,
                                                                fused_object)
                    else:
                        # Fuse the confidences from all frames
                        self.fuse_object(fused_object, track_object, confidences,
                                         fused_track_msg)

                    counter += 1
                    if counter >= len(color_rotation):
                        counter = 0

        except Exception as error:
            self.get_logger().error(
                f'Failed to classify tracks: {error}\n{traceback.format_exc()}')

        # Remove expired entries from the confidence storage
        self.conf_storage.kill_expred()

        # Convert the annotated images back to compressed image messages and publish them
        for publisher, image in zip(self.image_publishers, images):
            publisher.publish(self.br.cv2_to_compressed_imgmsg(image, 'jpeg'))

        # Publish the classified object list
        self.fused_track_publisher_.publish(fused_track_msg)

    def fuse_object(self, fused_object, track_object, confidences, fused_track_msg):
        """Fuse the camera classifications of a track and append it to the message."""
        valid_confidences = [conf for conf in confidences if conf is not None]
        if valid_confidences:
            confidence = conf_fusion(
                valid_confidences, self.get_double_parameter('tracker_trust'),
                self.get_double_parameter('camera_trust'),
                track_object.classification.classification)
            if sum(confidence) < 0.99 or sum(confidence) > 1.01:
                self.get_logger().warning(
                    f'Sum of fused confidence is {sum(confidence)} != 1')
            if (confidence is not None and isinstance(confidence, np.ndarray)
                    and confidence.shape == (7,)):
                fused_object.classification.classification = self.conf_storage.call_fusion(
                    fused_object.id, confidence, self.get_double_parameter('smoothing_factor'))
            else:
                self.get_logger().error(f'Invalid confidence value: {confidence}')
            fused_track_msg.objects = np.append(fused_track_msg.objects, fused_object)

        classification_sum = sum(fused_object.classification.classification)
        if classification_sum < 0.99 or classification_sum > 1.01:
            self.get_logger().warning(
                f'Sum of smoothed confidence is {classification_sum} != 1')


def main(args=None):
    """Initialize and spin the YoloClassifier node."""
    rclpy.init(args=args)

    yolo_classifier = YoloClassifier()

    try:
        rclpy.spin(yolo_classifier)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        yolo_classifier.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
