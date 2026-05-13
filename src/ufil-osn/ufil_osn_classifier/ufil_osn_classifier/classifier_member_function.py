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

from rclpy.qos import QoSPresetProfiles

import traceback
from classifier_projection import to_cam
from conf_fusion import conf_fusion, ConfidenceStorage

from ultralytics import YOLO
import copy
import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

from cv_bridge import CvBridge
import cv2

from ufil_msgs.msg import ObjectList
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Transform

class YoloClassifier(Node):
    """
    A ROS 2 node that performs object detection using the YOLO algorithm.
    Subscribes to image and camera info topics, processes the images to detect objects,
    and publishes annotated images and fused object tracks.
    """

    def __init__(self):
        super().__init__('yolo_classifier')

        # Declare parameters for input and output topics and camera info
        self.track_input = self.declare_parameter('track_input', 'pole/tracks').get_parameter_value().string_value
        self.track_output = self.declare_parameter('track_output', 'pole/fused_tracks').get_parameter_value().string_value

        self.declare_parameter('camera_input_topics', ['/camera_right/image_rect/compressed', '/camera_left/image_rect/compressed'])
        self.declare_parameter('camera_output_topics', ['/annot_right/image_rect/compressed', '/annot_left/image_rect/compressed'])
        self.declare_parameter('camera_info_topics', ['/camera_right/camera_info', '/camera_left/camera_info'])
        
        self.target_frame = self.declare_parameter('target_frame', 'm1').get_parameter_value().string_value
        
        self.declare_parameter('tracker_trust', 0.2)
        self.declare_parameter('camera_trust', 0.4)
        self.declare_parameter('smoothing_factor', 0.1)
        self.declare_parameter('task', "classify")
        
        
        self.declare_parameter('T_camera_right_lidar', """[
            [-0.26207, 0.96484, 0.01995, -0.29462],
            [0.07057, -0.00146, 0.99751, 0.20912],
            [0.96247, 0.26282, -0.06771, 0.97677],
            [0.00000, 0.00000, 0.00000, 1.00000]
        ]""")
        self.declare_parameter('T_camera_left_lidar', """[
            [0.26207, 0.96484, -0.01995, -0.29462],
            [0.07056, 0.00146, 0.99751, 0.20912],
            [0.96247, -0.26283, -0.06770, 0.97677],
            [0.00000, 0.00000, 0.00000, 1.00000]
        ]""")

        # Initialize camera parameters
        self.cameras = [
            {
                'object_list': ObjectList(),
                'frame': "camera_right_image",
                'transform': Transform,
                'image': CompressedImage,
                'distortion': [],
                'intrinsic': [],
                'projection': [],
                'camera_info': CameraInfo,
                'T_camera_lidar': np.matrix(eval(self.get_parameter('T_camera_right_lidar').get_parameter_value().string_value)),
                
                #'T_camera_lidar': np.matrix([
                #    [-0.26207, 0.96484, 0.01995, -0.29462],
                #    [0.07057, -0.00146, 0.99751, 0.20912],
                #    [0.96247, 0.26282, -0.06771, 0.97677],
                #    [0.00000, 0.00000, 0.00000, 1.00000]
                #])
            },
            {
                'object_list': ObjectList(),
                'frame': "camera_left_image",
                'transform': Transform,
                'image': CompressedImage,
                'distortion': [],
                'intrinsic': [],
                'projection': [],
                'camera_info': CameraInfo,
                'T_camera_lidar': np.matrix(eval(self.get_parameter('T_camera_left_lidar').get_parameter_value().string_value)),
                #'T_camera_lidar': np.matrix([
                #    [0.26207, 0.96484, -0.01995, -0.29462],
                #    [0.07056, 0.00146, 0.99751, 0.20912],
                #    [0.96247, -0.26283, -0.06770, 0.97677],
                #    [0.00000, 0.00000, 0.00000, 1.00000]
                #])
            }
        ]

        # YOLO classes to detect
        self.yolo_classes = [0, 1, 2, 3, 5, 7, 15, 16, 17, 36]

        # Initialize confidence storage
        self.conf_storage = ConfidenceStorage()

        # Initialize CvBridge and YOLO model
        self.br = CvBridge()
        # self.yolo = YOLO("src/ufil-osn/ufil_osn_classifier/ufil_osn_classifier/yolo11s.pt")
        self.yolo = YOLO(
            "src/ufil-osn/ufil_osn_classifier/ufil_osn_classifier/yolo11s.engine",
            self.get_parameter('task').get_parameter_value().string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscriptions for object list, images, and camera info
        self.object_subscription = self.create_subscription(
            ObjectList,
            self.track_input,
            self.listener_callback_object,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.object_subscription

        self.image_subscriptions = []
        self.info_subscriptions = []
        for i, camera in enumerate(self.cameras):
            image_subscription = self.create_subscription(
                CompressedImage,
                self.get_parameter('camera_input_topics').get_parameter_value().string_array_value[i],
                lambda msg, i=i: self.listener_callback_image(msg, i),
                QoSPresetProfiles.SENSOR_DATA.value)
            self.image_subscriptions.append(image_subscription)

            info_subscription = self.create_subscription(
                CameraInfo,
                self.get_parameter('camera_info_topics').get_parameter_value().string_array_value[i],
                lambda msg, i=i: self.listener_callback_info(msg, i),
                QoSPresetProfiles.SENSOR_DATA.value)
            self.info_subscriptions.append(info_subscription)

        # Create publishers for annotated images and fused tracks
        self.publish = []
        for i in range(len(self.cameras)):
            publisher = self.create_publisher(CompressedImage, self.get_parameter('camera_output_topics').get_parameter_value().string_array_value[i], 10)
            self.publish.append(publisher)
        
        self.fused_track_publisher_ = self.create_publisher(ObjectList, self.track_output, 10)
        

    def cls_yolo_to_tracker(self, yolo_cls, yolo_conf):
        """
        Maps YOLO class indexes to Ufil confidence intervals.
        """
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
        Fills confidence interval for all other classes except the given index with equal confidence
        to ensure the sum of the confidence array equals 1.
        """
        for i in range(0, 7):
            if i == index:
                continue
            else:
                track_cls[i] = (1 - conf) / 6
        return track_cls

    def track_object_to_string(self, track):
        """
        Maps Ufil tracks to string for display.
        """
        if np.argmax(track) == 0:
            return "CAR"
        elif np.argmax(track) == 1:
            return "TRUCK"
        elif np.argmax(track) == 2:
            return "MOTORCYCLE"
        elif np.argmax(track) == 3:
            return "BICYCLE"
        elif np.argmax(track) == 4:
            return "PEDESTRIAN"
        else:
            return "OTHER"

    def assign_yolo_to_point(self, track_object, image, boxes_yolo, u, v, threshold, color_rotation, counter):
        """
        Assigns each point a YOLO box and classification.
        """
        fit = []
        yolo_conf = []
        yolo_cls = []
        box_e_dist = []
        in_yolo = True
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
            in_yolo = False
            return in_yolo, image
        else:
            if 1 in yolo_cls and 0 in yolo_cls:
                box_e_dist[yolo_cls.index(0)] = 64

            if 3 in yolo_cls and 0 in yolo_cls:
                box_e_dist[yolo_cls.index(0)] = 64

            bfi = box_e_dist.index(min(box_e_dist))
            fit = fit[bfi]
            image = cv2.rectangle(image, (int(fit[0]), int(fit[1])), (int(fit[2]), int(fit[3])), color_rotation[counter], 2)
            cv2.putText(image, str(self.track_object_to_string(self.cls_yolo_to_tracker(yolo_cls[bfi], yolo_conf[bfi]))), (int(fit[0]), int(fit[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, color_rotation[counter], 1)
            #cv2.putText(image,  str(yolo_conf[bfi]), (int(fit[0]- 80), int(fit[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, color_rotation[counter], 1)

            track_object.classification.classification = self.cls_yolo_to_tracker(yolo_cls[bfi], yolo_conf[bfi])
            return in_yolo, image

    def listener_callback_info(self, msg, index):
        """
        Callback function for camera info subscription.
        Gets camera info once and destroys the subscription after.
        """
        self.cameras[index]['camera_info'] = msg
        self.cameras[index]['distortion'] = np.array(msg.d)
        self.cameras[index]['intrinsic'] = np.array(msg.k)
        self.cameras[index]['projection'] = np.array(msg.p)
        #print(f"camera Initialised with:{self.cameras[index]['camera_info']},{self.cameras[index]['frame']} ")
        self.destroy_subscription(self.info_subscriptions[index])

    def listener_callback_image(self, msg, index):
        """
        Callback function for image subscription.
        Saves the last received image.
        """
        self.cameras[index]['image'] = msg

    def listener_callback_object(self, msg):
        """
        Callback function for object list subscription.
        Processes the received object list and performs object detection.
        """
        self.objectlist = msg

        try:
            # Convert the received compressed images to OpenCV images
            images = [self.br.compressed_imgmsg_to_cv2(camera['image'], "passthrough") for camera in self.cameras]

        except Exception as e:
            print("no image yet")
            return

        print(1)
        # Get the transform from the object list frame to each camera frame
        for camera in self.cameras:
            camera['transform'] = self.tf_buffer.lookup_transform(
                camera['frame'],
                self.objectlist.header.frame_id,
                rclpy.time.Time())

        # Perform YOLO object detection on each image
        yolo_objects = [self.yolo(image, classes=self.yolo_classes, verbose=True) for image in images]
        boxes_yolo = [yolo_object[0].boxes.cpu().numpy() if len(yolo_object) > 0 else [] for yolo_object in yolo_objects]

        # Initialize the fused track message
        fused_track_msg = ObjectList()
        fused_track_msg.header = self.objectlist.header

        counter = 0  # Reset the counter to cycle through color_rotation
        color_rotation = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
        try:
            for track_object in self.objectlist.objects:
                # Ensure the classification sum is valid
                if sum(track_object.classification.classification) < 0.99999 or sum(track_object.classification.classification) > 1.00001:
                    #print(f"ERROR track_object = {sum(track_object.classification.classification)} !=1 for Track with id: {track_object.id}")
                    if sum(track_object.classification.classification) == 0.0:
                        track_object.classification.classification = np.full(7, 1 / 7)

                # Project the track object to each camera's image plane
                projections = [to_cam(track_object, camera['transform'], camera['intrinsic'], camera['distortion'], camera['projection'])
                               for camera in self.cameras]

                # Check if the projections are within the image boundaries
                img_size = (self.cameras[0]['camera_info'].width, self.cameras[0]['camera_info'].height)
                #in_frames = [(u >= 0 and v >= 0 and u <= img_size[0] and v <= img_size[1]) for u, v, _, _ in projections]
                in_frames = [(u >= 10 and v >= 0 and u <= img_size[0]-10 and v <= img_size[1]) for u, v, _, _ in projections]

                in_yolos = [False] * len(self.cameras)
                in_yolo = True

                fused_object = copy.deepcopy(track_object)

                if not any(in_frames):
                    # If the object is not in any frame, check if it exists in the confidence storage
                    if not self.conf_storage.check_for_id(fused_object.id):
                        pass
                    else:
                    # If the object is in storage keep it for future 
                        fused_object.classification.classification = self.conf_storage.keep_fusion(fused_object.id)
                        fused_track_msg.objects = np.append(fused_track_msg.objects, fused_object)
                else:
                    confidences = []
                    # Safe conficences for all detections of object
                    for i, (in_frame, (u, v, _, _), camera, boxes) in enumerate(zip(in_frames, projections, self.cameras, boxes_yolo)):
                        if in_frame:
                            track_copy = copy.deepcopy(track_object)
                            in_yolos[i], images[i] = self.assign_yolo_to_point(track_copy, images[i], boxes, u, v, 80, color_rotation, counter)
                            images[i] = cv2.circle(images[i], (u, v), 2, color_rotation[counter], -1)
                            if in_yolos[i]:
                                confidences.append(track_copy.classification.classification)
                            else:
                                confidences.append(None)

                    if not any(in_yolos):
                        # If the object is not detected by YOLO in any frame, check if it exists in the confidence storage
                        if not self.conf_storage.check_for_id(fused_object.id):
                            pass
                        else:
                        # If the object is in storage keep it for future 
                            fused_object.classification.classification = self.conf_storage.keep_fusion(fused_object.id)
                            fused_track_msg.objects = np.append(fused_track_msg.objects, fused_object)

                    else:
                        # Fuse the confidences from all frames
                        valid_confidences = [conf for conf in confidences if conf is not None]
                        if valid_confidences:
                            confidence = conf_fusion(valid_confidences, self.get_parameter('tracker_trust').get_parameter_value().double_value, self.get_parameter('camera_trust').get_parameter_value().double_value, track_object.classification.classification)
                            if sum(confidence) < 0.99 or sum(confidence) > 1.01:
                                print(f"ERROR conf sum   conf_fusion= {sum(confidence)} !=1")
                            if confidence is not None and isinstance(confidence, np.ndarray) and confidence.shape == (7,):
                                fused_object.classification.classification = self.conf_storage.call_fusion(fused_object.id, confidence, self.get_parameter('smoothing_factor').get_parameter_value().double_value)
                            else:
                                print(f"Invalid confidence value: {confidence}")
                            fused_track_msg.objects = np.append(fused_track_msg.objects, fused_object)

                        if sum(fused_object.classification.classification) < 0.99 or sum(fused_object.classification.classification) > 1.01:
                            print(f"ERROR conf sum = {sum(fused_object.classification.classification)} !=1")
                          

                    counter += 1
                    if counter >= len(color_rotation):
                        counter = 0

        except Exception as error:
            print(error)
            traceback.print_exc()  # Print the full traceback of the exception for debugging purposes

        # Remove expired entries from the confidence storage
        self.conf_storage.kill_expred()

        # Convert the annotated images back to compressed image messages
        image_msgs = [self.br.cv2_to_compressed_imgmsg(image, "jpeg") for image in images]

        # Publish the annotated images
        [publisher.publish(image_msg) for publisher, image_msg in zip(self.publish, image_msgs)]

        # Publish the fused track message
        self.fused_track_publisher_.publish(fused_track_msg)
        print(self.get_parameter('smoothing_factor').get_parameter_value().double_value)

def main(args=None):
    """
    Main function to initialize and run the YoloClassifier node.
    """
    rclpy.init(args=args)

    yolo_cassifier = YoloClassifier()

    rclpy.spin(yolo_cassifier)

    yolo_cassifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()