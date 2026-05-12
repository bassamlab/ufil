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


import ufil_msgs.msg
import h5py
import numpy as np

from ._helper import * 

def flatten_object(msg: ufil_msgs.msg.Object, timestamp: np.float64, header_timestamp: np.float64, root_group: h5py.Group):
    """Conversion function for Object messages."""
    obj_datasets = {
        "message_time":
        (
            (1,), 
            np.float64,
            {
                "name": "message creation time",
                "description": "time stamp in ns at wiche this message was created",
                "dim": "[message time]",
                "axis": "x",
                "unit": "ns",
                "quantity": "-"
            }
        ),
        "header_time":
        (
            (1,), 
            np.float64,
            {
                "name": "message header time",
                "description": "time stamp in ns at wiche this message was created",
                "dim": "[message time]",
                "axis": "x",
                "unit": "ns",
                "quantity": "-"
            }
        ),
        "existence_probability":
        (
            (1,), 
            np.float64,
            {
                "name": "object existence probability",
                "description": "existence probability given at point in time",
                "dim": "[existence probability]",
                "axis": "y",
                "unit": "percent",
                "quantity": "probability"
            }
        ),
        "state/position":
        (
            (3,),
            float,
            {
                "name": "position",
                "description": "position given at point in time",
                "dim": "[x position, y position, x position]",
                "axis": "y",
                "unit": "m",
                "quantity": "position"
            }
        ),
        "state/position_variance":
        (
            (3,),
            float,
            {
                "name": "position_variance",
                "description": "position variance given at point in time",
                "dim": "[x position variance, y position variance, z position variance]",
                "axis": "y",
                "unit": "m",
                "quantity": "variance"}
        ),
        "state/linear_velocity": (
            (3,),
            float,
            {
                "name": "velocity",
                "description": "linear velocity given at point in time",
                "dim": "[x velocity, y velocity, z velocity]",
                "axis": "y",
                "unit": "m/s",
                "quantity": "velocity"
            }
        ),
        "state/linear_velocity_variance": (
            (3,),
            float,
            {
                "name": "velocity_variance",
                "description": "linear velocity variance given at point in time",
                "dim": "[x velocity variance, y velocity variance, z velocity variance]",
                "axis": "y",
                "unit": "m/s",
                "quantity": "variance"
            }
        ),
        "state/linear_acceleration": (
            (3,),
            float,
            {
                "name": "acceleration",
                "description": "linear acceleration given at point in time",
                "dim": "[x acceleration, y acceleration, z acceleration]",
                "axis": "y",
                "unit": "m/s^2",
                "quantity": "acceleration"
            }
        ),
        "state/linear_acceleration_variance": (
            (3,),
            float,
            {
                "name": "acceleration_variance",
                "description": "linear acceleration variance given at point in time",
                "dim": "[x acceleration variance, y acceleration variance, z acceleration variance]",
                "axis": "y",
                "unit": "m/s^2",
                "quantity": "variance"
            }
        ),
        "state/orientation": (
            (3,),
            float,
            {
                "name": "orientation",
                "description": "orientation of the sensor in euler angles",
                "dim": "[roll, pitch, yaw]",
                "axis": "y",
                "unit": "rad",
                "quantity": "orientation"
            }
        ),
        "state/orientation_variance": (
            (3,),
            float,
            {
                "name": "orientation_variance",
                "description": "orientation variance given at point in time",
                "dim": "[roll variance, pitch variance, yaw variance]",
                "axis": "y",
                "unit": "rad",
                "quantity": "variance"
            }
        ),
        "state/angular_velocity": (
            (3,),
            float,
            {
                "name": "angular velocity", 
                "description": "angular velocity",
                "dim": "[x angular velocity, y angular velocity, z angular velocity]",
                "axis": "y",
                "unit": "rad/s", 
                "quantity": "angular velocity"
            }
        ),
        "state/angular_velocity_variance": (
            (3,),
            float,
            {
                "name": "angular_velocity_variance",
                "description": "orientation variance given at point in time",
                "dim": "[x angular velocity variance, y angular velocity variance, z angular velocity variance]",
                "axis": "y",
                "unit": "rad",
                "quantity": "variance"
            }
        ),
        "dimension/dimension":
        (
            (3,),
            float,
            {
                "name": "dimension",
                "description": "dimension given at point in time",
                "dim": "[x dimension, y dimension, x v]",
                "axis": "y",
                "unit": "m",
                "quantity": "dimension"
            }
        ),
        "dimension/dimension_variance":
        (
            (3,),
            float,
            {
                "name": "dimension_variance",
                "description": "dimension variance given at point in time",
                "dim": "[x dimension variance, y dimension variance, z dimension variance]",
                "axis": "y",
                "unit": "m",
                "quantity": "variance"}
        ),
        "classification":
        (
            (7,),
            float,
            {
                "name": "classification",
                "description": "classification vector variance given at point in time",
                "dim": "[car, truck, motorcycle, bicycle, pedestrian, stationary, other]",
                "axis": "y",
                "unit": "percent",
                "quantity": "confidence"}
        ),
    }
    
    ensure_group_integrity(root_group, obj_datasets, "simulation", "data from an object state estimation algorithm")
   
    dataset_mapping = {
        "message_time": (timestamp),
        "header_time": (header_timestamp),
        "existence_probability": (msg.existence_probability),
        "state/position": (msg.state.state.x, msg.state.state.y, 0),
        "state/position_variance": (msg.state.covariance[0], msg.state.covariance[9], -1),
        "state/linear_velocity": (msg.state.state.v_x, msg.state.state.v_y, 0),
        "state/linear_velocity_variance": (msg.state.covariance[18], msg.state.covariance[27], -1),
        "state/linear_acceleration": (msg.state.state.a_x, msg.state.state.a_y, 0),
        "state/linear_acceleration_variance": (msg.state.covariance[36], msg.state.covariance[45], -1),
        "state/orientation": (0, 0, msg.state.state.yaw),
        "state/orientation_variance": (-1, -1, msg.state.covariance[54]),
        "state/angular_velocity": (0, 0, msg.state.state.yaw_rate),
        "state/angular_velocity_variance": (-1, -1, msg.state.covariance[63]),
        "dimension/dimension": (msg.dimension.dimension.length, msg.dimension.dimension.width, msg.dimension.dimension.height),
        "dimension/dimension_variance": (msg.dimension.covariance[0], msg.dimension.covariance[4], msg.dimension.covariance[8]),
        "classification": (msg.classification.classification)
    }

    append_empty_rows_to_datasets(root_group, list(dataset_mapping.keys()))
    fill_new_data_into_last_row(root_group, dataset_mapping)

