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

def flatten_object_list(msg: ufil_msgs.msg.ObjectList, timestamp: np.float64, root_group: h5py.Group):
    """Conversion function for Object messages."""
    obj_datasets = {
        "message_time":
        (
            (1,), 
            np.float64,
            {
                "name": "message reception time",
                "description": "time stamp in ns at wiche this message was received",
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
  	"number_of_objects":
        (
            (1,), 
            int,
            {
                "name": "number of objects",
                "description": "number of objects in list",
                "dim": "[number of objects]",
                "axis": "y",
                "unit": "amount",
                "quantity": "-"
            }
        ),
    }
    
    ensure_group_integrity(root_group, obj_datasets, "simulation", "data from an object state estimation algorithm")
   
    dataset_mapping = {
        "message_time": (timestamp),        
        "header_time": (msg.header.stamp.sec*1e9 + msg.header.stamp.nanosec),
        "number_of_objects": (len(msg.objects)),

    }

    append_empty_rows_to_datasets(root_group, list(dataset_mapping.keys()))
    fill_new_data_into_last_row(root_group, dataset_mapping)

