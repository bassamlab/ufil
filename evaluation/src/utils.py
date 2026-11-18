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


import numpy as np
from config import cfg
from collections import OrderedDict


def print_dict_structure(d, indent=0):
    """
    Recursively prints the keys of a nested dictionary, ignoring the actual data content.
    """
    for key, value in d.items():
        if key == "_attrs":
            print("  " * indent + f"{key} (attributes)")
        elif isinstance(value, dict):
            print("  " * indent + f"{key}/")
            print_dict_structure(value, indent + 1)
        else:
            print(
                "  " * indent
                + f"{key} (dataset, shape={getattr(value, 'shape', None)})"
            )


def print_summary_metrics(metrics):
    """
    Prints summary RMSE metrics (position, yaw, and dimension) for matched tracked vs. reference objects.
    """

    print(f"Matches: {metrics['matches_number']}")
    print(f"Pos RMSE: {metrics['position_rmse']:.3f} m")
    print(f"Orientation RMSE: {metrics['orientation_rmse']:.3f} rad")
    print(f"Velocity RMSE: {metrics['velocity_rmse']:.3f} m/s")
    print(f"Dimension RMSE: {metrics['dimension_rmse']:.3f} m")
    print(f"Classification Accuracy: {metrics['classification_accuracy']}")
    print(f"Classification Error Rate: {metrics['classification_error_rate']}")


def sort_objects(objects_dict, field):
    """
    Sort a dictionary of objects by a given field, and returns a dictionary.
    """
    sorted_items = sorted(
        objects_dict.items(), key=lambda item: item[1].get(field, float("inf"))
    )

    return OrderedDict(sorted_items)


def class_index_to_class_name(class_index):
    """
    Return the class name corresponding to the given class index, using the class map defined in the configuration file.
    """
    class_map = cfg["classes_map"]
    return class_map.get(class_index, "other")
