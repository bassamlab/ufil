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
import pandas as pd
import h5py


def read_objects_hdf(hdf_path):
    """
    Loads and processes an HDF5 file produced by Ufil.
    Reads reference data, and tracks from each objects list separately.
    Returns a dict where the first-level keys correspond to object list names (e.g., "pole").
    """
    reference_data = {}
    tracked_data = {}

    with h5py.File(hdf_path, "r") as f:
        ref_group = f["objects/obu/object_list_unfiltered"]
        reference_data = load_group_in_dict(ref_group)

        objects_group = f["objects/"]
        for obj_name, obj_group in objects_group.items():
            if obj_name == "obu":
                continue
            if "object_list" in obj_group:
                tracks_group = obj_group["object_list"]
                # print(tracks_group)
                tracked_data[obj_name] = load_group_in_dict(tracks_group)
        objects_group = f["objects/rsu/"]
        for obj_name, obj_group in objects_group.items():
            if obj_name == "obu":
                continue
            if "object_list" in obj_group:
                tracks_group = obj_group["object_list"]
                tracked_data[obj_name] = load_group_in_dict(tracks_group)
    return reference_data, tracked_data


def write_objects_hdf5(dic, filename):
    """
    Saves a nested dictionary to an HDF5 file.
    """

    def _recursively_save_dict(h5file, path, dic):
        for key, item in dic.items():
            if key == "_attrs":
                continue
            if isinstance(item, dict):
                h5file.create_group(f"{path}/{key}")
                _recursively_save_dict(h5file, f"{path}/{key}", item)
            elif isinstance(item, np.ndarray) or np.isscalar(item):
                h5file.create_dataset(f"{path}/{key}", data=item)
            else:
                h5file.create_dataset(f"{path}/{key}", data=str(item))

    with h5py.File(filename, "w") as h5file:
        _recursively_save_dict(h5file, "/", dic)


def load_group_in_dict(h5group):
    """
    Copy an HDF5 group into a Python dictionary.
    """
    result = {}

    for key, item in h5group.items():
        if isinstance(item, h5py.Dataset):
            # if not any(c.isdigit() for c in h5group.name):
            #     continue

            data = item[()]

            if key in ("message_time", "header_time"):
                # Always 1D
                result[key] = np.asarray(data).reshape(-1)
                # Make sure header time are in order if a single element is not in order replace it by linear intepolation 
                if key == "header_time":
                    header_time = result[key]
                    indices = np.diff(header_time) < -0.1e9
                    if np.any(indices):
                        # Print indecies to terminal in which the header time is not in order
                        print("Header time not in order at indices:", np.where(indices)[0], " with values:", np.diff(header_time)[indices])
                        print("key:", h5group.name + "/" + key)
                        for idx in np.where(indices)[0]:
                            if 0 < idx < len(header_time) - 1:
                                header_time[idx] = (header_time[idx-1] + header_time[idx + 1]) / 2
                        result[key] = header_time
            else:
                # Always 2D
                result[key] = np.atleast_2d(data)

        elif isinstance(item, h5py.Group):
            result[key] = load_group_in_dict(item)

    return result


def read_ref_classes(ref_classes_path):
    """
    Loads and processes a CSV file containing the reference classes for each object.
    Returns a dict where the first-level keys correspond to object list names (e.g., "pole").
    """
    df = pd.read_csv(
        ref_classes_path,
        sep=r"\s+",
        dtype=str,
        skiprows=None,
        nrows=None,
        engine="python",
        comment=None,
        index_col=0,
        header=None,
        skip_blank_lines=False,
    )

    classes_by_type = {}

    for full_path, class_name in df.iloc[:, 0].items():
        parts = full_path.strip("/").split("/")
        if len(parts) >= 4 and parts[0] == "objects" and parts[2] == "tracks":
            object_type = parts[1]
            object_id = parts[3]

            if object_type not in classes_by_type:
                classes_by_type[object_type] = {}

            classes_by_type[object_type][object_id] = class_name

    return dict(classes_by_type)
