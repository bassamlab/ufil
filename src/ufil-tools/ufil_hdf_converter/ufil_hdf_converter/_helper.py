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


import h5py
import numpy as np


def ensure_group_integrity(root_group: h5py.Group, datasets: dict, group_type: str, group_description: str):
    """Ensures the hdf5 group integrity by creating required datasets and attributes if missing."""
    if "type" not in root_group.attrs:
        root_group.attrs.create("type", group_type)
        root_group.attrs.create("description", group_description)

        for name, (shape, dtype, attrs) in datasets.items():
            create_dataset(root_group, name, shape, dtype, attrs)


def create_dataset(root_group: h5py.Group, name: str, shape: tuple, dtype, attrs: dict):
    """Generic function to create a dataset with attributes if it does not exist."""
    if name not in root_group:
        dataset = root_group.create_dataset(name, (0, *shape), dtype, chunks=True, maxshape=(None, *shape))
        for attr_name, attr_value in attrs.items():
            dataset.attrs.create(attr_name, attr_value)


def append_empty_rows_to_datasets(root_group: h5py.Group, dataset_names: list):
    """Appends an empty row to all datasets in dataset_names."""
    num_entries = root_group[dataset_names[0]].shape[0]  # Get current dataset length
    for dataset in dataset_names:
        root_group[dataset].resize(num_entries + 1, axis=0)


def fill_new_data_into_last_row(root_group: h5py.Group, dataset_mapping: dict):
    """Fills new data into the last row of datasets."""
    for dataset_name, values in dataset_mapping.items():
        root_group[dataset_name][-1] = values


