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

def _ensure_root(root: h5py.Group):
    if "times" not in root:
        root.create_dataset("times", shape=(0,), maxshape=(None,),
                            dtype=np.float64, chunks=True)

    info = root.require_group("info")
    return info

def flatten_occ_grid(root: h5py.Group, msg, t_sec: float):
    info = _ensure_root(root)

    h, w = int(msg.info.height), int(msg.info.width)
    res  = float(msg.info.resolution)
    ox   = float(msg.info.origin.position.x)
    oy   = float(msg.info.origin.position.y)
    fid  = msg.header.frame_id

    info.attrs["resolution"] = res
    info.attrs["width"] = w
    info.attrs["height"] = h
    info.attrs["origin_x"] = ox
    info.attrs["origin_y"] = oy
    info.attrs["frame_id"] = fid

    raw = np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8)
    mask = (raw > 0).astype(np.uint8)

    if "value" not in root:
        root.create_dataset(
            "value", shape=(0, h, w), maxshape=(None, h, w),
            dtype=np.int8, chunks=(1, h, w), compression="gzip", shuffle=True
        )
    if "mask" not in root:
        root.create_dataset(
            "mask", shape=(0, h, w), maxshape=(None, h, w),
            dtype=np.uint8, chunks=(1, h, w), compression="gzip", shuffle=True
        )

    if root["value"].shape[1:] != (h, w):
        raise RuntimeError(
            f"Occ grid size changed from {root['value'].shape[1:]} to {(h,w)}; "
            "split runs or implement resizing logic."
        )

    T = root["value"].shape[0]
    root["value"].resize((T+1, h, w)); root["value"][T, :, :] = raw
    root["mask"].resize((T+1, h, w));  root["mask"][T, :, :]  = mask

    ts = root["times"]; ts.resize((T+1,)); ts[T] = float(t_sec)
