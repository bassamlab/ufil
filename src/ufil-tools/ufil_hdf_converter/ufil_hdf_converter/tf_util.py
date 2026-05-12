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

import h5py, numpy as np
import math

def _ensure_tf_group(root: h5py.Group, parent: str, child: str) -> h5py.Group:
    name = f"{parent}__to__{child}"
    g = root.require_group(name)
    if "times" not in g:
        g.create_dataset("times",  shape=(0,),   maxshape=(None,),  dtype=np.float64, chunks=True)
        g.create_dataset("xytheta",shape=(0,3),  maxshape=(None,3), dtype=np.float64, chunks=True)
        g.create_dataset("quat",   shape=(0,4),  maxshape=(None,4), dtype=np.float64, chunks=True)
        g.create_dataset("trans",  shape=(0,3),  maxshape=(None,3), dtype=np.float64, chunks=True)
        g.attrs["parent_frame"] = parent
        g.attrs["child_frame"]  = child
    return g

def _yaw_from_quat(qx, qy, qz, qw) -> float:
    s = 2.0*(qw*qz + qx*qy)
    c = 1.0 - 2.0*(qy*qy + qz*qz)
    return math.atan2(s, c)

def append_tf_transform(root_tf: h5py.Group, parent: str, child: str, t_sec: float,
                        tx: float, ty: float, tz: float, qx: float, qy: float, qz: float, qw: float) -> None:
    g = _ensure_tf_group(root_tf, parent, child)
    yaw = _yaw_from_quat(qx, qy, qz, qw)
    T = g["times"].shape[0]
    g["times"].resize((T+1,));            g["times"][T]   = float(t_sec)
    g["xytheta"].resize((T+1,3));         g["xytheta"][T] = (float(tx), float(ty), float(yaw))
    g["quat"].resize((T+1,4));            g["quat"][T]    = (float(qx), float(qy), float(qz), float(qw))
    g["trans"].resize((T+1,3));           g["trans"][T]   = (float(tx), float(ty), float(tz))
