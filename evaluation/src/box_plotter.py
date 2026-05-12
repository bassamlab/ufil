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
import matplotlib.pyplot as plt


# -----------------------------------------------------------
# Helper: Load arrays and compute squared errors from HDF
# -----------------------------------------------------------
def load_squared_errors(hdf_path):
    with h5py.File(hdf_path, "r") as f:
        # Find root object (randomized name)
        root = list(f.keys())[0]
        state = f[f"{root}/state"]
        ref = f[f"{root}/reference"]

        # Read arrays
        vel = np.array(state["linear_velocity"])      # shape 2×N
        pos = np.array(state["position"])             # shape 2×N
        ori = np.array(state["orientation"])          # shape 1×N

        ref_vel = np.array(ref["linear_velocity"])
        ref_pos = np.array(ref["position"])
        ref_ori = np.array(ref["orientation"])

    # Squared errors per time step
    se_vel = np.sum((vel - ref_vel)**2, axis=0)      # N values
    se_pos = np.sum((pos - ref_pos)**2, axis=0)      # N values
    se_ori = (ori - ref_ori)**2                      # N values (1×N)

    print(se_vel.shape)
    return se_vel, se_pos, se_ori.squeeze()


# -----------------------------------------------------------
# Load data from two files
# -----------------------------------------------------------
file1 = "/home/ufildev/ufil_ws/Bags/man3_eval/ekf/matched_objects.hdf5"
file2 = "/home/ufildev/ufil_ws/Bags/man3_eval/fatrop/matched_objects.hdf5"

vel1, pos1, ori1 = load_squared_errors(file1)
vel2, pos2, ori2 = load_squared_errors(file2)

def plot_measure(title, measure1, measure2, ylabel, filename=None):
    plt.figure(figsize=(6, 5))

    plt.boxplot([measure1, measure2],
                labels=["EKF", "Fatrop"],
                patch_artist=True,
                boxprops=dict(facecolor="lightgray"),
                medianprops=dict(color="black"),
                showfliers=False
                )

    plt.title(title)
    plt.ylabel(ylabel)
    plt.grid(axis="y", linestyle="--", alpha=0.4)

    if filename:
        plt.savefig(filename, dpi=200, bbox_inches="tight")

    plt.show()


# -----------------------------------------------------------
# Generate three separate plots
# -----------------------------------------------------------
plot_measure("Velocity Squared Error", vel1, vel2,
             ylabel="Squared Error", filename="vel_se_boxplot.png")

plot_measure("Position Squared Error", pos1, pos2,
             ylabel="Squared Error", filename="pos_se_boxplot.png")

plot_measure("Orientation Squared Error", ori1, ori2,
             ylabel="Squared Error", filename="ori_se_boxplot.png")