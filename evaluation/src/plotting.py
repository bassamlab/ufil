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
import matplotlib as plt
from utils import print_dict_structure


def plot_shaded(ax, t, data, var, color="g", label="3σ"):
    upper = data + 3 * np.sqrt(var)
    lower = data - 3 * np.sqrt(var)
    ax.fill_between(
        t, lower, upper, color=color, alpha=0.1, edgecolor="none", label=label
    )


def plot_tracking_error(match):
    fig, axes = plt.pyplot.subplots(2, 4, figsize=(16, 9))
    fig.suptitle(
        f"Object ID: {match['object_id']} <- Reference ID: {match['reference']['object_id']}",
        fontsize=16,
    )

    data = match["plotting"]["tracking"]

    # Pos X
    ax = axes[0, 0]
    ax.plot(
        data["time"], data["ref_position"][0, :], color="b", linewidth=1, label="Ref"
    )
    plot_shaded(
        ax, data["time"], data["position"][0, :], data["position_variance"][0, :]
    )
    ax.plot(data["time"], data["position"][0, :], color="g", linewidth=2, label="Track")
    ax.set_title("Pos X")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Pos X [m]")
    ax.legend()

    # Pos Y
    ax = axes[0, 1]
    ax.plot(
        data["time"], data["ref_position"][1, :], color="b", linewidth=1, label="Ref"
    )
    plot_shaded(
        ax, data["time"], data["position"][1, :], data["position_variance"][1, :]
    )
    ax.plot(data["time"], data["position"][1, :], color="g", linewidth=2, label="Track")
    ax.set_title("Pos Y")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Pos Y [m]")
    ax.legend()

    # Pos X Error
    ax = axes[1, 0]
    ax.plot(
        data["time"],
        np.abs(data["position"][0, :] - data["ref_position"][0, :]),
        color="r",
        linewidth=1,
        label="Error",
    )
    ax.set_title(f"RMSE X {match['metrics']['position_rmse'][0]:.3f}")
    ax.set_ylim([0, 5])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [m]")

    # Pos Y Error
    ax = axes[1, 1]
    ax.plot(
        data["time"],
        np.abs(data["position"][1, :] - data["ref_position"][1, :]),
        color="r",
        linewidth=1,
        label="Error",
    )
    ax.set_title(f"RMSE Y {match['metrics']['position_rmse'][1]:.3f}")
    ax.set_ylim([0, 5])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [m]")

    # Yaw
    ax = axes[0, 3]
    ax.plot(data["time"], data["ref_orientation"], color="b", linewidth=1, label="Ref")
    plot_shaded(ax, data["time"], data["orientation"], data["orientation_variance"])
    ax.plot(data["time"], data["orientation"], color="g", linewidth=2, label="Track")
    ax.set_title("Yaw")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Yaw [rad]")
    ax.legend()

    # Yaw Error
    ax = axes[1, 3]
    ax.plot(
        data["time"],
        np.abs(data["orientation"] - data["ref_orientation"]),
        color="r",
        linewidth=1,
        label="Error",
    )
    ax.set_title(f"RMSE Yaw {match['metrics']['orientation_rmse']:.3f}")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [rad]")

    # XY Plane Trajectory
    ax = axes[0, 2]
    ax.plot(
        data["ref_position"][0, :],
        data["ref_position"][1, :],
        color="b",
        linewidth=1,
        label="Ref",
    )
    ax.plot(
        data["position"][0, :],
        data["position"][1, :],
        color="g",
        linewidth=1,
        label="Track",
    )
    # ax.fill(
    #     [0, 15, 15, 0],
    #     [0, -30, 30, 0],
    #     color="g",
    #     alpha=0.1,
    #     edgecolor="none",
    #     label="3σ",
    # )
    ax.set_title("Pos X/Y")
    ax.set_xlabel("Pos X [m]")
    ax.set_ylabel("Pos Y [m]")
    # ax.set_xlim([25, 60])
    # ax.set_ylim([120, 140])
    ax.axis('equal')
    ax.legend()

    # Position X/Y Error
    ax = axes[1, 2]
    dim_error = np.sum(np.abs(data["position"] - data["ref_position"]), axis=0)
    ax.plot(data["time"], dim_error, color="r", linewidth=1, label="Error")
    ax.set_title(f"RMSE {match['metrics']['position_rmse'][2]:.3f}")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [m]")
    ax.set_ylim([0, 5])

    plt.pyplot.tight_layout()
    return fig


def plot_dimension_error(match):
    fig, axes = plt.pyplot.subplots(2, 3, figsize=(12.8, 7.2))
    fig.suptitle(
        f"Object ID: {match['object_id']} <- Reference ID: {match['reference']['object_id']}",
        fontsize=14,
    )

    data = match["plotting"]["dimension"]
    dimension_labels = ["Length", "Width", "Height"]

    for i in range(3):
        ax = axes[0, i]
        ax.plot(
            data["time"],
            data["reference_dimension"][i, :],
            linewidth=1.0,
            label="Ref",
            color="b",
        )

        upper = data["dimension"][i, :] + 3.0 * np.sqrt(
            data["dimension_variance"][i, :]
        )
        lower = data["dimension"][i, :] - 3.0 * np.sqrt(
            data["dimension_variance"][i, :]
        )
        ax.fill_between(data["time"], lower, upper, color="g", alpha=0.1, label="3σ")

        ax.plot(
            data["time"],
            data["dimension"][i, :],
            linewidth=2.0,
            label="Track",
            color="g",
        )
        ax.set_title(dimension_labels[i])
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(f"{dimension_labels[i]} [m]")
        ax.legend()

        ax_err = axes[1, i]
        error = np.abs(data["dimension"][i, :] - data["reference_dimension"][i, :])
        ax_err.plot(data["time"], error, linewidth=1.0, label="Error", color="r")
        ax_err.set_title(
            f"RMSE {dimension_labels[i]}: {data['dimension_rmse'][i]:.3f} m"
        )
        ax_err.set_ylim([0, 5])
        ax_err.set_xlabel("Time [s]")
        ax_err.set_ylabel("Error [m]")
        ax_err.legend()

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


def plot_timing_error(match):
    fig, ax = plt.pyplot.subplots(figsize=(8, 6))
    fig.suptitle(f"Object ID: {match['object_id']}", fontsize=14)

    data = match["plotting"]["timing"]["tracker"]
    ax.hist(data, bins=50, color="b", alpha=0.7)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Frequency")
    return fig
