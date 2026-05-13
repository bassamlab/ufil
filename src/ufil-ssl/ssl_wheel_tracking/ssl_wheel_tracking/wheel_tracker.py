# Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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
from datetime import datetime, timedelta
from typing import List, Set

import numpy as np

from stonesoup.dataassociator.neighbour import GNNWith2DAssignment
from stonesoup.deleter.time import UpdateTimeDeleter
from stonesoup.hypothesiser.distance import DistanceHypothesiser
from stonesoup.initiator.simple import MultiMeasurementInitiator
from stonesoup.measures import Euclidean
from stonesoup.models.measurement.linear import LinearGaussian
from stonesoup.models.transition.linear import (
    CombinedLinearGaussianTransitionModel,
    ConstantVelocity,
)
from stonesoup.predictor.kalman import KalmanPredictor
from stonesoup.tracker.simple import MultiTargetTracker
from stonesoup.types.detection import Detection
from stonesoup.types.state import GaussianState
from stonesoup.types.track import Track
from stonesoup.updater.kalman import KalmanUpdater

from .online_detection_reader import OnlineDetectionReader


class WheelTracker:
    """Wheel tracker using the StoneSoup library"""

    def __init__(self) -> None:
        self.track_history_size: timedelta = timedelta(seconds=0.25)
        """Maximum history length. Used to prune history"""

        self.detector: OnlineDetectionReader = OnlineDetectionReader()
        """Custom detection reader implementation is used to feed Detections into the tracker"""

        # #### Parameter ####

        # Magnitude of noise created through model
        q_x: float = 0.01
        q_y: float = 0.01

        # Measurement noise matrix
        R: np.ndarray = np.diag([0.01, 0.01])

        # #### Kalman Filter ####

        # Transition model defines system matrix A and system noise matrix Q
        self.transition_model: CombinedLinearGaussianTransitionModel = (
            CombinedLinearGaussianTransitionModel(
                [ConstantVelocity(q_x), ConstantVelocity(q_y)]
            )
        )

        # Measurement model defines measurement matrix H and measurement noise matrix R
        self.measurement_model: LinearGaussian = LinearGaussian(
            # Number of state dimensions (position and velocity in 2D)
            ndim_state=4,
            mapping=[0, 2],  # Mapping measurement vector index to state index
            noise_covar=np.array(R),  # Covariance matrix of measurement R
        )

        # Use a Kalman style models
        self.predictor: KalmanPredictor = KalmanPredictor(self.transition_model)
        self.updater: KalmanUpdater = KalmanUpdater(self.measurement_model)

        # #### Association ####

        self.hypothesiser: DistanceHypothesiser = DistanceHypothesiser(
            self.predictor,
            self.updater,
            measure=Euclidean(),
            missed_distance=2.0,
        )
        self.data_associator: GNNWith2DAssignment = GNNWith2DAssignment(
            self.hypothesiser
        )

        self.deleter: UpdateTimeDeleter = UpdateTimeDeleter(
            time_since_update=self.track_history_size
        )

        self.initiator: MultiMeasurementInitiator = MultiMeasurementInitiator(
            prior_state=GaussianState(
                [[0], [0.35], [0], [0.35]], np.diag([0.02 / 3, 10, 0.02 / 3, 10])),
            measurement_model=self.measurement_model,
            deleter=self.deleter,
            data_associator=self.data_associator,
            updater=self.updater,
            min_points=4,
        )

        self.tracker: MultiTargetTracker = MultiTargetTracker(
            initiator=self.initiator,
            deleter=self.deleter,
            detector=self.detector,
            data_associator=self.data_associator,
            updater=self.updater,
        )

    def step(self, time: datetime, detections: Set[Detection], limit_valid_association: int) -> List[Track]:
        """Executes one tracking step. Sets the new `Detection` objects into the detection reader and asks the tracker
        to run once. Finally, prunes the track histories"""

        _, tracks_unfiltered = self.tracker.update_tracker(time, detections)

        tracks: List[Track] = [
            track
            for track in tracks_unfiltered
            if len(track.states) >= limit_valid_association
        ]

        return tracks
