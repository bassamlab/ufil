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


"""ROS-independent fusion of per-class confidences of tracked objects."""

import logging

import numpy as np

NUM_CLASSES = 7
"""Number of entries of a UFIL classification vector."""

_LOGGER = logging.getLogger(__name__)


class ConfidenceStorage:
    """Stores the smoothed classification of every track that was seen by a camera."""

    def __init__(self):
        """Create an empty storage."""
        self.storage = []

    def add_value(self, track_id, conf):
        """Add a new confidence value for the track ``track_id`` to the storage."""
        self.storage = np.append(self.storage, ConfidenceValue(track_id, conf))

    def check_for_id(self, track_id):
        """
        Look up the stored confidence value of a track.

        :return: The ConfidenceValue of ``track_id`` or False if it is not stored.
        """
        for conf in self.storage:
            if conf.id == track_id:
                return conf
        return False

    def fuse_log(self, new_conf, old_log, alpha):
        """Fuse new confidence values into the log-odds of ``old_log`` using decay ``alpha``."""
        eps = np.finfo(float).eps  # Small value to prevent division by zero
        log_prior = np.log((1 / NUM_CLASSES) / (1 - (1 / NUM_CLASSES)))
        for i in range(NUM_CLASSES):
            new_conf.set_log_conf(i, max(min(new_conf.log_conf[i], 1 - eps), eps))
            new_log = np.log(new_conf.log_conf[i] / (1 - new_conf.log_conf[i]))
            updated_log = (new_log - log_prior + alpha * old_log.log_conf[i])
            old_log.set_log_conf(i, updated_log)
            old_log.seen()
        return old_log

    def log_to_conf(self, logs):
        """Convert log-odds values to normalised confidence values."""
        bel = 1 - (1 / (1 + np.exp(logs.log_conf)))
        return bel / np.sum(bel)

    def call_fusion(self, track_id, conf, alpha):
        """
        Exponentially smooth the stored classification of a track with a new one.

        A track that is not stored yet starts from the uniform distribution.

        :param track_id: ID of the track.
        :param conf: New classification vector with NUM_CLASSES entries.
        :param alpha: Smoothing factor, weight of the new classification.
        :return: The updated (stored) classification vector.
        """
        old_log = self.check_for_id(track_id)
        if not old_log:
            self.add_value(track_id, np.full(NUM_CLASSES, 1 / NUM_CLASSES))
            old_log = self.check_for_id(track_id)
        for i in range(NUM_CLASSES):
            fused_conf = alpha * conf[i] + (1 - alpha) * old_log.log_conf[i]
            old_log.set_log_conf(i, fused_conf)
            old_log.seen()
        return old_log.log_conf

    def keep_fusion(self, track_id):
        """
        Keep the stored classification of a track that was not classified in this frame.

        Must only be called for tracks that are stored (see check_for_id). Otherwise a
        new entry is created, an error is logged and None is returned.
        """
        old_log = self.check_for_id(track_id)
        if not old_log:
            l_0 = np.log((1 - 1 / NUM_CLASSES) / (1 / NUM_CLASSES))
            self.add_value(track_id, np.full(NUM_CLASSES, l_0))
            _LOGGER.error('keep_fusion() called for unknown track id %s', track_id)
            return None
        old_log.seen()
        return old_log.log_conf

    def kill_expred(self):
        """Remove entries that were not seen since the last call and reset the seen flags."""
        for conf in self.storage:
            if not conf.was_seen:
                self.storage = np.delete(self.storage, self.storage == conf)
            else:
                conf.reset_seen()


class ConfidenceValue:
    """Classification of a single track together with a seen flag."""

    def __init__(self, track_id, log_conf):
        """Create a value for track ``track_id`` that has not been seen yet."""
        self.id = track_id
        self.log_conf = log_conf
        self.was_seen = False

    def set_log_conf(self, index, value):
        """Set the confidence value at a specific index."""
        self.log_conf[index] = value

    def seen(self):
        """Mark the confidence value as seen."""
        self.was_seen = True

    def reset_seen(self):
        """Reset the seen flag for the confidence value."""
        self.was_seen = False


def conf_fusion(confs, tracker_trust, camera_trust, track_conf=None):
    """
    Fuse the camera classifications of a track with the tracker classification.

    Without a valid tracker classification (None or fewer than NUM_CLASSES entries) the
    camera classifications are averaged. Otherwise the weighted sum of the camera
    classifications (weight ``camera_trust`` each) and the tracker classification
    (weight ``tracker_trust``) is used. The result is normalised to sum up to 1.

    :param confs: Non-empty list of camera classification vectors.
    :param tracker_trust: Weight of the tracker classification.
    :param camera_trust: Weight of each camera classification.
    :param track_conf: Classification vector of the tracker.
    :return: Fused classification vector with NUM_CLASSES entries.
    """
    fused_conf = np.zeros(NUM_CLASSES)
    if track_conf is None or len(track_conf) < NUM_CLASSES:
        for conf in confs:
            fused_conf += conf * (1 / len(confs))
    else:
        for conf in confs:
            fused_conf += conf * camera_trust
        fused_conf += track_conf * tracker_trust
    return fused_conf / sum(fused_conf)
