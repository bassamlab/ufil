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


"""Unit tests of the ROS-independent confidence fusion."""

import numpy as np
import pytest

from ufil_osn_classifier.conf_fusion import conf_fusion, ConfidenceStorage, NUM_CLASSES


def one_hot(index, conf=1.0):
    """Return a classification vector with ``conf`` at ``index`` and the rest uniform."""
    vector = np.full(NUM_CLASSES, (1.0 - conf) / (NUM_CLASSES - 1))
    vector[index] = conf
    return vector


def test_conf_fusion_averages_cameras_without_tracker_classification():
    fused = conf_fusion([one_hot(0), one_hot(1)], 0.2, 0.4)
    expected = np.zeros(NUM_CLASSES)
    expected[0] = expected[1] = 0.5
    np.testing.assert_allclose(fused, expected)


def test_conf_fusion_ignores_too_short_tracker_classification():
    fused = conf_fusion([one_hot(2)], 0.2, 0.4, [None])
    np.testing.assert_allclose(fused, one_hot(2))


def test_conf_fusion_weights_tracker_and_cameras():
    camera = one_hot(0)
    tracker = one_hot(4)
    fused = conf_fusion([camera], 0.2, 0.4, tracker)
    expected = (0.4 * camera + 0.2 * tracker) / 0.6
    np.testing.assert_allclose(fused, expected)
    assert fused.sum() == pytest.approx(1.0)


def test_conf_fusion_two_cameras_and_tracker_is_normalised():
    cameras = [one_hot(0, 0.8), one_hot(1, 0.6)]
    tracker = one_hot(1, 0.5)
    fused = conf_fusion(cameras, 0.2, 0.4, tracker)
    expected = 0.4 * cameras[0] + 0.4 * cameras[1] + 0.2 * tracker
    np.testing.assert_allclose(fused, expected / expected.sum())
    assert fused.sum() == pytest.approx(1.0)


def test_storage_starts_empty():
    storage = ConfidenceStorage()
    assert storage.check_for_id(1) is False


def test_call_fusion_smooths_from_uniform_prior():
    storage = ConfidenceStorage()
    conf = one_hot(3)
    result = storage.call_fusion(7, conf, 0.1)
    expected = 0.1 * conf + 0.9 * np.full(NUM_CLASSES, 1 / NUM_CLASSES)
    np.testing.assert_allclose(result, expected)
    assert storage.check_for_id(7) is not False
    assert storage.check_for_id(7).was_seen

    result = storage.call_fusion(7, conf, 0.1)
    np.testing.assert_allclose(result, 0.1 * conf + 0.9 * expected)
    assert result.sum() == pytest.approx(1.0)


def test_call_fusion_with_factor_one_takes_new_classification():
    storage = ConfidenceStorage()
    conf = one_hot(5, 0.7)
    np.testing.assert_allclose(storage.call_fusion(1, conf, 1.0), conf)


def test_keep_fusion_returns_stored_classification():
    storage = ConfidenceStorage()
    stored = storage.call_fusion(3, one_hot(0), 0.5).copy()
    storage.check_for_id(3).reset_seen()
    np.testing.assert_allclose(storage.keep_fusion(3), stored)
    assert storage.check_for_id(3).was_seen


def test_keep_fusion_for_unknown_id_returns_none():
    storage = ConfidenceStorage()
    assert storage.keep_fusion(42) is None
    assert storage.check_for_id(42) is not False


def test_kill_expred_removes_unseen_entries_and_resets_seen():
    storage = ConfidenceStorage()
    storage.call_fusion(1, one_hot(0), 0.1)
    storage.call_fusion(2, one_hot(1), 0.1)

    # Both were seen: kept, flags reset.
    storage.kill_expred()
    assert storage.check_for_id(1) is not False
    assert storage.check_for_id(2) is not False
    assert not storage.check_for_id(1).was_seen

    # Only track 2 is seen again: track 1 expires.
    storage.keep_fusion(2)
    storage.kill_expred()
    assert storage.check_for_id(1) is False
    assert storage.check_for_id(2) is not False
