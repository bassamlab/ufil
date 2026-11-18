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


"""Unit tests of the parsing of matrix parameters."""

import numpy as np
import pytest

from ufil_osn_classifier.matrix_parameter import parse_matrix

# parse_matrix returns numpy.matrix like the former eval()-based parsing in the node.
pytestmark = pytest.mark.filterwarnings('ignore::PendingDeprecationWarning')

T_CAMERA_RIGHT_LIDAR = """[
            [-0.26207, 0.96484, 0.01995, -0.29462],
            [0.07057, -0.00146, 0.99751, 0.20912],
            [0.96247, 0.26282, -0.06771, 0.97677],
            [0.00000, 0.00000, 0.00000, 1.00000]
        ]"""


def test_parses_multiline_default():
    matrix = parse_matrix(T_CAMERA_RIGHT_LIDAR)
    assert isinstance(matrix, np.matrix)
    assert matrix.shape == (4, 4)
    assert matrix.dtype == float
    assert matrix[0, 0] == pytest.approx(-0.26207)
    assert matrix[2, 3] == pytest.approx(0.97677)
    np.testing.assert_array_equal(matrix[3], [[0, 0, 0, 1]])


def test_parses_integers_and_tuples():
    matrix = parse_matrix('((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))')
    np.testing.assert_array_equal(matrix, np.eye(4))


def test_custom_shape():
    assert parse_matrix('[[1, 2, 3]]', (1, 3)).shape == (1, 3)


@pytest.mark.parametrize('text', [
    '[[1, 0], [0, 1]]',
    '[1, 0, 0, 0]',
    '[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]',
    '[[1, 0, 0, 0], [0, 1, 0], [0, 0, 1, 0], [0, 0, 0, 1]]',
    '[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, "1"]]',
    '[[1j, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]',
    '',
    'not a matrix',
    '[[1, 0, 0, 0],',
])
def test_rejects_invalid_matrices(text):
    with pytest.raises(ValueError):
        parse_matrix(text)


def test_does_not_execute_code(tmp_path):
    marker = tmp_path / 'executed'
    text = f'__import__("pathlib").Path({str(marker)!r}).touch() or np.eye(4).tolist()'
    with pytest.raises(ValueError):
        parse_matrix(text)
    assert not marker.exists()
