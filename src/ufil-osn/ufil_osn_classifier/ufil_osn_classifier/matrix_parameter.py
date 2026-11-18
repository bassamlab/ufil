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


"""ROS-independent parsing of matrix parameters given as strings."""

import ast

import numpy as np


def parse_matrix(text, shape=(4, 4)):
    """
    Parse a matrix given as a Python literal, e.g. ``'[[1, 0], [0, 1]]'``.

    The string is evaluated with :func:`ast.literal_eval`, so only literals are accepted
    and no code is executed.

    :param text: Nested list of numbers as string.
    :param shape: Expected shape of the matrix.
    :return: The parsed matrix as numpy.matrix of floats.
    :raises ValueError: If the string is no numeric matrix literal of the expected shape.
    """
    try:
        value = ast.literal_eval(text)
    except (ValueError, TypeError, SyntaxError, MemoryError, RecursionError) as error:
        raise ValueError(f'Matrix parameter is no valid literal: {error}') from error
    try:
        matrix = np.array(value)
    except ValueError as error:
        raise ValueError(f'Matrix parameter is no regular matrix: {error}') from error
    if not (np.issubdtype(matrix.dtype, np.integer)
            or np.issubdtype(matrix.dtype, np.floating)):
        raise ValueError(f'Matrix parameter contains non-numeric values: {text!r}')
    if matrix.shape != tuple(shape):
        raise ValueError(
            f'Matrix parameter has shape {matrix.shape}, expected {tuple(shape)}')
    return np.matrix(matrix, dtype=float)
