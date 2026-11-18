// Copyright 2025 Chair of Embedded Software
// (Computer Science 11) - RWTH Aachen University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__GREEDY_ASSOCIATOR_HPP_
#define UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__GREEDY_ASSOCIATOR_HPP_

#include <vector>
#include <set>
#include "ufil_object_tracking/types/scalar.hpp"


template<typename T>
class GreedyAssociator
{
private:
  std::set<T> available_rows;
  std::set<T> available_columns;
  const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cost_matrix;
  Eigen::Array<bool, -1, -1> assignment_matrix_;
  int cost_matrix_rows = 0, cost_matrix_cols = 0;
  std::vector<int> row_minima_index;

  // fills the sets with all available Columns and Rows
  void fillSets()
  {
    for(int i = 0; i < cost_matrix_rows; i++) {
      available_rows.insert(i);
    }
    for(int i = 0; i < cost_matrix_cols; i++) {
      available_columns.insert(i);
    }
  }

  void allRowsMinima()
  {
    for(int i = 0; i < cost_matrix_rows; i++) {
      ufil::type::Scalar minimum = -1.0f;
      int minimum_index = 0;
      for(int j = 0; j < cost_matrix_cols; j++) {
        if(cost_matrix(i, j) < minimum || minimum < 0.0f) {
          minimum = cost_matrix(i, j);
          minimum_index = j;
        }
      }
      row_minima_index[i] = minimum_index;
    }
  }

  int findMinimum()
  {
    ufil::type::Scalar minimum = -1.0f;
    int minimum_row_index = 0;
    for(int i = 0; i < cost_matrix_rows; i++) {
      if(available_rows.count(i) &&
        (cost_matrix(i, row_minima_index[i]) < minimum || minimum < 0.0f))
      {
        minimum = cost_matrix(i, row_minima_index[i]);
        minimum_row_index = i;
      }
    }
    return minimum_row_index;
  }

  int findRowMinimum(int row)
  {
    ufil::type::Scalar minimum = -1.0f;
    int minimum_col_index = 0;
    for(int i = 0; i < cost_matrix_cols; i++) {
      if(available_columns.count(i) && (cost_matrix(row, i) < minimum || minimum < 0.0f)) {
        minimum = cost_matrix(row, i);
        minimum_col_index = i;
      }
    }
    return minimum_col_index;
  }

  void GenerateAssignmentMatrix()
  {
    while(!(available_rows.empty() || available_columns.empty())) {
      int minimum_row = findMinimum();
      // here i still need to check if the column is still available and erase the row
      if(available_columns.count(row_minima_index[minimum_row])) {
        assignment_matrix_(minimum_row, row_minima_index[minimum_row]) = true;
        available_columns.erase(row_minima_index[minimum_row]);
        available_rows.erase(minimum_row);
      } else {
        row_minima_index[minimum_row] = findRowMinimum(minimum_row);
      }
    }
  }

public:
  // Default object constructor, cost function matrix must be set later
  GreedyAssociator() = default;

  // Create the Jonker-Volgenant algorithm object using the cost_matrix_
  explicit GreedyAssociator(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> & input_cost_matrix)
  : cost_matrix (input_cost_matrix)
  {
  }
  // Wrapper to execute all steps of the Greedy algorithm and solve the assignment problem
  void SolveAssignmentProblem()
  {
    cost_matrix_rows = cost_matrix.rows();
    cost_matrix_cols = cost_matrix.cols();
    row_minima_index.resize(cost_matrix_rows);
    assignment_matrix_.setConstant(cost_matrix_rows, cost_matrix_cols, false);
    fillSets();
    allRowsMinima();
    GenerateAssignmentMatrix();
  }
    // Get the assignment matrix after solving the problem
  void GetAssignmentMatrix(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> & outMatrix)
  {
    if ((outMatrix.rows() != cost_matrix_rows) || (outMatrix.cols() != cost_matrix_cols)) {
      throw std::invalid_argument(
        "The input matrix dimensions is inconsistent with the assignment matrix!");
    }
    // Copy assignment matrix to the output
    outMatrix = assignment_matrix_.block(0, 0, cost_matrix_rows, cost_matrix_cols).cast<int>();
  }
};

// Explicit template instantiations
template class GreedyAssociator<int>;
template class GreedyAssociator<float>;
template class GreedyAssociator<double>;

#endif  // UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__GREEDY_ASSOCIATOR_HPP_
