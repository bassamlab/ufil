// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#ifndef UFIL_OBJECT_TRACKING__ASSOCIATION__ASSOCIATION_FUNCTIONS_HPP_
#define UFIL_OBJECT_TRACKING__ASSOCIATION__ASSOCIATION_FUNCTIONS_HPP_

#include <iostream>
#include <algorithm>

#include "ufil_object_tracking/association/hypothesize_functions.hpp"

#include "ufil_object_tracking/association/impl/hungarian_algorithm.hpp"
#include "ufil_object_tracking/association/impl/jonker_volgenant_algorithm.hpp"
#include "ufil_object_tracking/association/impl/lp_solver.hpp"
#include "ufil_object_tracking/association/impl/greedy_associator.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/utility_functions.hpp"

namespace ufil
{
namespace association
{
inline void hungarian(
  const type::CostMatrix & cost_matrix,
  type::AssignmentMatrix & assignment_matrix)
{
  const auto n_rows = cost_matrix.rows();
  const auto n_cols = cost_matrix.cols();

  // Resize and initialize assignment matrix
  assignment_matrix.resize(n_rows, n_cols);
  assignment_matrix.setZero();

  // No assignment if matrix is empty
  if (!(n_rows > 0 && n_cols > 0)) {
    return;
  }

  // Create and solve assignment problem with hungarian algorithm
  auto problem = HungarianAlgorithm<ufil::type::Scalar>(cost_matrix);
  problem.SolveAssignmentProblem();

  // Set assignment matrix with solution
  problem.GetAssignmentMatrix(assignment_matrix);
}

inline void greedy(
  const type::CostMatrix & cost_matrix, type::AssignmentMatrix & assignment_matrix)
{
  const auto n_rows = cost_matrix.rows();
  const auto n_cols = cost_matrix.cols();

  // Resize and initialize assignment matrix
  assignment_matrix.resize(n_rows, n_cols);
  assignment_matrix.setZero();

  // No assignment if matrix is empty
  if (!(n_rows > 0 && n_cols > 0)) {
    return;
  }

  // Create and solve assignment problem with hungarian algorithm
  auto problem = GreedyAssociator<ufil::type::Scalar>(cost_matrix);
  problem.SolveAssignmentProblem();

  // Set assignment matrix with solution
  problem.GetAssignmentMatrix(assignment_matrix);
}

inline void jonker_volgenant(
  const type::CostMatrix & cost_matrix,
  type::AssignmentMatrix & assignment_matrix)
{
  const auto n_rows = cost_matrix.rows();
  const auto n_cols = cost_matrix.cols();

  // Resize and initialize assignment matrix
  assignment_matrix.resize(n_rows, n_cols);
  assignment_matrix.setZero();

  // No assignment if matrix is empty
  if (!(n_rows > 0 && n_cols > 0)) {
    return;
  }

  // Create and solve assignment problem with Jonker-Volgenant algorithm
  auto problem = JonkerVolgenantAlgorithm<ufil::type::Scalar>(cost_matrix);
  problem.SolveAssignmentProblem();

  // Set assignment matrix with solution
  problem.GetAssignmentMatrix(assignment_matrix);
}

inline void lp_solver(
  const type::CostMatrix & cost_matrix,
  type::AssignmentMatrix & assignment_matrix)
{
  const auto n_rows = cost_matrix.rows();
  const auto n_cols = cost_matrix.cols();

  // No assignment if matrix is empty
  if (!(n_rows > 0 && n_cols > 0)) {
    return;
  }

  const type::Scalar max_coeff = std::max(cost_matrix.maxCoeff(), static_cast<type::Scalar>(1.0));
  type::CostMatrix cost_matrix_invers = type::CostMatrix::Constant(n_rows, n_cols,
        max_coeff) - cost_matrix;

  // Resize and initialize assignment matrix
  assignment_matrix.resize(n_rows, n_cols);
  assignment_matrix.setZero();

  // Create and solve assignment problem with Jonker-Volgenant algorithm
  auto problem = LP_Solver<ufil::type::Scalar>(cost_matrix_invers);
  problem.SolveAssignmentProblem();

  type::AssignmentMatrix assignment_matrix_padded;
  assignment_matrix_padded.resize(n_rows, n_cols);
  assignment_matrix_padded.setZero();

  // Set assignment matrix with solution
  problem.GetAssignmentMatrix(assignment_matrix_padded);

  assignment_matrix.resize(n_rows, n_cols);
  assignment_matrix.setZero();
  assignment_matrix = assignment_matrix_padded.block(0, 0, n_rows, n_cols);
}

}  // namespace association
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__ASSOCIATION__ASSOCIATION_FUNCTIONS_HPP_
