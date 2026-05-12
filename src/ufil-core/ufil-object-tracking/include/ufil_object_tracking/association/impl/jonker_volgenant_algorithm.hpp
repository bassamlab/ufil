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

#ifndef UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__JONKER_VOLGENANT_ALGORITHM_HPP_
#define UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__JONKER_VOLGENANT_ALGORITHM_HPP_

#include <algorithm>
#include <iostream>
#include <limits>
#include <random>
#include <string>

#include "ufil_object_tracking/types/matrix.hpp"

#define IsApproxZERO(X) ((X) <= 1e-6)

template<typename T>
class JonkerVolgenantAlgorithm
{
private:
  // Dimensions of the cost function matrix
  int number_of_rows_, number_of_cols_;
  // Size of the cost function matrix
  int matrix_size_;
  // Original cost function matrix
  Eigen::Matrix<T, -1, -1> cost_matrix_;
  // Editable work matrix
  Eigen::Matrix<T, -1, -1> working_cost_matrix_;
  // Assignment matrix
  Eigen::Array<bool, -1, -1> assignment_matrix_;
  // List of unassigned rows
  Eigen::VectorXi list_of_unassinged_rows_;
  // Column assigned to row in solution
  Eigen::VectorXi row_assigned_;
  // Row assigned to column in solution
  Eigen::VectorXi col_assigned_;
  // Number of unassigned rows
  int number_of_unassinged_rows_ = 0;
  // List of columns to be scanned
  Eigen::VectorXi list_unscanned_cols_;
  // 'Cost-distance' for augmenting path calculation
  Eigen::Vector<T, -1> d;
  // Row-predecessor of column in augmenting/alternating path
  Eigen::Vector<T, -1> pred;
  // Dual variables, column reduction numbers
  Eigen::Vector<T, -1> v;
  //
  Eigen::VectorXi matches;
  // Variable to indicate current status
  ProblemStatus problemStatus = ProblemStatus::NotReady;

  // Step 1: Initialization Column Reduction
  void SubtractColMinima()
  {
    // // Subtract the minimum value in each column
    // for (Eigen::Index col = 0; col < working_cost_matrix_.cols(); col++)
    // {
    //   T colMinCoeff = working_cost_matrix_.col(col).minCoeff();
    //   working_cost_matrix_.col(col).array() -= colMinCoeff;
    // }
  }

  // Find assignments
  void SolveColumnAssignment()
  {
    // Initialize matches vector to zero
    matches.setZero();

    // Iterate over columns in reverse order
    for (int j = matrix_size_ - 1; j >= 0; j--) {
      // Find the minimum cost in column j and its corresponding row index
      Eigen::Index imin;
      T minCost = working_cost_matrix_.col(j).minCoeff(&imin);

      v[j] = minCost;

      if (matches[imin] == 0) {
        // First assignment for this row
        row_assigned_[imin] = j;
        col_assigned_[j] = imin;
      } else if (minCost < v[row_assigned_[imin]]) {
        // Reassign row to column j and unassign previous column
        Eigen::Index j1 = row_assigned_[imin];
        row_assigned_[imin] = j;
        col_assigned_[j1] = -1;
        col_assigned_[j] = imin;
      } else {
        // Keep the previous assignment and mark column j as unassigned
        col_assigned_[j] = -1;
      }

      // Increment match count for the row
      matches[imin]++;
    }
  }

  // Step 2: Reduction Transfer
  void TransferReductions()
  {
    for (int i = 0; i < matrix_size_; i++) {
      // Fill list of unassigned rows
      if (matches[i] == 0) {
        list_of_unassinged_rows_[number_of_unassinged_rows_] = i;
        number_of_unassinged_rows_++;
      } else if (matches[i] == 1) {
        // Transfer reduction from rows that are assigned once
        Eigen::Index j1 = row_assigned_[i];
        T min = std::numeric_limits<T>::max();

        for (int j = 0; j < matrix_size_; j++) {
          if (j != j1 && (working_cost_matrix_(i, j) - v[j] < min)) {
            // Calculate cost of current row
            min = working_cost_matrix_(i, j) - v[j];
          }
        }

        // Reduce cost of assigned column
        v[j1] = v[j1] - min;
      }
    }
    // Reset counter for unassigned rows
    // number_of_unassinged_rows_ = 0;

    // for (int i = 0; i < matrix_size_; i++)
    // {
    //   if (matches[i] == 0)
    //   {
    //     // Store unassigned rows
    //     list_of_unassinged_rows_[number_of_unassinged_rows_] = i;
    //     number_of_unassinged_rows_++;
    //   }
    //   else if (matches[i] == 1)
    //   {
    //     // Transfer reduction from rows assigned exactly once
    //     Eigen::Index j1 = row_assigned_[i];

    //     // Compute minimum cost difference for the row, excluding j1
    //     Eigen::Vector<T, -1> cost_differences = working_cost_matrix_.row(i).transpose().array()
    // - v.array();

    //     // Set column j1 to a large value so it doesn't affect minCoeff()
    //     cost_differences[j1] = std::numeric_limits<T>::max();

    //     // Get the minimum cost while excluding j1
    //     T min = cost_differences.minCoeff();

    //     // Reduce cost of assigned column
    //     v[j1] -= min;
    //   }
    // }
  }

  // Step 3: Augmenting reduction of unassigned rows
  void AugmentingReduction()
  {
    for (int loop = 0; loop < 2; loop++) {
      // scan all start_row_of_path rows
      int k = 0;
      int number_of_unassinged_rows_1 = number_of_unassinged_rows_;
      number_of_unassinged_rows_ = 0;
      while (k < number_of_unassinged_rows_1) {
        int i = list_of_unassinged_rows_[k];
        k++;
        // std::cout << i << " " << v[0] << std::endl;
        // find minimum and second minimum and reduce cost over columns
        T min1 = working_cost_matrix_(i, 0) - v[0];
        int j1 = 0;
        int j2 = -1;
        T min2 = std::numeric_limits<T>::max();

        for (int j = 1; j < matrix_size_; j++) {
          T x = working_cost_matrix_(i, j) - v[j];
          if (x < min2) {
            if (x >= min1) {
              min2 = x;
              j2 = j;
            } else {
              min2 = min1;
              min1 = x;
              j2 = j1;
              j1 = j;
            }
          }
        }

        int i0 = col_assigned_[j1];

        if (min1 < min2) {
          // change the reduction of the minimum column to increase the minimum reduced cost in the
          // row to the second minimum
          v[j1] = v[j1] - (min2 - min1);
        } else if (i0 >= 0) {
          // minimum and second minimum are equal and minimum column is assigned
          // j2 could be unassigned so we change collumns
          j1 = j2;
          i0 = col_assigned_[j2];
        }

        // (re)assign row i to coulumn j1
        row_assigned_[i] = j1;
        col_assigned_[j1] = i;

        if (i0 >= 0) {
          if (min1 < min2) {
            // go back to current k and continue augmenting path i to j1
            list_of_unassinged_rows_[--k] = i0;
          } else {
            // no further augmenting reduction possible, therefore store i0 as start_row_of_path
            // row for next phase
            list_of_unassinged_rows_[number_of_unassinged_rows_++] = i0;
          }
        }
      }
    }
    // for (int loop = 0; loop < 2; loop++)
    // {
    //   int number_of_unassinged_rows_1 = number_of_unassinged_rows_;
    //   number_of_unassinged_rows_ = 0;
    //   int k = 0;

    //   while (k < number_of_unassinged_rows_1)
    //   {
    //     int i = list_of_unassinged_rows_[k++];

    //     // Compute reduced costs for row i
    //     Eigen::Vector<T, -1> cost_reduction = working_cost_matrix_.row(i).array() - v.array();

    //     // Find the two smallest values and their indices
    //     Eigen::Index j1, j2;
    //     T min1 = cost_reduction.minCoeff(&j1);
    //     cost_reduction[j1] = std::numeric_limits<T>::max();  // Temporarily exclude min1
    //     T min2 = cost_reduction.minCoeff(&j2);

    //     int i0 = col_assigned_[j1];

    //     if (min1 < min2)
    //     {
    //       // Increase the minimum reduced cost to match min2
    //       v[j1] -= (min2 - min1);
    //     }
    //     else if (i0 >= 0)
    //     {
    //       // Swap to second minimum column if j1 is already assigned
    //       j1 = j2;
    //       i0 = col_assigned_[j2];
    //     }

    //     // Assign row i to column j1
    //     row_assigned_[i] = j1;
    //     col_assigned_[j1] = i;

    //     if (i0 >= 0)
    //     {
    //       if (min1 < min2)
    //       {
    //         // Continue augmenting path for i0 by backtracking
    //         list_of_unassinged_rows_[--k] = i0;
    //       }
    //       else
    //       {
    //         // No further augmentation possible, store i0 for next phase
    //         list_of_unassinged_rows_[number_of_unassinged_rows_++] = i0;
    //       }
    //     }
    //   }
    // }
  }

  // Step 4: Augment Solution for each Row
  void RowAugmentSolution()
  {
    for (int c = 0; c < number_of_unassinged_rows_; c++) {
      // start row of augmenting path (or current row of augmenting path)
      int start_row_of_path = list_of_unassinged_rows_[c];
      // end row of augmenting path
      int end_of_path = 0;

      // Run Dijkstras shortest path algorithm until column is added to shortest path tree
      for (int j = matrix_size_; j--; ) {
        d[j] = working_cost_matrix_(start_row_of_path, j) - v[j];
        pred[j] = start_row_of_path;
        list_unscanned_cols_[j] = j;
      }  // count columns that are ready from from 0 to low-1
      int low = 0;
      // count columns that need to be checked for current minimum from low to up-1
      // columns from up to matrix_size_-1 need to be checked later to find new minimum
      int up = 0;

      int last = 0;

      bool unassigned = false;
      T min = d[list_unscanned_cols_[up++]];

      do{
        if (up == low) {
          last = low - 1;
          min = d[list_unscanned_cols_[up++]];

          // find all columns up...matrix_size_-1 where new minimum occurs and store indices in low
          for (int k = up; k < matrix_size_; k++) {
            int j = list_unscanned_cols_[k];
            T h = d[j];
            if (h <= min) {
              if (h < min) {
                // h is new minimum
                up = low;
                min = h;
              }

              // new index with same minimum extends list
              list_unscanned_cols_[k] = list_unscanned_cols_[up];
              list_unscanned_cols_[up++] = j;
            }
          }

          // check if all minimum columns are assigned
          for (int k = low; k < up; k++) {
            if (col_assigned_[list_unscanned_cols_[k]] < 0) {
              // augmenting path right away
              end_of_path = list_unscanned_cols_[k];
              unassigned = true;
              break;
            }
          }
        }

        if (!unassigned) {
          // update the distances between start_row_of_path and the unscanned columns vie next
          // scanned column
          int j1 = list_unscanned_cols_[low];
          low++;
          int i = col_assigned_[j1];
          T x = working_cost_matrix_(i, j1) - v[j1] - min;

          for (int k = up; k < matrix_size_; k++) {
            int j = list_unscanned_cols_[k];
            T v2 = working_cost_matrix_(i, j) - v[j] - x;

            if (v2 < d[j]) {
              pred[j] = i;

              if (v2 == min) {
                if (col_assigned_[j] < 0) {
                  end_of_path = j;
                  unassigned = true;
                  break;
                } else {
                  list_unscanned_cols_[k] = list_unscanned_cols_[up];
                  list_unscanned_cols_[up++] = j;
                }
              }
              d[j] = v2;
            }
          }
        }
      } while (!unassigned);

      // update column costs
      for (int l = 0; l <= last; l++) {
        int j1 = list_unscanned_cols_[l];
        v[j1] = v[j1] + d[j1] - min;
      }

      // reset row and column assignments along the alternating path
      int i;
      do{
        i = pred[end_of_path];
        col_assigned_[end_of_path] = 1;
        int j1 = end_of_path;
        end_of_path = row_assigned_[i];
        row_assigned_[i] = j1;
      } while (i != start_row_of_path);
    }
  }

  // Step 5: Generate assignment matrix
  void GenerateAssignmentMatrix()
  {
    for (int i = 0; i < matrix_size_; i++) {
      assignment_matrix_(i, row_assigned_[i]) = true;
    }
  }

public:
  // Default object constructor, cost function matrix must be set later
  JonkerVolgenantAlgorithm() = default;

  // Create the Jonker-Volgenant algorithm object using the cost_matrix_
  explicit JonkerVolgenantAlgorithm(
    const Eigen::Matrix<T, Eigen::Dynamic,
    Eigen::Dynamic> & input_cost_matrix)
  {
    // Set cost function matrix
    SetCostFunctionMatrix(input_cost_matrix);
    // Update problemStatus
    problemStatus = ProblemStatus::ReadyToSolve;
  }

  // Set the cost function matrix
  void SetCostFunctionMatrix(
    const Eigen::Matrix<T, Eigen::Dynamic,
    Eigen::Dynamic> & input_cost_matrix)
  {
    // Check if the input matrix contains any negative values
    if ((input_cost_matrix.array() < 0).any()) {
      throw std::invalid_argument("The cost function matrix cannot contain negative values!");
    }

    // Save the matrix size
    number_of_rows_ = static_cast<int>(input_cost_matrix.rows());
    number_of_cols_ = static_cast<int>(input_cost_matrix.cols());

    // Check if the cost_matrix_ is not square
    if (number_of_rows_ != number_of_cols_) {
      // Get the size for the used matrices
      matrix_size_ = std::max(number_of_cols_, number_of_rows_);
      // Initialize cost function matrix with the dummyCost
      cost_matrix_.setConstant(matrix_size_, matrix_size_, input_cost_matrix.maxCoeff());

      // std::random_device r;
      // std::default_random_engine e1(r());
      // std::uniform_real_distribution<T> uniform_dist(1, 1000);

      // for (int i = 0; i < matrix_size_; i++)
      // {
      //   for (int j = 0; j < matrix_size_; j++)
      //   {
      //     T value = uniform_dist(e1);
      //     cost_matrix_(j, i) = cost_matrix_(j, i) + value;
      //   }
      // }

      // Copy relevant data from the cost_matrix_
      cost_matrix_.block(0, 0, number_of_rows_, number_of_cols_) = input_cost_matrix;
    } else {
      // Get the size for the used matrices
      matrix_size_ = number_of_rows_;
      // Initialize cost function matrix with input matrix
      cost_matrix_.resize(matrix_size_, matrix_size_);
      cost_matrix_ = input_cost_matrix;
    }

    // Initialize all vectors
    list_of_unassinged_rows_.setConstant(matrix_size_, 0);
    matches.setConstant(matrix_size_, 0);
    pred.setConstant(matrix_size_, -1);
    list_unscanned_cols_.setConstant(matrix_size_, 0);
    d.setConstant(matrix_size_, 0);
    v.setConstant(matrix_size_, 0);

    row_assigned_.setConstant(matrix_size_, -1);
    col_assigned_.setConstant(matrix_size_, -1);
    number_of_unassinged_rows_ = 0;

    // Copy the cost_matrix_ contents to the working_cost_matrix_
    working_cost_matrix_ = cost_matrix_;
    // Initialize assignment_matrix_ with false
    assignment_matrix_.setConstant(matrix_size_, matrix_size_, false);
    // Update problemStatus
    problemStatus = ProblemStatus::ReadyToSolve;
  }

  // Get the cost function matrix
  void GetCostFunctionMatrix(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> & outMatrix)
  {
    if (problemStatus < ProblemStatus::ReadyToSolve) {
      throw std::invalid_argument("The cost function matrix is undefined!");
    }
    if ((outMatrix.rows() != number_of_rows_) || (outMatrix.cols() != number_of_cols_)) {
      throw std::invalid_argument(
        "The input matrix dimensions is inconsistent with the cost function matrix!");
    }
    // Copy cost function matrix to the output
    outMatrix = cost_matrix_;
  }

  // Get the assignment matrix after solving the problem
  void GetAssignmentMatrix(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> & outMatrix)
  {
    if (problemStatus < ProblemStatus::Done) {
      throw std::invalid_argument("The assignment problem has not been solved yet!");
    }
    if ((outMatrix.rows() != number_of_rows_) || (outMatrix.cols() != number_of_cols_)) {
      throw std::invalid_argument(
        "The input matrix dimensions is inconsistent with the assignment matrix!");
    }
    // Copy assignment matrix to the output
    outMatrix = assignment_matrix_.block(0, 0, number_of_rows_, number_of_cols_).cast<int>();
  }

  // Get current problem status
  ProblemStatus getProblemStatus()
  {
    return problemStatus;
  }

  // Get current problem status name
  std::string getProblemStatusName()
  {
    return ProblemStatusName[problemStatus];
  }

  // Wrapper to execute all steps of the Hungarian algorithm and solve the assignment problem
  void SolveAssignmentProblem()
  {
    if (problemStatus < ProblemStatus::ReadyToSolve) {
      throw std::invalid_argument("The cost function matrix is undefined!");
    }

    // Step 1
    SubtractColMinima();
    SolveColumnAssignment();

    // Step 2
    TransferReductions();

    // Step 3
    AugmentingReduction();

    // Step 4
    RowAugmentSolution();

    // Step 5
    GenerateAssignmentMatrix();

    // Assignment is done
    problemStatus = ProblemStatus::Done;
  }
};

template class JonkerVolgenantAlgorithm<int>;
template class JonkerVolgenantAlgorithm<float>;
template class JonkerVolgenantAlgorithm<double>;

#endif  // UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__JONKER_VOLGENANT_ALGORITHM_HPP_
