//  Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
//  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.

// ----------------------------------------------------------------------------------//
//  MIT License
//
//  Copyright (c) [2020-] [Mostafa Emam]
//
//  Author(s): Mostafa Emam (mostafa.emam92@gmail.com)
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
// ----------------------------------------------------------------------------------//

#ifndef UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__HUNGARIAN_ALGORITHM_HPP_
#define UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__HUNGARIAN_ALGORITHM_HPP_

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

//  Check if a value is approximately zero (only positive values are expected in the
//  cost function). The macro is better here as it is used for simple values, arrays,
//  and matrices.
#define IsApproxZERO(X) ((X) <= 1e-6)

// ----------------------------------------------------------------------------------//
//  Enumeration for the state of the assignment problem
//
//   ELEMENTS
//       NotReady:       Problem initialized without cost function
//       ReadyToSolve:   Problem initialized with cost function, ready to solve
//       Done:           Problem sovled
// ----------------------------------------------------------------------------------//
enum ProblemStatus
{
  NotReady,
  ReadyToSolve,
  Done
};
static std::map<ProblemStatus, const char *> ProblemStatusName = {{NotReady, "NotReady"},
  {ReadyToSolve, "ReadyToSolve"},
  {Done, "Done"}};

// ----------------------------------------------------------------------------------//
//  An implementation of the Hungarian algorithm to solve optimal assignment problems.
//  The supported cost function matrix types are <int>, <float>, and <double>, and the
//  result assignment matrix has the type <int>.
//
//  Example:
//       Eigen::Matrix3f costFcnMatrix;
//       costFcnMatrix << 10.5, 22, 18, 42, 5.9, 6, 71.2, 8.4, 69;
//       auto problem = HungarianAlgorithm<float>(costFcnMatrix);
//       problem.SolveAssignmentProblem();
//       Eigen::MatrixXi assignmentMatrix(3, 3);
//       problem.GetAssignmentMatrix(assignmentMatrix);
//       std::cout << assignmentMatrix; //  [1, 0, 0; 0, 0, 1; 0, 1, 0]
// ----------------------------------------------------------------------------------//
template<typename T>
class HungarianAlgorithm
{
private:
  //  Dimensions of the cost function matrix
  Eigen::Index nrRows, nrCols;
  //  Size of the cost function matrix
  int matrixSize;
  //  Dummy cost to indicate a very large number (Inf)
  double dummyCost;
  //  Original cost function matrix
  Eigen::Matrix<T, -1, -1> costFunctionMatrix;
  //  Editable work matrix
  Eigen::Matrix<T, -1, -1> workingMatrix;
  //  Matrix used to determine the checked/covered elements
  Eigen::Array<bool, -1, -1> coveredMatrix;
  //  Minimum number of lines needed to cover all the zeroes in the workingMatrix
  int nrLinesToCoverZeroes;
  //  Assignment matrix
  Eigen::Array<bool, -1, -1> assignmentMatrix;
  //  Variable to indicate current status
  ProblemStatus problemStatus = ProblemStatus::NotReady;

  //  Step 1: Subtract row minima
  void SubtractRowMinima()
  {
    //  Subtract the minimum value in each row
    for (int row = 0; row < matrixSize; row++) {
      T rowMinCoeff = workingMatrix.row(row).minCoeff();
      //  Do the operation only if the coeff value is non-zero (reduces operation time)
      if (!IsApproxZERO(rowMinCoeff)) {
        workingMatrix.row(row).array() -= rowMinCoeff;
      }
    }
  }

  //  Step 2: Subtract column minima
  void SubtractColMinima()
  {
    //  Subtract the minimum value in each column
    for (int col = 0; col < matrixSize; col++) {
      T colMinCoeff = workingMatrix.col(col).minCoeff();
      //  Do the operation only if the coeff value is non-zero (reduces operation time)
      if (!IsApproxZERO(colMinCoeff)) {
        workingMatrix.col(col).array() -= colMinCoeff;
      }
    }
  }

  //  Step 3: Get the minimum number of lines to cover all the zeroes
  int MinNrOfLinesToCoverAllZeros()
  {
    //  Reset the variables used to check the covered elements in the matrix
    coveredMatrix.fill(false);
    nrLinesToCoverZeroes = 0;

    //  Total number of uncovered zeroes in the workingMatrix
    //  (workingMatrix == 0).count()
    int nrUncoveredZeroes = static_cast<int>((IsApproxZERO(workingMatrix.array())).count());

    //  Loop till all elements are checked
    while (nrUncoveredZeroes > 0) {
      //  Check if a new zero was covered in this iteration
      bool newZeroCovered = false;

      //  Start looking from the first uncovered zero
      //  ((workingMatrix == 0) && (!coveredMatrix)).index()
      int idxRow, idxCol;
      (IsApproxZERO(workingMatrix.array()) && (!coveredMatrix.array())).maxCoeff(&idxRow, &idxCol);

      //  Loop on all elements in the workingMatrix
      for (int row = idxRow; row < matrixSize; row++) {
        for (int col = idxCol; col < matrixSize; col++) {
          //  Check only uncovered elements
          if (!coveredMatrix(row, col)) {
            //  Check zero elements in the workingMatrix (with a certain precision)
            if (IsApproxZERO(workingMatrix(row, col))) {
              //  Check the total number of uncovered zero elements in the same row of the current
              // zero element ((workingMatrix.row() == 0) && (!coveredMatrix.row())).count()
              int nrZeroesInRow = static_cast<int>(
                (IsApproxZERO(workingMatrix.row(row).array()) &&
                (!coveredMatrix.row(row).array())).count());

              // Check the total number of uncovered zero elements in the same column of the
              // current zero element ((workingMatrix.col() == 0) && (!coveredMatrix.col())).count()
              int nrZeroesInCol = static_cast<int>(
                (IsApproxZERO(workingMatrix.col(col).array()) &&
                (!coveredMatrix.col(col).array())).count());

              //  Check if multiple zeroes were found
              if ((nrZeroesInRow > 1) || (nrZeroesInCol > 1)) {
                // If (total number of uncovered zeroes in the row) > (total number of uncovered
                // zeroes in the column) -> Cover the row
                if (nrZeroesInRow > nrZeroesInCol) {
                  //  Cover the row
                  coveredMatrix.row(row).fill(true);
                  nrLinesToCoverZeroes++;
                  nrUncoveredZeroes -= nrZeroesInRow;
                  //  A new zero was covered
                  newZeroCovered = true;
                } else if (nrZeroesInRow < nrZeroesInCol) {
                  //  Else if (total number of uncovered zeroes in the row) < (total number of
                  // uncovered zeroes in the column) -> Cover the column Cover the column
                  coveredMatrix.col(col).fill(true);
                  nrLinesToCoverZeroes++;
                  nrUncoveredZeroes -= nrZeroesInCol;
                  //  A new zero was covered
                  newZeroCovered = true;
                } else {
                  //  Else -> Skip element
                  //  A decision cannot be made based on this element alone to cover the row or the
                  // column. The next zeroes in the row/column will be checked before deciding
                }
              } else {
                //  Only one zero was found
                //  No difference in covering the row or the column, cover the row
                coveredMatrix.row(row).fill(true);
                nrLinesToCoverZeroes++;
                nrUncoveredZeroes -= nrZeroesInRow;
                //  A new zero was covered
                newZeroCovered = true;
              }
              //  If all zeroes are covered, no need for further checking
              if (nrUncoveredZeroes == 0) {
                return nrLinesToCoverZeroes;
              }
            }
          }
        }
      }

      //  Check if no new zeroes were covered in this iteration
      if (!newZeroCovered) {
        //  Multiple solutions are possible -> Cover the row for any set of zeroes
        //  Find the index of the first uncovered zero
        // ((workingMatrix == 0) && (!coveredMatrix)).index()
        int idxRow, idxCol;
        (IsApproxZERO(workingMatrix.array()) && (!coveredMatrix.array())).maxCoeff(&idxRow,
          &idxCol);
        //  Find the total number of uncovered zeroes in its row
        int nrZeroesInRow = static_cast<int>(
          (IsApproxZERO(workingMatrix.row(idxRow).array()) &&
          (!coveredMatrix.row(idxRow).array())).count());
        //  Cover the row
        coveredMatrix.row(idxRow).fill(true);
        nrLinesToCoverZeroes++;
        nrUncoveredZeroes -= nrZeroesInRow;
      }
    }
    //  return the total number of lines needed to cover the zeroes in the workingMatrix
    return nrLinesToCoverZeroes;
  }

  //  Step 4: Augment the matrix (Create additional zeroes)
  void AugmentCostFunctionMatrix()
  {
    //  Find the minimum value of the uncovered elements
    // ((maskMatrix == condition) ? (A) : (B)).minCoeff
    // ((!coveredMatrix) ? (workingMatrix) : (dummyCost)).minCoeff()
    // (dummyCost+1) since costFunctionMatrix may contain the dummyCost
    T minUncoveredCoeff = (!coveredMatrix.array()).select((workingMatrix.array()),
        (dummyCost + 1)).minCoeff();

    //  Subtract the minimum value from the uncovered elements
    // ((maskMatrix == condition) ? (A) : (B))
    // ((!coveredMatrix) ? (workingMatrix - minCoeff) : (workingMatrix))
    workingMatrix = (!coveredMatrix.array()).select((workingMatrix.array() - minUncoveredCoeff),
      workingMatrix.array());

    // Check the elements covered by more than one line (at the intersection of a vertical and
    // horizontal line)
    for (int row = 0; row < matrixSize; row++) {
      for (int col = 0; col < matrixSize; col++) {
        //  If row is covered && column is covered -> element is at intersection -> add the
        // minUncoveredCoeff
        if ((coveredMatrix.row(row).all()) && (coveredMatrix.col(col).all())) {
          //  Add the minimum value to the elements at the intersection of two lines
          workingMatrix(row, col) += minUncoveredCoeff;
        }
      }
    }
  }

  //  Step 5: Find optimal cost
  void FindOptimalCost()
  {
    //  Find the zeroes in the workingMatrix
    // ((maskMatrix == condition) ? (A) : (B))
    // (workingMatrix <= 0 ? (true) : (false))
    assignmentMatrix = (IsApproxZERO(workingMatrix.array())).select(true, assignmentMatrix.array());

    //  Check if direct assignment is possible
    //  Condition: Total number of assignments == number of elements to be assigned
    // (Example: Matrix contains 4 elements -> total of 4 assignments)
    if ((assignmentMatrix.count()) == matrixSize) {
      return;
    } else {
      //  This matrix will be reused to cover the assignments made in the assignmentMatrix
      coveredMatrix.fill(false);

      //  Loop until all elements in the matrix are checked
      while (!coveredMatrix.all()) {
        //  Check if a new assignment was made in this iteration
        bool newAssignmentMade = false;

        //  Check if a row contains a single zero
        for (int row = 0; row < matrixSize; row++) {
          //  Check only rows with uncovered elements
          if (!coveredMatrix.row(row).all()) {
            //  If a row contains a single zero -> Directly assign that element, exclude the row
            // and colummn from future checking
            if ((assignmentMatrix.row(row).count()) == 1) {
              //  A new assignment was made in this iteration
              newAssignmentMade = true;

              //  Find the element position
              int idxCol;
              assignmentMatrix.row(row).maxCoeff(&idxCol);

              //  Clear all other possible assignments in the corresponding column
              // (if any are present)
              assignmentMatrix.col(idxCol).fill(false);
              //  Assign the element
              assignmentMatrix(row, idxCol) = true;

              //  Exclude row and column from future checking/assignments
              coveredMatrix.row(row).fill(true);
              coveredMatrix.col(idxCol).fill(true);

              //  Check if the assignment is complete
              if (coveredMatrix.all()) {
                return;
              }
            }
          }
        }

        //  Check if a column contains a single zero
        for (int col = 0; col < matrixSize; col++) {
          //  Check only columns with uncovered elements
          if (!coveredMatrix.col(col).all()) {
            //  If a column contains a single zero -> Directly assign that element, exclude the
            // row and colummn from future checking
            if ((assignmentMatrix.col(col).count()) == 1) {
              //  A new assignment was made in this iteration
              newAssignmentMade = true;

              //  Find the element position
              int idxRow;
              assignmentMatrix.col(col).maxCoeff(&idxRow);

              //  Clear all other possible assignments in the corresponding column
              // (if any are present)
              assignmentMatrix.row(idxRow).fill(false);
              //  Assign the element
              assignmentMatrix(idxRow, col) = true;

              //  Exclude row and column from future checking/assignments
              coveredMatrix.row(idxRow).fill(true);
              coveredMatrix.col(col).fill(true);

              //  Check if the assignment is complete
              if (coveredMatrix.all()) {
                return;
              }
            }
          }
        }

        //  Check if no new assignments were made in this iteration
        if (!newAssignmentMade) {
          //  Since the assignment is not yet complete, this means that the assignment problem has
          // multiple possible solutions
          //  Choose a possible solution: (!(coveredMatrix.array()) && (assignmentMatrix.array())).
          // maxCoeff(&idxRow, &idxCol);

          //  Optimal solution for the problem at hand: Find the index of the minimum cost (in the
          // costFunctionMatrix) for possible candidates
          //  Possible candidate: !coveredMatrix && assignmentMatrix
          // ((maskMatrix == condition) ? (A) : (B)).minCoeff
          // ((!coveredMatrix && assignmentMatrix) ? (costFunctionMatrix) : (dummyCost+1)).minCoeff
          // (dummyCost+1) since costFunctionMatrix may contain the dummyCost
          int idxRow, idxCol;
          //  Check if there are any possible candidates left
          if (((!coveredMatrix.array()) && (assignmentMatrix.array())).any()) {
            //  Assign the optimal candidate
            ((!coveredMatrix.array()) && (assignmentMatrix.array()))
            .select((costFunctionMatrix.array()), (dummyCost + 1))
            .minCoeff(&idxRow, &idxCol);
          } else {
            //  Else -> No possible candidates left, check uncovered elements
            (!coveredMatrix.array()).select((costFunctionMatrix.array()),
              (dummyCost + 1)).minCoeff(&idxRow, &idxCol);
          }
          //  Clear all other possible assignments in the corresponding row and column
          assignmentMatrix.row(idxRow).fill(false);
          assignmentMatrix.col(idxCol).fill(false);
          //  Assign the element
          assignmentMatrix(idxRow, idxCol) = true;

          //  Exclude row and column from future checking/assignments
          coveredMatrix.row(idxRow).fill(true);
          coveredMatrix.col(idxCol).fill(true);

          //  Check if the assignment is complete
          if (coveredMatrix.all()) {
            return;
          }

          //  Check the matrix again (iterate)
        }
      }
    }
  }

public:
  //  Default object constructor, cost function matrix must be set later
  HungarianAlgorithm() = default;

  //  Create the Hungarian algorithm object using the costFunctionMatrix
  explicit HungarianAlgorithm(
    const Eigen::Matrix<T, Eigen::Dynamic,
    Eigen::Dynamic> & costFcnMatrix)
  {
    //  Set cost function matrix
    SetCostFunctionMatrix(costFcnMatrix);
    //  Update problemStatus
    problemStatus = ProblemStatus::ReadyToSolve;
  }
  //  Set the cost function matrix
  void SetCostFunctionMatrix(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> & costFcnMatrix)
  {
    //  Check if the input matrix contains any negative values
    if ((costFcnMatrix.array() < 0).any()) {
      throw std::invalid_argument("The cost function matrix cannot contain negative values!");
    }

    //  Set the dummy cost as a very large number
    dummyCost = (costFcnMatrix.maxCoeff() + 100);
    //  Save the matrix size
    nrRows = static_cast<int>(costFcnMatrix.rows());
    nrCols = static_cast<int>(costFcnMatrix.cols());
    //  Check if the costFunctionMatrix is not square
    if (nrRows != nrCols) {
      //  Get the size for the used matrices
      matrixSize = std::max(nrCols, nrRows);
      //  Initialize cost function matrix with the dummyCost
      costFunctionMatrix.resize(matrixSize, matrixSize);
      costFunctionMatrix.fill(dummyCost);
      //  Copy relevant data from the costFunctionMatrix
      costFunctionMatrix.block(0, 0, nrRows, nrCols) = costFcnMatrix;
    } else {
      //  Get the size for the used matrices
      matrixSize = nrRows;
      //  Initialize cost function matrix with input matrix
      costFunctionMatrix.resize(matrixSize, matrixSize);
      costFunctionMatrix = costFcnMatrix;
    }
    //  Copy the costFunctionMatrix contents to the workingMatrix
    workingMatrix = costFunctionMatrix;
    //  Initialize coveredMatrix with false
    coveredMatrix.resize(matrixSize, matrixSize);
    coveredMatrix.fill(false);
    //  Initialize assignmentMatrix with false
    assignmentMatrix.resize(matrixSize, matrixSize);
    assignmentMatrix.fill(false);
    //  Update problemStatus
    problemStatus = ProblemStatus::ReadyToSolve;
  }

  //  Get the cost function matrix
  void GetCostFunctionMatrix(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> & outMatrix)
  {
    if (problemStatus < ProblemStatus::ReadyToSolve) {
      throw std::invalid_argument("The cost function matrix is undefined!");
    }
    if ((outMatrix.rows() != nrRows) || (outMatrix.cols() != nrCols)) {
      throw std::invalid_argument(
        "The input matrix dimensions is inconsistent with the cost function matrix!");
    }
    //  Copy cost function matrix to the output
    outMatrix = costFunctionMatrix;
  }

  //  Get the assignment matrix after solving the problem
  void GetAssignmentMatrix(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> & outMatrix)
  {
    if (problemStatus < ProblemStatus::Done) {
      throw std::invalid_argument("The assignment problem has not been solved yet!");
    }
    if ((outMatrix.rows() != nrRows) || (outMatrix.cols() != nrCols)) {
      throw std::invalid_argument(
        "The input matrix dimensions is inconsistent with the assignment matrix!");
    }
    //  Copy assignment matrix to the output
    outMatrix = assignmentMatrix.block(0, 0, nrRows, nrCols).cast<int>();
  }

  //  Get the assignment indices in two vectors for easy access
  void GetAssignmentResults(std::vector<int> & rowIndices, std::vector<int> & colIndices)
  {
    if (problemStatus < ProblemStatus::Done) {
      throw std::invalid_argument("The assignment problem has not been solved yet!");
    }
    if (static_cast<Eigen::Index>(rowIndices.size()) != nrRows) {
      throw std::invalid_argument(
        "The input row vector size is inconsistent with the number of rows!");
    }
    if (static_cast<Eigen::Index>(colIndices.size()) != nrCols) {
      throw std::invalid_argument(
        "The input col vector size is inconsistent with the number of cols!");
    }
    //  Initialize output vector data with (-1)
    std::fill(rowIndices.begin(), rowIndices.end(), -1);
    std::fill(colIndices.begin(), colIndices.end(), -1);
    //  Loop on all elements in the assignmentMatrix
    for (Eigen::Index row = 0; row < nrRows; row++) {
      for (Eigen::Index col = 0; col < nrCols; col++) {
        if (assignmentMatrix(row, col)) {
          //  Save the valid index data
          rowIndices[row] = col;
          colIndices[col] = row;
        }
      }
    }
  }

  //  Get current problem status
  ProblemStatus getProblemStatus()
  {
    return problemStatus;
  }

  //  Get current problem status name
  std::string getProblemStatusName()
  {
    return ProblemStatusName[problemStatus];
  }

  //  Wrapper to execute all steps of the Hungarian algorithm and solve the assignment problem
  void SolveAssignmentProblem()
  {
    if (problemStatus < ProblemStatus::ReadyToSolve) {
      throw std::invalid_argument("The cost function matrix is undefined!");
    }
    //  Execute the Hungarian algorithm sequence
    if (nrRows >= nrCols) {
      //  Step 1
      SubtractRowMinima();
      //  Step 2
      SubtractColMinima();
    } else {
      //  Reverse the order
      //  Step 2
      SubtractColMinima();
      //  Step 1
      SubtractRowMinima();
    }
    //  Step 3
    while (MinNrOfLinesToCoverAllZeros() != matrixSize) {
      //  Step 4
      AugmentCostFunctionMatrix();
    }
    //  Step 5
    FindOptimalCost();
    //  Assignment is done
    problemStatus = ProblemStatus::Done;
  }
};

template class HungarianAlgorithm<int>;
template class HungarianAlgorithm<float>;
template class HungarianAlgorithm<double>;

#endif  //  UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__HUNGARIAN_ALGORITHM_HPP_
