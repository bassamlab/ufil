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

#ifndef UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__LP_SOLVER_HPP_
#define UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__LP_SOLVER_HPP_

#include <Eigen/Eigen>
#include <vector>
#include <limits>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <algorithm>

struct SimplexTableau
{
  // s is the matrix of size (numConstraints+1) x (numVariables+1)
  // The last row is the objective row. The last column is the RHS (b).
  std::vector<std::vector<double>> s;
  // basicVar[i] holds the index of the variable that is basic in row i.
  // nonBasicVar is optional for indexing, we won't use it heavily here.
  std::vector<int> basicVar;
};

template<typename T>
class LP_Solver
{
private:
  int m_;  // row constraints = #rows in cost matrix
  int n_;  // col constraints = #cols in cost matrix
  int totalVars_;  //  m_ * n_ real assignment variables
  int totalConstraints_;  // m_ + n_

  Eigen::Matrix<T, -1, -1> costMatrix_;     // For reference
  Eigen::Matrix<bool, -1, -1> assignment_;  // Final result

  SimplexTableau tableau_;

public:
  LP_Solver()
  : m_(0), n_(0), totalVars_(0), totalConstraints_(0) {}

  /**
   * @brief Construct with an initial cost matrix
   */
  explicit LP_Solver(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> & costFcnMatrix)
  {
    SetCostFunctionMatrix(costFcnMatrix);
  }

  /**
   * @brief Load the cost function matrix. For an assignment problem,
   *        we assume it's non-negative.
   */
  void SetCostFunctionMatrix(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> & originalCosts)
  {
    if ((originalCosts.array() < 0).any()) {
      throw std::invalid_argument("Negative values not allowed in cost function matrix!");
    }
    int M = static_cast<int>(originalCosts.rows());
    int N = static_cast<int>(originalCosts.cols());
    if (M == 0 || N == 0) {
      throw std::invalid_argument("Cost matrix must be non-empty.");
    }

    // 1) Pick S = max(M, N).
    int S = std::max(M, N);

    // 2) Create an S x S padded matrix.
    //    Decide what to use for “dummy” rows/columns; e.g. 0.0 or a big penalty.
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> costPadded =
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Constant(S, S, T(0));

    // 3) Copy the original MxN block.
    costPadded.block(0, 0, M, N) = originalCosts;

    // If you want to discourage assigning dummy rows/columns, set them to a large cost:
    // costPadded.block(0, N, M, S-N).setConstant(LARGE_VALUE);
    // costPadded.block(M, 0, S-M, N).setConstant(LARGE_VALUE);

    // Save these dimensions for the solver’s internal understanding
    m_ = S;  // the solver will treat it as S constraints for rows
    n_ = S;  // and S constraints for columns
    costMatrix_ = costPadded;

    totalVars_ = m_ * n_;
    totalConstraints_ = m_ + n_;

    assignment_.resize(m_, n_);
    assignment_.setZero();
  }

  /**
   * @brief Solve the assignment problem with a two-phase simplex approach.
   */
  void SolveAssignmentProblem()
  {
    buildPhase1Tableau();
    if (!runSimplex()) {
      throw std::runtime_error("Phase 1: Simplex did not converge (unbounded?).");
    }
    double sumArt = computeSumOfArtificial();
    if (sumArt > 1e-9) {
      throw std::runtime_error("Infeasible assignment problem (artificial vars not zero).");
    }

    // Pivot out any artificial variables from the basis if needed
    removeArtificialVarsFromBasis();

    // Setup Phase 2 objective
    buildPhase2Objective();
    if (!runSimplex()) {
      throw std::runtime_error("Phase 2: Simplex did not converge (unbounded?).");
    }

    // Reconstruct solution from final basis
    buildAssignmentMatrix();
  }

  /**
   * @brief Return the assignment as a 0/1 integer matrix
   */
  void GetAssignmentMatrix(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> & outMatrix)
  {
    outMatrix = assignment_.cast<int>();
  }

private:
  /**
   * @brief For equality constraints sum_j x_{ij} = 1 (for each row i),
   *        sum_i x_{ij} = 1 (for each column j),
   *        we add one artificial variable per constraint, so totalConstraints_ = m_ + n_.
   *        final # of variables in Phase 1 = totalVars_ + totalConstraints_ (the a_k).
   *        We also have one more row for the objective and one more column for the RHS.
   */
  void buildPhase1Tableau()
  {
    int numRows = totalConstraints_ + 1;          // +1 for objective row
    int numCols = totalVars_ + totalConstraints_ + 1;  // +1 for RHS

    tableau_.s.assign(numRows, std::vector<double>(numCols, 0.0));
    tableau_.basicVar.assign(totalConstraints_, -1);

    // Fill constraints:
    // Row constraints: sum_{j=0..n-1} x_{i,j} = 1, for i in [0..m-1]
    // => each row i of the tableau has x_{i,0..n-1}, plus an artificial variable, = 1
    // Actually we have m + n constraints, so the first m rows are row constraints,
    // next n rows are column constraints.

    // The layout for row constraints (i in [0..m-1]):
    // x_{i,0}, x_{i,1}, ..., x_{i,n-1}, x_{i+1,0}, ..., artificial var for row i, ...
    //   but we need the correct index for x_{i,j} in the flattened scheme:
    //   varIndex = i * n_ + j
    for (int i = 0; i < m_; i++) {
      int rowIndex = i;  // row constraints
      for (int j = 0; j < n_; j++) {
        int varIndex = i * n_ + j;  // flatten row-major
        tableau_.s[rowIndex][varIndex] = 1.0;
      }
      // Add artificial variable for row 'i', let's place it at index (totalVars_ + i)
      int artIndex = totalVars_ + i;
      tableau_.s[rowIndex][artIndex] = 1.0;

      // RHS = 1
      tableau_.s[rowIndex].back() = 1.0;
      // We'll make that artificial variable the basic variable for row i initially
      tableau_.basicVar[rowIndex] = artIndex;
    }

    // Column constraints: sum_{i=0..m-1} x_{i,j} = 1, for j in [0..n-1]
    // The next n rows:
    for (int j = 0; j < n_; j++) {
      int rowIndex = m_ + j;  // next row in the tableau
      for (int i = 0; i < m_; i++) {
        int varIndex = i * n_ + j;
        tableau_.s[rowIndex][varIndex] = 1.0;
      }
      // Add artificial variable for column constraint j
      int artIndex = totalVars_ + m_ + j;
      tableau_.s[rowIndex][artIndex] = 1.0;
      tableau_.s[rowIndex].back() = 1.0;
      tableau_.basicVar[rowIndex] = artIndex;
    }

    // Phase 1 objective: minimize sum of all artificial vars => sum_i a_i
    // => objective row = [Coeffs...] with +1 for each artificial variable.
    // We'll store the negative of that sum in the objective row
    // so that we can run a standard "maximize" approach, or we do "minimize" by
    // flipping signs. (Below let's treat it as: objective row tries to drive it to 0.)
    int objRow = totalConstraints_;
    for (int r = 0; r < totalConstraints_; r++) {
      // int artIndex = tableau_.basicVar[r];
      // We want to minimize sum(a_i). If we do standard simplex "maximize",
      // we can put negative coefficients for each a_i in the objective row,
      // or do a separate approach with "minimize." Let's do a standard approach:
      // We'll sum up all rows into the objective row so that artificial variables
      // are penalized. The typical approach is to subtract each row from the objective.
      for (int c = 0; c < numCols; c++) {
        tableau_.s[objRow][c] -= tableau_.s[r][c];
      }
    }
  }

  /**
   * @brief Standard simplex loop (naive version).
   *        This tries to "maximize" the objective row.
   *        Columns with negative objective coefficients can pivot in.
   */
  bool runSimplex()
  {
    const double eps = 1e-9;
    while (true) {
      // 1) Find pivot column = most negative (for Max) in the objective row
      int pivotCol = -1;
      double minVal = +1e12;
      int objRow = static_cast<int>(tableau_.s.size()) - 1;
      const int lastCol = static_cast<int>(tableau_.s[0].size()) - 1;  // RHS is last column
      for (int col = 0; col < lastCol; col++) {
        double val = tableau_.s[objRow][col];
        if (val < minVal) {
          minVal = val;
          pivotCol = col;
        }
      }
      if (minVal >= -eps) {
        // no negative => optimal
        return true;
      }

      // 2) Find pivot row by min ratio
      int pivotRow = -1;
      double minRatio = 1e12;
      for (int r = 0; r < objRow; r++) {
        double colVal = tableau_.s[r][pivotCol];
        if (colVal > eps) {
          double ratio = tableau_.s[r].back() / colVal;
          if (ratio < minRatio) {
            minRatio = ratio;
            pivotRow = r;
          }
        }
      }
      if (pivotRow < 0) {
        // unbounded
        return false;
      }

      // 3) Pivot on (pivotRow, pivotCol)
      pivot(pivotRow, pivotCol);
    }
  }

  /**
   * @brief Pivot operation on (pivotRow, pivotCol)
   */
  void pivot(int pivotRow, int pivotCol)
  {
    double pivotVal = tableau_.s[pivotRow][pivotCol];
    // Scale pivot row
    for (double & x : tableau_.s[pivotRow]) {
      x /= pivotVal;
    }
    // Eliminate above/below
    for (int r = 0; r < static_cast<int>(tableau_.s.size()); r++) {
      if (r == pivotRow) {continue;}
      double factor = tableau_.s[r][pivotCol];
      for (int c = 0; c < static_cast<int>(tableau_.s[r].size()); c++) {
        tableau_.s[r][c] -= factor * tableau_.s[pivotRow][c];
      }
    }
    // Update basic variable
    tableau_.basicVar[pivotRow] = pivotCol;
  }

  /**
   * @brief Sum of artificial variables: For row k, if basicVar[k] is an artificial var,
   *        then its RHS is s[k].back().
   */
  double computeSumOfArtificial() const
  {
    double sum = 0.0;
    int artStart = totalVars_;  // index of first artificial
    int artEnd = totalVars_ + totalConstraints_;
    for (int r = 0; r < totalConstraints_; r++) {
      int bv = tableau_.basicVar[r];
      if (bv >= artStart && bv < artEnd) {
        sum += tableau_.s[r].back();
      }
    }
    return sum;
  }

  /**
   * @brief We want to remove artificial variables from the basis if any remain.
   *        They have to be pivoted out so that the feasible solution doesn't rely on them.
   */
  void removeArtificialVarsFromBasis()
  {
    // Pivot out any artificial var that is in the basis with > 1e-9 value
    int artStart = totalVars_;
    int artEnd = totalVars_ + totalConstraints_;
    // int objRow = totalConstraints_;
    for (int r = 0; r < totalConstraints_; r++) {
      int bv = tableau_.basicVar[r];
      if (bv >= artStart && bv < artEnd) {
        // Try to find a pivot column with a non-zero row coefficient
        // for a real variable. We'll do a simple approach:
        for (int col = 0; col < totalVars_; col++) {
          double val = tableau_.s[r][col];
          if (std::fabs(val) > 1e-9) {
            // pivot here
            pivot(r, col);
            break;
          }
        }
      }
    }
  }

  /**
   * @brief Build the actual (Phase 2) objective row from the cost matrix.
   *        For a minimization problem: we can do "maximize -cost" or "minimize cost".
   *        We'll do "maximize" of negative cost => objective row = sum of cost * x_{i,j}
   */
  void buildPhase2Objective()
  {
    int objRow = totalConstraints_;
    // Reset objective row to zero
    for (size_t c = 0; c < tableau_.s[objRow].size(); c++) {
      tableau_.s[objRow][c] = 0.0;
    }
    // We want to "maximize" the negative of the cost => coefficient = - cost(i,j)
    // For each x_{i,j}, that is varIndex = i*n_ + j
    for (int i = 0; i < m_; i++) {
      for (int j = 0; j < n_; j++) {
        int varIdx = i * n_ + j;
        tableau_.s[objRow][varIdx] = -static_cast<double>(costMatrix_(i, j));
      }
    }
    // Next, we must account for the fact that the basis variables have some
    // effect on the objective if any x_{i,j} is in the basis with a non-zero solution.
    // So we'll do the standard "objective row = objective row - sum_{r} of ( objRowPivot * row )"
    // We'll subtract each row's multiple from the objective row.
    for (int r = 0; r < totalConstraints_; r++) {
      int bv = tableau_.basicVar[r];
      // If it's a real var, then we do: objectiveRow -= objectiveCoeff(bv) * row.
      // Our objectiveCoeff(bv) is what we put in the objective row for var bv.
      double coeff = tableau_.s[objRow][bv];
      if (std::fabs(coeff) > 1e-9) {
        for (size_t c = 0; c < tableau_.s[objRow].size(); c++) {
          tableau_.s[objRow][c] -= coeff * tableau_.s[r][c];
        }
      }
    }
  }

  /**
   * @brief After Phase 2, we interpret each row in the basis:
   *        If basicVar[r] < m_*n_, that means it's x_{i,j} for i = varIdx/n_, j = varIdx%n_.
   *        If the RHS is ~1, we mark assignment_(i,j)=true.
   */
  void buildAssignmentMatrix()
  {
    assignment_.setZero();
    for (int r = 0; r < totalConstraints_; r++) {
      int varIdx = tableau_.basicVar[r];
      if (varIdx < totalVars_) {
        double val = tableau_.s[r].back();  // RHS
        if (std::fabs(val - 1.0) < 1e-6) {
          int i = varIdx / n_;
          int j = varIdx % n_;
          assignment_(i, j) = true;
        }
      }
    }

    // Debug
    // std::cerr << "Final (Phase 2) tableau:\n";
    for (size_t rr = 0; rr < tableau_.s.size(); rr++) {
      for (size_t cc = 0; cc < tableau_.s[rr].size(); cc++) {
        // std::cerr << tableau_.s[rr][cc] << " ";
      }
      // std::cerr << "\n";
    }
    // std::cerr << "Basic vars: ";
    // for (auto bv : tableau_.basicVar) {std::cerr << bv << " ";}
    // std::cerr << "\n";
    // std::cerr << "Assignment:\n" << assignment_.cast<int>() << std::endl;
  }
};

// Explicit template instantiations
template class LP_Solver<int>;
template class LP_Solver<float>;
template class LP_Solver<double>;

#endif  // UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__LP_SOLVER_HPP_
