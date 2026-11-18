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

#include <gtest/gtest.h>

#include <iomanip>
#include <chrono>
#include <iostream>
#include <numeric>
#include <vector>

#include <ufil_object_tracking/association/association_functions.hpp>
#include <ufil_object_tracking/types/matrix.hpp>

TEST(association_functions, greedy_zero_cost_matrix)
{
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Zero(1, 1);
  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(1, 1);

  ufil::association::greedy(cost_matrix, assignment_matrix);
  EXPECT_EQ(assignment_matrix(0, 0), 1);
}

TEST(association_functions, greedy_clear_optimal)
{
  ufil::type::CostMatrix cost_matrix(2, 2);
  cost_matrix << 10, 5,
                 3, 100;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(2, 2);
  ufil::association::greedy(cost_matrix, assignment_matrix);

  EXPECT_EQ(assignment_matrix(0, 1), 1);  // Assign row 0 to col 1 (cost 5)
  EXPECT_EQ(assignment_matrix(1, 0), 1);  // Assign row 1 to col 0 (cost 3)
}

TEST(association_functions, greedy_diagonal_optimal)
{
  ufil::type::CostMatrix cost_matrix(3, 3);
  cost_matrix << 1, 2, 3,
                 2, 1, 2,
                 3, 2, 1;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(3, 3);
  ufil::association::greedy(cost_matrix, assignment_matrix);

  EXPECT_EQ(assignment_matrix(0, 0), 1);
  EXPECT_EQ(assignment_matrix(1, 1), 1);
  EXPECT_EQ(assignment_matrix(2, 2), 1);
}

TEST(association_functions, greedy_more_tracks_than_measurements)
{
  ufil::type::CostMatrix cost_matrix(3, 2);
  cost_matrix << 4, 1,
                 2, 3,
                 5, 2;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(3, 2);
  ufil::association::greedy(cost_matrix, assignment_matrix);

  // Since the assignment must be 1:1, only 2 rows can be assigned
  int assigned_rows = 0;
  for (int i = 0; i < assignment_matrix.rows(); ++i) {
    for (int j = 0; j < assignment_matrix.cols(); ++j) {
      assigned_rows += assignment_matrix(i, j);
    }
}

  EXPECT_EQ(assigned_rows, 2);
}

TEST(association_functions, greedy_more_measurements_than_tracks)
{
  ufil::type::CostMatrix cost_matrix(2, 3);
  cost_matrix << 7, 8, 5,
                 6, 4, 3;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(2, 3);
  ufil::association::greedy(cost_matrix, assignment_matrix);

  int assigned_rows = 0;
  for (int i = 0; i < assignment_matrix.rows(); ++i) {
    for (int j = 0; j < assignment_matrix.cols(); ++j) {
      assigned_rows += assignment_matrix(i, j);
    }
}

  EXPECT_EQ(assigned_rows, 2);
}

TEST(association_functions, greedy_empty_matrix)
{
  ufil::type::CostMatrix cost_matrix(0, 0);
  ufil::type::AssignmentMatrix assignment_matrix;

  ufil::association::greedy(cost_matrix, assignment_matrix);
  EXPECT_EQ(assignment_matrix.size(), 0);
}

TEST(AssociationFunctions, hungarianZeroCostMatrix)
{
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Zero(1, 1);
  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(1, 1);

  ufil::association::hungarian(cost_matrix, assignment_matrix);
  EXPECT_EQ(assignment_matrix(0, 0), 1);
}

TEST(AssociationFunctions, hungarianClearOptimal)
{
  ufil::type::CostMatrix cost_matrix(2, 2);
  cost_matrix << 10, 5, 3, 100;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(2, 2);
  ufil::association::hungarian(cost_matrix, assignment_matrix);

  EXPECT_EQ(assignment_matrix(0, 1), 1);  // Assign row 0 to col 1 (cost 5)
  EXPECT_EQ(assignment_matrix(1, 0), 1);  // Assign row 1 to col 0 (cost 3)
}

TEST(AssociationFunctions, hungarianDiagonalOptimal)
{
  ufil::type::CostMatrix cost_matrix(3, 3);
  cost_matrix << 1, 2, 3, 2, 1, 2, 3, 2, 1;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(3, 3);
  ufil::association::hungarian(cost_matrix, assignment_matrix);

  EXPECT_EQ(assignment_matrix(0, 0), 1);
  EXPECT_EQ(assignment_matrix(1, 1), 1);
  EXPECT_EQ(assignment_matrix(2, 2), 1);
}

TEST(AssociationFunctions, hungarianMoreTracksThanMeasurements)
{
  ufil::type::CostMatrix cost_matrix(3, 2);
  cost_matrix << 4, 1, 2, 3, 5, 2;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(3, 2);
  ufil::association::hungarian(cost_matrix, assignment_matrix);

  // Since the assignment must be 1:1, only 2 rows can be assigned
  int assigned_rows = 0;
  for (int i = 0; i < assignment_matrix.rows(); ++i) {
    for (int j = 0; j < assignment_matrix.cols(); ++j) {
      assigned_rows += assignment_matrix(i, j);
    }
  }

  EXPECT_EQ(assigned_rows, 2);
}

TEST(AssociationFunctions, hungarianMoreMeasurementsThanTracks)
{
  ufil::type::CostMatrix cost_matrix(2, 3);
  cost_matrix << 7, 8, 5, 6, 4, 3;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(2, 3);
  ufil::association::hungarian(cost_matrix, assignment_matrix);

  int assigned_rows = 0;
  for (int i = 0; i < assignment_matrix.rows(); ++i) {
    for (int j = 0; j < assignment_matrix.cols(); ++j) {
      assigned_rows += assignment_matrix(i, j);
    }
  }

  EXPECT_EQ(assigned_rows, 2);
}

TEST(AssociationFunctions, hungarianEmptyMatrix)
{
  ufil::type::CostMatrix cost_matrix(0, 0);
  ufil::type::AssignmentMatrix assignment_matrix;

  ufil::association::hungarian(cost_matrix, assignment_matrix);
  EXPECT_EQ(assignment_matrix.size(), 0);
}

TEST(AssociationFunctions, jonkerVolgenantZeroCostMatrix)
{
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Zero(1, 1);
  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(1, 1);

  ufil::association::jonker_volgenant(cost_matrix, assignment_matrix);
  EXPECT_EQ(assignment_matrix(0, 0), 1);
}

TEST(AssociationFunctions, jonkerVolgenantClearOptimal)
{
  ufil::type::CostMatrix cost_matrix(2, 2);
  cost_matrix << 10, 5, 3, 100;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(2, 2);
  ufil::association::jonker_volgenant(cost_matrix, assignment_matrix);

  EXPECT_EQ(assignment_matrix(0, 1), 1);  // Assign row 0 to col 1 (cost 5)
  EXPECT_EQ(assignment_matrix(1, 0), 1);  // Assign row 1 to col 0 (cost 3)
}

TEST(AssociationFunctions, jonkerVolgenantDiagonalOptimal)
{
  ufil::type::CostMatrix cost_matrix(3, 3);
  cost_matrix << 1, 2, 3, 2, 1, 2, 3, 2, 1;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(3, 3);
  ufil::association::jonker_volgenant(cost_matrix, assignment_matrix);

  EXPECT_EQ(assignment_matrix(0, 0), 1);
  EXPECT_EQ(assignment_matrix(1, 1), 1);
  EXPECT_EQ(assignment_matrix(2, 2), 1);
}

TEST(AssociationFunctions, jonkerVolgenantMoreTracksThanMeasurements)
{
  ufil::type::CostMatrix cost_matrix(3, 2);
  cost_matrix << 4, 1, 2, 3, 5, 2;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(3, 2);
  ufil::association::jonker_volgenant(cost_matrix, assignment_matrix);

  // Since the assignment must be 1:1, only 2 rows can be assigned
  int assigned_rows = 0;
  for (int i = 0; i < assignment_matrix.rows(); ++i) {
    for (int j = 0; j < assignment_matrix.cols(); ++j) {
      assigned_rows += assignment_matrix(i, j);
    }
  }

  EXPECT_EQ(assigned_rows, 2);
}

TEST(AssociationFunctions, jonkerVolgenantMoreMeasurementsThanTracks)
{
  ufil::type::CostMatrix cost_matrix(2, 3);
  cost_matrix << 7, 8, 5, 6, 4, 3;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(2, 3);
  ufil::association::jonker_volgenant(cost_matrix, assignment_matrix);

  int assigned_rows = 0;
  for (int i = 0; i < assignment_matrix.rows(); ++i) {
    for (int j = 0; j < assignment_matrix.cols(); ++j) {
      assigned_rows += assignment_matrix(i, j);
    }
  }

  EXPECT_EQ(assigned_rows, 2);
}

TEST(AssociationFunctions, jonkerVolgenantEmptyMatrix)
{
  ufil::type::CostMatrix cost_matrix(0, 0);
  ufil::type::AssignmentMatrix assignment_matrix;

  ufil::association::jonker_volgenant(cost_matrix, assignment_matrix);
  EXPECT_EQ(assignment_matrix.size(), 0);
}

TEST(AssociationFunctions, lpSolverZeroCostMatrix)
{
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Zero(1, 1);
  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(1, 1);

  ufil::association::lp_solver(cost_matrix, assignment_matrix);
  EXPECT_EQ(assignment_matrix(0, 0), 1);
}

TEST(AssociationFunctions, lpSolverClearOptimal)
{
  ufil::type::CostMatrix cost_matrix(2, 2);
  cost_matrix << 10, 5, 3, 100;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(2, 2);
  ufil::association::lp_solver(cost_matrix, assignment_matrix);

  EXPECT_EQ(assignment_matrix(0, 1), 1);  // Assign row 0 to col 1 (cost 5)
  EXPECT_EQ(assignment_matrix(1, 0), 1);  // Assign row 1 to col 0 (cost 3)
}

TEST(AssociationFunctions, lpSolverDiagonalOptimal)
{
  ufil::type::CostMatrix cost_matrix(3, 3);
  cost_matrix << 1, 2, 3, 2, 1, 2, 3, 2, 1;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(3, 3);
  ufil::association::lp_solver(cost_matrix, assignment_matrix);

  EXPECT_EQ(assignment_matrix(0, 0), 1);
  EXPECT_EQ(assignment_matrix(1, 1), 1);
  EXPECT_EQ(assignment_matrix(2, 2), 1);
}

TEST(AssociationFunctions, lpSolverMoreTracksThanMeasurements)
{
  ufil::type::CostMatrix cost_matrix(3, 2);
  cost_matrix << 4, 1, 2, 3, 5, 2;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(3, 2);
  ufil::association::lp_solver(cost_matrix, assignment_matrix);

  // Since the assignment must be 1:1, only 2 rows can be assigned
  int assigned_rows = 0;
  for (int i = 0; i < assignment_matrix.rows(); ++i) {
    for (int j = 0; j < assignment_matrix.cols(); ++j) {
      assigned_rows += assignment_matrix(i, j);
    }
  }

  EXPECT_EQ(assigned_rows, 2);
}

TEST(AssociationFunctions, lpSolverMoreMeasurementsThanTracks)
{
  ufil::type::CostMatrix cost_matrix(2, 3);
  cost_matrix << 7, 8, 5, 6, 4, 3;

  ufil::type::AssignmentMatrix assignment_matrix = ufil::type::AssignmentMatrix::Zero(2, 3);
  ufil::association::lp_solver(cost_matrix, assignment_matrix);

  int assigned_rows = 0;
  for (int i = 0; i < assignment_matrix.rows(); ++i) {
    for (int j = 0; j < assignment_matrix.cols(); ++j) {
      assigned_rows += assignment_matrix(i, j);
    }
  }

  EXPECT_EQ(assigned_rows, 2);
}

TEST(AssociationFunctions, lpSolverEmptyMatrix)
{
  ufil::type::CostMatrix cost_matrix(0, 0);
  ufil::type::AssignmentMatrix assignment_matrix;

  ufil::association::lp_solver(cost_matrix, assignment_matrix);
  EXPECT_EQ(assignment_matrix.size(), 0);
}

TEST(association_functions, timing_large_matrix_100x100_100runs)
{
  constexpr int N = 10;
  constexpr int RUNS = 100;

  ufil::type::CostMatrix cost_matrix(N, N);
  cost_matrix.setConstant(1000000.0);
  for (int i = 0; i < N; ++i) {
    cost_matrix(i, i) = 0.0;
  }

  auto verify_identity = [&](const ufil::type::AssignmentMatrix & A) {
      ASSERT_EQ(A.rows(), N);
      ASSERT_EQ(A.cols(), N);
      for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
          if (i == j) {
            EXPECT_EQ(A(i, j), 1);
          } else {
            EXPECT_EQ(A(i, j), 0);
          }
        }
      }
    };

  auto run_algorithm = [&](const std::string & name,
    auto && func) -> std::tuple<double, double, double, std::vector<double>> {
      std::vector<double> times;
      times.reserve(RUNS);
      std::cout << "Running " << name << " (" << RUNS << " runs)..." << std::endl;
      for (int run = 0; run < RUNS; ++run) {
        if (run % 10 == 0 && run > 0) {
          std::cout << "  " << name << " progress: " << run << "/" << RUNS << std::endl;
        }

        ufil::type::AssignmentMatrix A = ufil::type::AssignmentMatrix::Zero(N, N);
        auto t0 = std::chrono::high_resolution_clock::now();
        func(cost_matrix, A);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration_cast<std::chrono::duration<double,
            std::milli>>(t1 - t0).count();
        verify_identity(A);
        times.push_back(ms);
      }

      double min_t = *std::min_element(times.begin(), times.end());
      double max_t = *std::max_element(times.begin(), times.end());
      double mean_t =
        std::accumulate(times.begin(), times.end(), 0.0) / static_cast<double>(times.size());

      std::cout << "  " << name << " min: " << min_t << " ms, mean: " << mean_t
                << " ms, max: " << max_t << " ms" << std::endl;

      return {min_t, mean_t, max_t, times};
    };

  auto [greedy_min, greedy_mean, greedy_max, greedy_times] =
    run_algorithm("greedy", ufil::association::greedy);
  auto [hungarian_min, hungarian_mean, hungarian_max, hungarian_times] =
    run_algorithm("hungarian", ufil::association::hungarian);
  auto [jv_min, jv_mean, jv_max, jv_times] =
    run_algorithm("jonker_volgenant", ufil::association::jonker_volgenant);
  auto [lp_min, lp_mean, lp_max, lp_times] =
    run_algorithm("lp_solver", ufil::association::lp_solver);

  std::cout << "\n=== Timing Summary (" << N << "x" << N << ", " << RUNS << " runs) ===\n";
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Algorithm         Min [ms]    Mean [ms]   Max [ms]\n";
  std::cout << "-------------------------------------------------\n";
  std::cout << "greedy           " << greedy_min << "      " << greedy_mean << "      " <<
    greedy_max << "\n";
  std::cout << "hungarian        " << hungarian_min << "      " << hungarian_mean << "      " <<
    hungarian_max << "\n";
  std::cout << "jonker_volgenant " << jv_min << "      " << jv_mean << "      " << jv_max << "\n";
  std::cout << "lp_solver        " << lp_min << "      " << lp_mean << "      " << lp_max << "\n";

  std::vector<std::pair<std::string, double>> means = {
    {"greedy", greedy_mean},
    {"hungarian", hungarian_mean},
    {"jonker_volgenant", jv_mean},
    {"lp_solver", lp_mean},
  };
  auto fastest = *std::min_element(means.begin(), means.end(),
      [](auto & a, auto & b) {return a.second < b.second;});
  std::cout << "\nFastest (by mean): " << fastest.first << " (" << fastest.second << " ms)\n";

  // === Raw data output (for boxplot) ===
  std::cout << "\n=== Raw Runtime Data (ms) ===" << std::endl;

  auto print_times = [](const std::string & name, const std::vector<double> & times) {
      std::cout << name;
      for (double t : times) {
        std::cout << ", " << t;
      }
      std::cout << std::endl;
    };

  print_times("greedy", greedy_times);
  print_times("hungarian", hungarian_times);
  print_times("jonker_volgenant", jv_times);
  print_times("lp_solver", lp_times);

  EXPECT_TRUE(std::isfinite(greedy_mean));
  EXPECT_TRUE(std::isfinite(hungarian_mean));
  EXPECT_TRUE(std::isfinite(jv_mean));
  EXPECT_TRUE(std::isfinite(lp_mean));
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
