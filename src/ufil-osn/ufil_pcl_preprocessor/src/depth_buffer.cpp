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

#include "ufil_pcl_preprocessor/depth_buffer.hpp"

#include <cmath>
#include <algorithm>
#include <numeric>

namespace ufil_pcl_preprocessor
{

// // ----------------------------
// // Configuration
// // ----------------------------
// void DepthBuffer::configure_angles(
//   double azimuth_min_rad,
//   double azimuth_max_rad,
//   double elevation_min_rad,
//   double elevation_max_rad,
//   double azimuth_resolution_rad,
//   double elevation_resolution_rad)
// {
//   azimuth_min_rad_ = azimuth_min_rad;
//   azimuth_max_rad_ = azimuth_max_rad;
//   elevation_min_rad_ = elevation_min_rad;
//   elevation_max_rad_ = elevation_max_rad;
//   azimuth_res_rad_ = azimuth_resolution_rad;
//   elevation_res_rad_ = elevation_resolution_rad;

//   // compute cell counts (floor: we want integer number of cells)
//   azimuth_cells_ = static_cast<int>(std::floor((azimuth_max_rad_ - azimuth_min_rad_) /
//       azimuth_res_rad_));
//   elevation_cells_ = static_cast<int>(std::floor((elevation_max_rad_ - elevation_min_rad_) /
//       elevation_res_rad_));

//   if (azimuth_cells_ <= 0) {azimuth_cells_ = 1;}
//   if (elevation_cells_ <= 0) {elevation_cells_ = 1;}

//   const int total_cells = azimuth_cells_ * elevation_cells_;
//   polar_cells_.assign(total_cells, PolarCell{});
//   occupancy_history_.assign(total_cells, std::deque<bool>{});
//   occupancy_count_.assign(total_cells, 0);
// }

//  /// Set min / max valid range for hits (meters).
// void DepthBuffer::setMinRange(double meters) {min_range_m_ = meters;}
// void DepthBuffer::setMaxRange(double meters) {max_range_m_ = meters;}

//   /// Grid output parameters (meters and resolution)
// void DepthBuffer::setGridResolution(double meters) {grid_resolution_m_ = meters;}
// void DepthBuffer::setGridSize(double size_x_m, double size_y_m)
// {
//   grid_size_x_m_ = size_x_m;
//   grid_size_y_m_ = size_y_m;
// }

// void DepthBuffer::setHistoryLength(size_t length)
// {
//   history_length_ = length > 0 ? length : 1;
// }


//   /// Rate at which previous occupancy decays each update (0..1).
// void DepthBuffer::setDecayRate(double decay) {decay_rate_ = decay;}

//   /// Set sensor position used for ray projection (x,y,z in meters).
// void DepthBuffer::setSensorOrigin(const Eigen::Vector3d & origin) {sensor_origin_ = origin;}

//   /// Set preprocessing thresholds (defaults preserved from prior code).
// void DepthBuffer::setPreprocessThresholds(
//   int threshold_low, int threshold_high,
//   int strong_neighbor_threshold, int fill_radius, int fill_min_count)
// {
//   preprocess_T_low_ = threshold_low;
//   preprocess_T_high_ = threshold_high;
//   preprocess_K_ = strong_neighbor_threshold;
//   preprocess_R_ = fill_radius;
//   preprocess_min_count_ = fill_min_count;
// }

// // ----------------------------
// // Public API
// // ----------------------------
// void DepthBuffer::update(const pcl::PointCloud<pcl::PointXYZ> & cloud)
// {
//   update_polar_buffer(cloud);
//   polar_buffer_to_grid();
// }

// void DepthBuffer::reset()
// {
//   std::fill(polar_cells_.begin(), polar_cells_.end(), PolarCell{});
//   std::fill(occupancy_count_.begin(), occupancy_count_.end(), 0);
//   for (auto & dq : occupancy_history_) {
//     dq.clear();
//   }

//   // reset occupancy grid to empty layout matching current params
//   OccupancyGrid empty_grid;
//   empty_grid.info.resolution = grid_resolution_m_;
//   empty_grid.info.width =
//                  static_cast<uint32_t>(std::floor(grid_size_x_m_ / grid_resolution_m_));
//   empty_grid.info.height =
//                  static_cast<uint32_t>(std::floor(grid_size_y_m_ / grid_resolution_m_));
//   empty_grid.info.origin.x = -grid_size_x_m_ * 0.5;
//   empty_grid.info.origin.y = -grid_size_y_m_ * 0.5;
//   empty_grid.info.origin.z = 0.15;
//   empty_grid.data.assign(empty_grid.info.width * empty_grid.info.height, 0);

//   occupancy_grid_ = std::move(empty_grid);
// }

// OccupancyGrid DepthBuffer::occupancyGrid() const
// {
//   return occupancy_grid_;
// }

// size_t DepthBuffer::historyLength() const {return history_length_;}
// double DepthBuffer::minRange() const {return min_range_m_;}
// double DepthBuffer::maxRange() const {return max_range_m_;}
// double DepthBuffer::gridResolution() const {return grid_resolution_m_;}

// // ----------------------------
// // Step 1: update polar buffer
// // ----------------------------
// void DepthBuffer::update_polar_buffer(const pcl::PointCloud<pcl::PointXYZ> & cloud)
// {
//   // Short names for speed
//   const double min_r = min_range_m_;
//   const double max_r = max_range_m_;

//   // 1) Record closest hit per polar cell
//   for (const auto & pt : cloud) {
//     const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

//     if (!std::isfinite(r) || r < min_r || r > max_r) {
//       continue;  // ignore invalid or out-of-range points
//     }

//     const double az = std::atan2(pt.y, pt.x);
//     const double el = std::asin(pt.z / r);

//     const int az_idx =
//              static_cast<int>(std::floor((az - azimuth_min_rad_) / azimuth_res_rad_));
//     const int el_idx =
//              static_cast<int>(std::floor((el - elevation_min_rad_) / elevation_res_rad_));


//     if (az_idx < 0 || az_idx >= azimuth_cells_ || el_idx < 0 || el_idx >= elevation_cells_) {
//       continue;  // outside configured angular range
//     }

//     const int idx = el_idx * azimuth_cells_ + az_idx;

//     PolarCell & cell = polar_cells_[idx];

//     if (r < cell.range) {
//       cell.range = r;
//       cell.hit_point = Eigen::Vector3d(pt.x, pt.y, pt.z);
//     }
//   }

//   // 2) Update temporal occupancy history
//   const int total = static_cast<int>(polar_cells_.size());
//   for (int i = 0; i < total; ++i) {
//     const bool occupied_now = std::isfinite(polar_cells_[i].range);
//     auto & dq = occupancy_history_[i];

//     dq.push_back(occupied_now);
//     occupancy_count_[i] += occupied_now ? 1 : 0;

//     if (dq.size() > history_length_) {
//       occupancy_count_[i] -= dq.front() ? 1 : 0;
//       dq.pop_front();
//     }
//   }
// }

// // ----------------------------
// // Step 2: build occupancy grid from polar buffer
// // ----------------------------
// void DepthBuffer::polar_buffer_to_grid()
// {
//   // Derived grid dimensions (integers)
//   const uint32_t x_cells =
//              static_cast<uint32_t>(std::floor(grid_size_x_m_ / grid_resolution_m_));
//   const uint32_t y_cells =
//              static_cast<uint32_t>(std::floor(grid_size_y_m_ / grid_resolution_m_));

//   OccupancyGrid grid;
//   grid.info.resolution = grid_resolution_m_;
//   grid.info.width = x_cells;
//   grid.info.height = y_cells;
//   grid.info.origin.x = -grid_size_x_m_ * 0.5;
//   grid.info.origin.y = -grid_size_y_m_ * 0.5;
//   grid.info.origin.z = 0.15;

//   // Start with decay applied to previous grid if sizes match
//   grid.data.assign(static_cast<size_t>(x_cells) * y_cells, 0);
//   if (occupancy_grid_.data.size() == grid.data.size()) {
//     for (size_t i = 0; i < grid.data.size(); ++i) {
//       grid.data[i] = static_cast<int8_t>(std::round(occupancy_grid_.data[i] * decay_rate_));
//     }
//   }

//   // Project each occupied polar cell into ground-plane grid cell
//   for (size_t i = 0; i < polar_cells_.size(); ++i) {
//     const auto & cell = polar_cells_[i];
//     if (!std::isfinite(cell.range)) {continue;}

//     const int hist_len = static_cast<int>(occupancy_history_[i].size());
//     const double confidence_frac = (hist_len >
//       0) ? static_cast<double>(occupancy_count_[i]) / hist_len : 0.0;
//     const int confidence = static_cast<int>(std::round(confidence_frac * 100.0));

//     // Compute ray from sensor to hit point
//     const Eigen::Vector3d v = cell.hit_point - sensor_origin_;
//     const double v_norm = v.norm();
//     if (v_norm <= 1e-9) {continue;}

//     const Eigen::Vector3d dir = v / v_norm;

//     // Only rays pointing downward can hit the ground plane z=0
//     if (dir.z() >= -1e-9) {continue;}

//     // Parameter t where ray intersects z=0:
//                          //sensor_origin_.z + t*dir.z = 0 -> t = -sensor_z/dir.z
//     const double t_ground = -sensor_origin_.z() / dir.z();
//     if (t_ground <= cell.range) {continue;}  // hit is closer than ground intersection

//     const Eigen::Vector3d ground_pt = sensor_origin_ + t_ground * dir;

//     const int gx = static_cast<int>(std::floor((ground_pt.x() - grid.info.origin.x) /
//         grid_resolution_m_));
//     const int gy = static_cast<int>(std::floor((ground_pt.y() - grid.info.origin.y) /
//         grid_resolution_m_));

//     if (gx >= 0 && gx < static_cast<int>(x_cells) && gy >= 0 && gy < static_cast<int>(y_cells)) {
//       grid.data[static_cast<size_t>(gy) * x_cells +
//         static_cast<size_t>(gx)] = static_cast<int8_t>(confidence);
//     }
//   }

//   // Post-process / smoothing / gap-filling
//   preprocess_grid(grid);

//   // Store last grid
//   occupancy_grid_ = std::move(grid);
// }

// // ----------------------------
// // Step 3: preprocess grid
// // ----------------------------
// void DepthBuffer::preprocess_grid(OccupancyGrid & g) const
// {
//   const int W = static_cast<int>(g.info.width);
//   const int H = static_cast<int>(g.info.height);
//   if (W <= 0 || H <= 0) {return;}

//   auto idx = [&](int x, int y) {return y * W + x;};

//   std::vector<int8_t> stage1 = g.data;

//   // 1) Remove low/confidence cells and weak isolated cells
//   for (int y = 0; y < H; ++y) {
//     for (int x = 0; x < W; ++x) {
//       const int v = static_cast<int>(g.data[idx(x, y)]);
//       if (v <= 0) {
//         stage1[idx(x, y)] = 0;
//         continue;
//       }

//       if (v < preprocess_T_low_) {
//         stage1[idx(x, y)] = 0;
//         continue;
//       }

//       if (v < preprocess_T_high_) {
//         int strong_neighbors = 0;
//         for (int dy = -preprocess_R_; dy <= preprocess_R_; ++dy) {
//           for (int dx = -preprocess_R_; dx <= preprocess_R_; ++dx) {
//             const int xx = x + dx;
//             const int yy = y + dy;
//             if (xx >= 0 && xx < W && yy >= 0 && yy < H) {
//               if (static_cast<int>(g.data[idx(xx, yy)]) >= preprocess_T_high_) {
//                 ++strong_neighbors;
//               }
//             }
//           }
//         }
//         if (strong_neighbors < preprocess_K_) {
//           stage1[idx(x, y)] = 0;
//         }
//       }
//     }
//   }

//   // 2) Gap filling: fill zeros if enough neighbors exist
//   std::vector<int8_t> stage2 = stage1;
//   for (int y = 0; y < H; ++y) {
//     for (int x = 0; x < W; ++x) {
//       if (stage1[idx(x, y)] > 0) {continue;}

//       int sum = 0;
//       int cnt = 0;
//       for (int dy = -preprocess_R_; dy <= preprocess_R_; ++dy) {
//         for (int dx = -preprocess_R_; dx <= preprocess_R_; ++dx) {
//           const int xx = x + dx;
//           const int yy = y + dy;
//           if (xx >= 0 && xx < W && yy >= 0 && yy < H) {
//             const int nv = static_cast<int>(stage1[idx(xx, yy)]);
//             if (nv > 0) {sum += nv; ++cnt;}
//           }
//         }
//       }
//       if (cnt >= preprocess_min_count_) {
//         const int avg = (sum + cnt / 2) / cnt;  // rounded average
//         stage2[idx(x, y)] = static_cast<int8_t>(std::clamp(avg, 0, 100));
//       }
//     }
//   }

//   g.data.swap(stage2);
// }

}  // namespace ufil_pcl_preprocessor
