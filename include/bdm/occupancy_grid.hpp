#pragma once

#include "bdm/boundary_map.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace bdm {

// Parameters for the first-version dense grid. This is not the paper's
// Section 7 log-odds sensor model.
struct OccupancyGridParams {
  double resolution_m = 0.5;
  double min_range_m = 1.0;
  double max_range_m = 30.0;
};

struct IntegrateStats {
  std::size_t points_in = 0;
  std::size_t points_integrated = 0;
  std::size_t points_rejected_range = 0;
  std::size_t points_rejected_nonfinite = 0;
};

// Axis-aligned bounds of sensor origins and in-range hits, in world meters.
struct WorldBounds {
  bool any = false;
  double min_x = 0;
  double min_y = 0;
  double min_z = 0;
  double max_x = 0;
  double max_y = 0;
  double max_z = 0;
};

// Sparse Free/Occupied grid. A missing key is Unknown.
//
// Hit policy: the voxel that contains an in-range return is Occupied.
// Miss policy: every other voxel the ray crosses from the sensor origin to
// that return is Free, unless it is already Occupied.
// Occupied is sticky: a later miss does not clear it.
// Returns closer than min_range_m or farther than max_range_m are ignored.
// There is no log-odds clamp, no ray past max range, and no sliding window.
class OccupancyGrid {
 public:
  explicit OccupancyGrid(OccupancyGridParams params);

  const OccupancyGridParams& params() const noexcept { return params_; }
  const VoxelGrid& voxels() const noexcept { return voxels_; }
  const WorldBounds& bounds() const noexcept { return bounds_; }
  std::size_t count(State state) const;

  // points_sensor are xyz in the sensor frame. world_from_sensor is row-major 3x4.
  IntegrateStats integrate(const std::vector<std::array<float, 3>>& points_sensor,
                           const std::array<double, 12>& world_from_sensor);

 private:
  Voxel voxelAt(double x, double y, double z) const;
  void includeBound(double x, double y, double z);
  void markFree(Voxel voxel);
  void markOccupied(Voxel voxel);
  void trace(const std::array<double, 3>& origin, const std::array<double, 3>& hit);

  OccupancyGridParams params_;
  VoxelGrid voxels_;
  WorldBounds bounds_;
};

}  // namespace bdm
