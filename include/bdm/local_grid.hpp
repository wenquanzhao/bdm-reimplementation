#pragma once

#include "bdm/boundary_map.hpp"
#include "bdm/log_odds.hpp"

#include <array>
#include <cstddef>
#include <unordered_set>
#include <vector>

namespace bdm {

struct LocalIntegrateStats {
  std::size_t points_in = 0;
  std::size_t points_integrated = 0;
  std::size_t points_rejected_range = 0;
  std::size_t points_rejected_nonfinite = 0;
  // Endpoint voxel was outside the current window. Misses that still crossed
  // the window were applied. The endpoint itself was not stored.
  std::size_t points_endpoint_outside = 0;
};

class IncrementalMapper;

// Fixed-size log-odds grid. Section 7.1's local map, as a skeleton.
//
// Storage is one array addressed by voxel index modulo the window size, the
// same idea as ROG-Map's rolling index: a voxel that stays inside the window
// keeps its slot when the origin moves. Slots that the slide reuses belong
// to the slide-in region and must be cleared by the caller before new values
// are written. This class does not slide itself. IncrementalMapper does.
//
// A missing geometric cell is not a hash miss. Every cell in the window has a
// value. The prior is 0, which equation (10) reads as unknown.
//
// integrate() adds l_hit or l_miss once per ray and clamps after that update
// (equations (7)–(9) with n_hit, n_miss in {0, 1}). It does not sum a whole
// scan and clamp once at the end. Voxels outside the window are left unchanged.
class LocalGrid {
 public:
  LocalGrid(double resolution_m, double min_range_m, double max_range_m, int size_x, int size_y,
            int size_z, LogOddsModel model);

  const LogOddsModel& model() const noexcept { return model_; }
  double resolution() const noexcept { return resolution_; }
  int sizeX() const noexcept { return size_x_; }
  int sizeY() const noexcept { return size_y_; }
  int sizeZ() const noexcept { return size_z_; }
  bool placed() const noexcept { return placed_; }
  Voxel origin() const noexcept { return origin_; }
  bool contains(Voxel voxel) const noexcept;
  Voxel voxelAt(double x, double y, double z) const;

  // Geometric center of the current window, in meters. Valid after place().
  std::array<double, 3> centerMeters() const;

  double logOdds(Voxel voxel) const;
  State occupancy(Voxel voxel) const;

  // points_sensor are xyz in the sensor frame. world_from_sensor is row-major 3x4.
  // Voxels whose discrete state changes are inserted into changed.
  LocalIntegrateStats integrate(const std::vector<std::array<float, 3>>& points_sensor,
                                const std::array<double, 12>& world_from_sensor,
                                std::unordered_set<Voxel, VoxelHash>& changed);

 private:
  friend class IncrementalMapper;

  // Moves the window without touching stored odds. The caller clears slide-in slots.
  void place(Voxel origin);
  void setLogOdds(Voxel voxel, double value);
  // Adds delta, clamps, and records a discrete-state change when changed != nullptr.
  void accumulate(Voxel voxel, double delta, std::unordered_set<Voxel, VoxelHash>* changed);

  std::size_t slot(Voxel voxel) const;
  void checkWindowFits(Voxel origin) const;

  double resolution_ = 1;
  double min_range_m_ = 0;
  double max_range_m_ = 1;
  int size_x_ = 1;
  int size_y_ = 1;
  int size_z_ = 1;
  LogOddsModel model_;
  Voxel origin_{};
  bool placed_ = false;
  std::vector<double> odds_;
};

}  // namespace bdm
