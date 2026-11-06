#include "bdm/occupancy_grid.hpp"

#include "bdm/rigid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace bdm {
namespace {

void includePoint(WorldBounds& bounds, double x, double y, double z) {
  if (!bounds.any) {
    bounds.any = true;
    bounds.min_x = bounds.max_x = x;
    bounds.min_y = bounds.max_y = y;
    bounds.min_z = bounds.max_z = z;
    return;
  }
  bounds.min_x = std::min(bounds.min_x, x);
  bounds.min_y = std::min(bounds.min_y, y);
  bounds.min_z = std::min(bounds.min_z, z);
  bounds.max_x = std::max(bounds.max_x, x);
  bounds.max_y = std::max(bounds.max_y, y);
  bounds.max_z = std::max(bounds.max_z, z);
}

}  // namespace

OccupancyGrid::OccupancyGrid(OccupancyGridParams params) : params_(params) {
  if (!(params_.resolution_m > 0.0) || !std::isfinite(params_.resolution_m)) {
    throw std::invalid_argument("resolution_m must be a finite number greater than 0");
  }
  if (!(params_.min_range_m >= 0.0) || !std::isfinite(params_.min_range_m)) {
    throw std::invalid_argument("min_range_m must be a finite number >= 0");
  }
  if (!(params_.max_range_m >= params_.min_range_m) || !std::isfinite(params_.max_range_m)) {
    throw std::invalid_argument("max_range_m must be finite and >= min_range_m");
  }
}

std::size_t OccupancyGrid::count(State state) const {
  std::size_t n = 0;
  for (const auto& [voxel, stored] : voxels_) {
    (void)voxel;
    if (stored == state) ++n;
  }
  return n;
}

Voxel OccupancyGrid::voxelAt(double x, double y, double z) const {
  const double inv = 1.0 / params_.resolution_m;
  const double scaled[3] = {x * inv, y * inv, z * inv};
  const double lo = static_cast<double>(std::numeric_limits<int>::min());
  const double hi = static_cast<double>(std::numeric_limits<int>::max());
  int index[3];
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(scaled[axis]) || scaled[axis] < lo || scaled[axis] > hi) {
      throw std::runtime_error("voxel index is outside the int range");
    }
    index[axis] = static_cast<int>(std::floor(scaled[axis]));
  }
  return {index[0], index[1], index[2]};
}

void OccupancyGrid::includeBound(double x, double y, double z) {
  includePoint(bounds_, x, y, z);
}

void OccupancyGrid::markFree(Voxel voxel) {
  const auto it = voxels_.find(voxel);
  if (it == voxels_.end()) {
    voxels_.emplace(voxel, State::Free);
    return;
  }
  // Occupied stays occupied. A miss never clears a hit.
  if (it->second == State::Occupied) return;
  it->second = State::Free;
}

void OccupancyGrid::markOccupied(Voxel voxel) { voxels_[voxel] = State::Occupied; }

void OccupancyGrid::trace(const std::array<double, 3>& origin, const std::array<double, 3>& hit) {
  const Voxel start = voxelAt(origin[0], origin[1], origin[2]);
  const Voxel end = voxelAt(hit[0], hit[1], hit[2]);
  const double dir[3] = {hit[0] - origin[0], hit[1] - origin[1], hit[2] - origin[2]};
  const double length_sq = dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2];
  if (length_sq < 1e-18 || (start.x == end.x && start.y == end.y && start.z == end.z)) {
    markOccupied(end);
    return;
  }

  // Amanatides & Woo. t is 0 at the sensor and 1 at the return.
  // Equal face-crossing times step x, then y, then z. A ray that only
  // touches a corner may skip one of the two face-adjacent cells.
  const double res = params_.resolution_m;
  const double inf = std::numeric_limits<double>::infinity();
  int step[3];
  double t_max[3];
  double t_delta[3];
  const int start_index[3] = {start.x, start.y, start.z};
  for (int axis = 0; axis < 3; ++axis) {
    if (dir[axis] > 0.0) {
      step[axis] = 1;
      const double boundary = (static_cast<double>(start_index[axis]) + 1.0) * res;
      t_max[axis] = (boundary - origin[axis]) / dir[axis];
      t_delta[axis] = res / dir[axis];
    } else if (dir[axis] < 0.0) {
      step[axis] = -1;
      const double boundary = static_cast<double>(start_index[axis]) * res;
      t_max[axis] = (boundary - origin[axis]) / dir[axis];
      t_delta[axis] = -res / dir[axis];
    } else {
      step[axis] = 0;
      t_max[axis] = inf;
      t_delta[axis] = inf;
    }
  }

  int x = start.x;
  int y = start.y;
  int z = start.z;
  const int span = std::abs(end.x - start.x) + std::abs(end.y - start.y) +
                   std::abs(end.z - start.z);
  const int max_steps = span + 3;
  int steps = 0;
  while (steps < max_steps && !(x == end.x && y == end.y && z == end.z)) {
    markFree({x, y, z});
    int axis = 0;
    if (t_max[1] < t_max[0]) axis = 1;
    if (t_max[2] < t_max[axis]) axis = 2;
    if (!(t_max[axis] <= 1.0) || step[axis] == 0) break;
    if (axis == 0) x += step[0];
    else if (axis == 1) y += step[1];
    else z += step[2];
    t_max[axis] += t_delta[axis];
    ++steps;
  }
  markOccupied(end);
}

IntegrateStats OccupancyGrid::integrate(const std::vector<std::array<float, 3>>& points_sensor,
                                        const std::array<double, 12>& world_from_sensor) {
  IntegrateStats stats;
  stats.points_in = points_sensor.size();
  const auto origin = transformPoint(world_from_sensor, 0.0, 0.0, 0.0);
  if (!std::isfinite(origin[0]) || !std::isfinite(origin[1]) || !std::isfinite(origin[2])) {
    throw std::runtime_error("sensor origin is not finite");
  }
  const double min_range = params_.min_range_m;
  const double max_range = params_.max_range_m;

  for (const auto& point : points_sensor) {
    const double sx = static_cast<double>(point[0]);
    const double sy = static_cast<double>(point[1]);
    const double sz = static_cast<double>(point[2]);
    if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz)) {
      ++stats.points_rejected_nonfinite;
      continue;
    }
    const double range = std::sqrt(sx * sx + sy * sy + sz * sz);
    if (!(range >= min_range && range <= max_range)) {
      ++stats.points_rejected_range;
      continue;
    }
    const auto hit = transformPoint(world_from_sensor, sx, sy, sz);
    if (!std::isfinite(hit[0]) || !std::isfinite(hit[1]) || !std::isfinite(hit[2])) {
      ++stats.points_rejected_nonfinite;
      continue;
    }
    includeBound(origin[0], origin[1], origin[2]);
    includeBound(hit[0], hit[1], hit[2]);
    trace(origin, hit);
    ++stats.points_integrated;
  }
  return stats;
}

}  // namespace bdm
