#include "bdm/occupancy_grid.hpp"

#include "bdm/ray_cast.hpp"
#include "bdm/rigid.hpp"

#include <algorithm>
#include <cmath>
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
  return voxelIndex(x, y, z, params_.resolution_m);
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
  // Same traversal as the log-odds local grid. The binary rule is the callbacks:
  // a miss is free unless the cell is already occupied, and the endpoint is occupied.
  castVoxelRay(start, end, origin, hit, params_.resolution_m,
               [&](Voxel voxel) { markFree(voxel); }, [&](Voxel voxel) { markOccupied(voxel); });
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
