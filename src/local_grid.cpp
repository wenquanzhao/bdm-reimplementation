#include "bdm/local_grid.hpp"

#include "bdm/ray_cast.hpp"
#include "bdm/rigid.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace bdm {
namespace {

int floorMod(int value, int modulus) {
  const int remainder = value % modulus;
  return remainder < 0 ? remainder + modulus : remainder;
}

}  // namespace

LocalGrid::LocalGrid(double resolution_m, double min_range_m, double max_range_m, int size_x,
                     int size_y, int size_z, LogOddsModel model)
    : resolution_(resolution_m),
      min_range_m_(min_range_m),
      max_range_m_(max_range_m),
      size_x_(size_x),
      size_y_(size_y),
      size_z_(size_z),
      model_(std::move(model)) {
  if (!(resolution_ > 0.0) || !std::isfinite(resolution_)) {
    throw std::invalid_argument("resolution_m must be a finite number greater than 0");
  }
  if (!(min_range_m_ >= 0.0) || !std::isfinite(min_range_m_)) {
    throw std::invalid_argument("min_range_m must be a finite number >= 0");
  }
  if (!(max_range_m_ >= min_range_m_) || !std::isfinite(max_range_m_)) {
    throw std::invalid_argument("max_range_m must be finite and >= min_range_m");
  }
  if (size_x_ < 1 || size_y_ < 1 || size_z_ < 1) {
    throw std::invalid_argument("local window sizes must be >= 1");
  }
  const double voxels = static_cast<double>(size_x_) * static_cast<double>(size_y_) *
                        static_cast<double>(size_z_);
  if (!(voxels <= 32000000.0)) {
    throw std::invalid_argument("local window exceeds 32000000 voxels");
  }
  odds_.assign(static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(size_y_) *
                   static_cast<std::size_t>(size_z_),
               0.0);
}

bool LocalGrid::contains(Voxel voxel) const noexcept {
  if (!placed_) return false;
  const long long x = voxel.x;
  const long long y = voxel.y;
  const long long z = voxel.z;
  return x >= origin_.x && x < static_cast<long long>(origin_.x) + size_x_ && y >= origin_.y &&
         y < static_cast<long long>(origin_.y) + size_y_ && z >= origin_.z &&
         z < static_cast<long long>(origin_.z) + size_z_;
}

Voxel LocalGrid::voxelAt(double x, double y, double z) const {
  return voxelIndex(x, y, z, resolution_);
}

std::array<double, 3> LocalGrid::centerMeters() const {
  if (!placed_) {
    throw std::logic_error("local window has not been placed");
  }
  return {(static_cast<double>(origin_.x) + static_cast<double>(size_x_) / 2.0) * resolution_,
          (static_cast<double>(origin_.y) + static_cast<double>(size_y_) / 2.0) * resolution_,
          (static_cast<double>(origin_.z) + static_cast<double>(size_z_) / 2.0) * resolution_};
}

std::size_t LocalGrid::slot(Voxel voxel) const {
  const int ix = floorMod(voxel.x, size_x_);
  const int iy = floorMod(voxel.y, size_y_);
  const int iz = floorMod(voxel.z, size_z_);
  return static_cast<std::size_t>(ix) +
         static_cast<std::size_t>(size_x_) *
             (static_cast<std::size_t>(iy) + static_cast<std::size_t>(size_y_) *
                                                 static_cast<std::size_t>(iz));
}

double LocalGrid::logOdds(Voxel voxel) const {
  if (!contains(voxel)) {
    throw std::out_of_range("voxel is outside the local window");
  }
  return odds_[slot(voxel)];
}

State LocalGrid::occupancy(Voxel voxel) const { return model_.state(logOdds(voxel)); }

void LocalGrid::checkWindowFits(Voxel origin) const {
  const long long ends[3] = {static_cast<long long>(origin.x) + size_x_ - 1,
                             static_cast<long long>(origin.y) + size_y_ - 1,
                             static_cast<long long>(origin.z) + size_z_ - 1};
  const long long lo = std::numeric_limits<int>::min();
  const long long hi = std::numeric_limits<int>::max();
  for (long long end : ends) {
    if (end < lo || end > hi) {
      throw std::runtime_error("local window origin is outside the int range");
    }
  }
}

void LocalGrid::place(Voxel origin) {
  checkWindowFits(origin);
  origin_ = origin;
  placed_ = true;
}

void LocalGrid::setLogOdds(Voxel voxel, double value) {
  if (!contains(voxel)) {
    throw std::out_of_range("voxel is outside the local window");
  }
  odds_[slot(voxel)] = model_.clamp(value);
}

void LocalGrid::accumulate(Voxel voxel, double delta,
                           std::unordered_set<Voxel, VoxelHash>* changed) {
  if (!contains(voxel)) return;
  double& slot_value = odds_[slot(voxel)];
  const State before = model_.state(slot_value);
  slot_value = model_.clamp(slot_value + delta);
  const State after = model_.state(slot_value);
  if (changed != nullptr && before != after) changed->insert(voxel);
}

LocalIntegrateStats LocalGrid::integrate(const std::vector<std::array<float, 3>>& points_sensor,
                                         const std::array<double, 12>& world_from_sensor,
                                         std::unordered_set<Voxel, VoxelHash>& changed) {
  if (!placed_) {
    throw std::logic_error("local window has not been placed");
  }
  LocalIntegrateStats stats;
  stats.points_in = points_sensor.size();
  const auto origin = transformPoint(world_from_sensor, 0.0, 0.0, 0.0);
  if (!std::isfinite(origin[0]) || !std::isfinite(origin[1]) || !std::isfinite(origin[2])) {
    throw std::runtime_error("sensor origin is not finite");
  }
  const double l_miss = model_.l_miss();
  const double l_hit = model_.l_hit();

  for (const auto& point : points_sensor) {
    const double sx = static_cast<double>(point[0]);
    const double sy = static_cast<double>(point[1]);
    const double sz = static_cast<double>(point[2]);
    if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz)) {
      ++stats.points_rejected_nonfinite;
      continue;
    }
    const double range = std::sqrt(sx * sx + sy * sy + sz * sz);
    if (!(range >= min_range_m_ && range <= max_range_m_)) {
      ++stats.points_rejected_range;
      continue;
    }
    const auto hit = transformPoint(world_from_sensor, sx, sy, sz);
    if (!std::isfinite(hit[0]) || !std::isfinite(hit[1]) || !std::isfinite(hit[2])) {
      ++stats.points_rejected_nonfinite;
      continue;
    }
    const Voxel start = voxelAt(origin[0], origin[1], origin[2]);
    const Voxel end = voxelAt(hit[0], hit[1], hit[2]);
    if (!contains(end)) ++stats.points_endpoint_outside;
    castVoxelRay(start, end, origin, hit, resolution_,
                 [&](Voxel voxel) { accumulate(voxel, l_miss, &changed); },
                 [&](Voxel voxel) { accumulate(voxel, l_hit, &changed); });
    ++stats.points_integrated;
  }
  return stats;
}

}  // namespace bdm
