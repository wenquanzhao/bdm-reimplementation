#include "bdm/incremental_mapper.hpp"

#include "bdm/rigid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>

namespace bdm {
namespace {

constexpr Voxel kNeighbors[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                 {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};

bool shift(Voxel voxel, Voxel delta, Voxel& out) {
  const long long x = static_cast<long long>(voxel.x) + delta.x;
  const long long y = static_cast<long long>(voxel.y) + delta.y;
  const long long z = static_cast<long long>(voxel.z) + delta.z;
  const long long lo = std::numeric_limits<int>::min();
  const long long hi = std::numeric_limits<int>::max();
  if (x < lo || x > hi || y < lo || y > hi || z < lo || z > hi) return false;
  out = Voxel{static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)};
  return true;
}

template <typename StateFn>
std::optional<Boundary> boundaryTag(Voxel voxel, StateFn state_of) {
  const State state = state_of(voxel);
  if (state == State::Occupied) return Boundary::Occupied;
  if (state == State::Free) {
    for (const Voxel& delta : kNeighbors) {
      Voxel neighbor{};
      const State neighbor_state =
          shift(voxel, delta, neighbor) ? state_of(neighbor) : State::Unknown;
      if (neighbor_state != State::Free) return Boundary::InteriorFree;
    }
    return std::nullopt;
  }
  for (const Voxel& delta : kNeighbors) {
    Voxel neighbor{};
    if (!shift(voxel, delta, neighbor)) continue;
    if (state_of(neighbor) == State::Free) return Boundary::ExteriorUnknown;
  }
  return std::nullopt;
}

// [lo, hi) minus [cut_lo, cut_hi), as (begin, count) pairs. Ends may be one
// past INT_MAX when the window touches the top of the index range.
void subtractZ(long long lo, long long hi, long long cut_lo, long long cut_hi,
               std::vector<std::pair<int, int>>& out) {
  const long long left_hi = std::min(hi, cut_lo);
  if (lo < left_hi) out.emplace_back(static_cast<int>(lo), static_cast<int>(left_hi - lo));
  const long long right_lo = std::max(lo, cut_hi);
  if (right_lo < hi) out.emplace_back(static_cast<int>(right_lo), static_cast<int>(hi - right_lo));
}

bool entryAt(const std::vector<Entry>& entries, int z) {
  const auto pos = std::lower_bound(entries.begin(), entries.end(), z,
      [](const Entry& entry, int value) { return entry.z < value; });
  return pos != entries.end() && pos->z == z;
}

}  // namespace

IncrementalMapper::IncrementalMapper(IncrementalMapperParams params)
    : params_(params),
      model_(params_.odds),
      local_(params_.resolution_m, params_.min_range_m, params_.max_range_m, params_.size_x,
             params_.size_y, params_.size_z, model_) {
  if (!(params_.slide_threshold_m >= 0.0) || !std::isfinite(params_.slide_threshold_m)) {
    throw std::invalid_argument("slide_threshold_m must be a finite number >= 0");
  }
}

bool IncrementalMapper::inBox(Voxel voxel, Voxel origin) const {
  const long long x = voxel.x;
  const long long y = voxel.y;
  const long long z = voxel.z;
  return x >= origin.x && x < static_cast<long long>(origin.x) + local_.sizeX() && y >= origin.y &&
         y < static_cast<long long>(origin.y) + local_.sizeY() && z >= origin.z &&
         z < static_cast<long long>(origin.z) + local_.sizeZ();
}

Voxel IncrementalMapper::centeredOrigin(Voxel sensor_voxel) const {
  const long long x = static_cast<long long>(sensor_voxel.x) - local_.sizeX() / 2;
  const long long y = static_cast<long long>(sensor_voxel.y) - local_.sizeY() / 2;
  const long long z = static_cast<long long>(sensor_voxel.z) - local_.sizeZ() / 2;
  const long long lo = std::numeric_limits<int>::min();
  const long long hi = std::numeric_limits<int>::max();
  if (x < lo || x > hi || y < lo || y > hi || z < lo || z > hi) {
    throw std::runtime_error("local window origin is outside the int range");
  }
  return Voxel{static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)};
}

double IncrementalMapper::distanceFromCenter(const std::array<double, 3>& sensor_m) const {
  const std::array<double, 3> center = local_.centerMeters();
  const double dx = sensor_m[0] - center[0];
  const double dy = sensor_m[1] - center[1];
  const double dz = sensor_m[2] - center[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

State IncrementalMapper::queryGlobalAvoiding(Voxel voxel, Voxel window_origin) const {
  SearchDirection direction = SearchDirection::PositiveZ;
  const long long z0 = window_origin.z;
  const long long z1 = z0 + local_.sizeZ();
  if (voxel.z < z0) direction = SearchDirection::NegativeZ;
  else if (voxel.z >= z1) direction = SearchDirection::PositiveZ;
  return global_.query(voxel, direction);
}

State IncrementalMapper::combinedState(Voxel voxel) const {
  if (local_.placed() && local_.contains(voxel)) return local_.occupancy(voxel);
  if (!local_.placed()) return global_.query(voxel, SearchDirection::PositiveZ);
  return queryGlobalAvoiding(voxel, local_.origin());
}

void IncrementalMapper::fuseVoxel(Voxel voxel, State state) {
  const double delta = state == State::Free ? model_.l_free() : model_.l_occ();
  local_.accumulate(voxel, delta, nullptr);
  // Equation (11) writes the threshold. A clamp that still misses that state
  // is recorded so the next slide-out recomputes the boundary.
  if (local_.occupancy(voxel) != state) dirty_.insert(voxel);
}

std::size_t IncrementalMapper::fuseColumnInterval(int x, int y, int z0, int count,
                                                  Voxel old_origin) {
  if (count <= 0) return 0;
  const long long z_last = static_cast<long long>(z0) + count - 1;
  std::vector<Entry> inside;
  const auto column = global_.columns().find(Column{x, y});
  if (column != global_.columns().end()) {
    for (const Entry& entry : column->second) {
      if (entry.z >= z0 && static_cast<long long>(entry.z) <= z_last) inside.push_back(entry);
    }
  }

  std::unordered_map<int, State> states;
  std::size_t fused = 0;
  auto write = [&](int z, State state) {
    const Voxel voxel{x, y, z};
    fuseVoxel(voxel, state);
    ++fused;
  };

  if (inside.empty()) {
    // Theorem 4: an empty boundary set means one state for the whole segment.
    // Occupied voxels are stored, so a hit here is only a defensive fallback.
    const State probed = queryGlobalAvoiding(Voxel{x, y, z0}, old_origin);
    if (probed == State::Free) {
      for (int dz = 0; dz < count; ++dz) {
        write(static_cast<int>(static_cast<long long>(z0) + dz), State::Free);
      }
    } else if (probed == State::Occupied) {
      write(z0, State::Occupied);
    }
    return fused;
  }

  for (const Entry& entry : inside) {
    if (entry.boundary == Boundary::Occupied) states[entry.z] = State::Occupied;
    else if (entry.boundary == Boundary::InteriorFree) states[entry.z] = State::Free;
  }
  // Theorem 5: tiles grown from interior-free boundaries, stopping at the next
  // boundary or at the end of this slide-in segment, are free.
  for (const Entry& entry : inside) {
    if (entry.boundary != Boundary::InteriorFree) continue;
    for (int step : {1, -1}) {
      for (long long z = static_cast<long long>(entry.z) + step; z >= z0 && z <= z_last; z += step) {
        const int iz = static_cast<int>(z);
        if (entryAt(inside, iz)) break;
        if (states.find(iz) == states.end()) states.emplace(iz, State::Free);
      }
    }
  }
  for (const auto& [z, state] : states) write(z, state);
  return fused;
}

std::size_t IncrementalMapper::fuseSlideIn(Voxel old_origin, Voxel new_origin) {
  std::size_t fused = 0;
  const int sx = local_.sizeX();
  const int sy = local_.sizeY();
  const int sz = local_.sizeZ();
  for (int dx = 0; dx < sx; ++dx) {
    for (int dy = 0; dy < sy; ++dy) {
      const int x = new_origin.x + dx;
      const int y = new_origin.y + dy;
      const bool xy_in_old =
          static_cast<long long>(x) >= old_origin.x &&
          static_cast<long long>(x) < static_cast<long long>(old_origin.x) + sx &&
          static_cast<long long>(y) >= old_origin.y &&
          static_cast<long long>(y) < static_cast<long long>(old_origin.y) + sy;
      const long long new_z0 = new_origin.z;
      const long long new_z1 = new_z0 + sz;
      const long long old_z0 = old_origin.z;
      const long long old_z1 = old_z0 + sz;
      std::vector<std::pair<int, int>> intervals;
      if (!xy_in_old) {
        intervals.emplace_back(new_origin.z, sz);
      } else {
        subtractZ(new_z0, new_z1, old_z0, old_z1, intervals);
      }
      for (const auto& interval : intervals) {
        fused += fuseColumnInterval(x, y, interval.first, interval.second, old_origin);
      }
    }
  }
  return fused;
}

void IncrementalMapper::refreshLiveBoundaries(
    const std::unordered_set<Voxel, VoxelHash>& changed_now) {
  if (!local_.placed() || changed_now.empty()) return;

  // Changed voxel plus its 6-neighbors. A neighbor one cell outside the window
  // can flip between interior-free, exterior-unknown, and absent.
  std::unordered_set<Voxel, VoxelHash> update_set;
  for (const Voxel& voxel : changed_now) {
    if (!local_.contains(voxel)) continue;
    update_set.insert(voxel);
    for (const Voxel& delta : kNeighbors) {
      Voxel neighbor{};
      if (shift(voxel, delta, neighbor)) update_set.insert(neighbor);
    }
  }
  if (update_set.empty()) return;

  // Snapshot the update set and one further ring before any write. The outer
  // ring is classification context, same as slideTo, and is not stored itself.
  const Voxel origin = local_.origin();
  std::unordered_map<Voxel, State, VoxelHash> snapshot;
  auto remember = [&](Voxel voxel) {
    if (snapshot.find(voxel) != snapshot.end()) return;
    const State state =
        local_.contains(voxel) ? local_.occupancy(voxel) : queryGlobalAvoiding(voxel, origin);
    snapshot.emplace(voxel, state);
  };
  for (const Voxel& voxel : update_set) {
    remember(voxel);
    for (const Voxel& delta : kNeighbors) {
      Voxel neighbor{};
      if (shift(voxel, delta, neighbor)) remember(neighbor);
    }
  }
  auto state_of = [&](Voxel voxel) {
    const auto found = snapshot.find(voxel);
    return found == snapshot.end() ? State::Unknown : found->second;
  };
  for (const Voxel& voxel : update_set) {
    const std::optional<Boundary> tag = boundaryTag(voxel, state_of);
    if (tag.has_value()) global_.insertAt(voxel, *tag);
    else global_.eraseAt(voxel);
  }
}

void IncrementalMapper::slideTo(Voxel new_origin, FrameUpdate& update) {
  const Voxel old_origin = local_.origin();

  std::vector<Voxel> leaving;
  leaving.reserve(dirty_.size());
  for (const Voxel& voxel : dirty_) {
    if (inBox(voxel, old_origin) && !inBox(voxel, new_origin)) leaving.push_back(voxel);
  }
  update.committed_voxels = leaving.size();

  std::unordered_set<Voxel, VoxelHash> update_set;
  for (const Voxel& voxel : leaving) {
    update_set.insert(voxel);
    for (const Voxel& delta : kNeighbors) {
      Voxel neighbor{};
      if (shift(voxel, delta, neighbor)) update_set.insert(neighbor);
    }
    dirty_.erase(voxel);
  }
  update.boundary_touched = update_set.size();

  // Snapshot occupancy before any edit. Slide-in slots alias slide-out slots
  // in the rolling array, so this has to happen before place() and the clear.
  std::unordered_map<Voxel, State, VoxelHash> snapshot;
  auto remember = [&](Voxel voxel) {
    if (snapshot.find(voxel) != snapshot.end()) return;
    const State state = inBox(voxel, old_origin) ? local_.occupancy(voxel)
                                                  : queryGlobalAvoiding(voxel, old_origin);
    snapshot.emplace(voxel, state);
  };
  for (const Voxel& voxel : update_set) {
    remember(voxel);
    for (const Voxel& delta : kNeighbors) {
      Voxel neighbor{};
      if (shift(voxel, delta, neighbor)) remember(neighbor);
    }
  }
  auto state_of = [&](Voxel voxel) {
    const auto found = snapshot.find(voxel);
    return found == snapshot.end() ? State::Unknown : found->second;
  };
  for (const Voxel& voxel : update_set) {
    const std::optional<Boundary> tag = boundaryTag(voxel, state_of);
    if (tag.has_value()) global_.insertAt(voxel, *tag);
    else global_.eraseAt(voxel);
  }

  local_.place(new_origin);
  for (int dx = 0; dx < local_.sizeX(); ++dx) {
    for (int dy = 0; dy < local_.sizeY(); ++dy) {
      for (int dz = 0; dz < local_.sizeZ(); ++dz) {
        const Voxel voxel{new_origin.x + dx, new_origin.y + dy, new_origin.z + dz};
        if (!inBox(voxel, old_origin)) local_.setLogOdds(voxel, 0.0);
      }
    }
  }
  update.slide_in_fused = fuseSlideIn(old_origin, new_origin);
}

FrameUpdate IncrementalMapper::integrate(const std::vector<std::array<float, 3>>& points_sensor,
                                         const std::array<double, 12>& world_from_sensor) {
  const auto sensor_m = transformPoint(world_from_sensor, 0.0, 0.0, 0.0);
  if (!std::isfinite(sensor_m[0]) || !std::isfinite(sensor_m[1]) || !std::isfinite(sensor_m[2])) {
    throw std::runtime_error("sensor origin is not finite");
  }
  const Voxel sensor_voxel = local_.voxelAt(sensor_m[0], sensor_m[1], sensor_m[2]);
  if (!local_.placed()) local_.place(centeredOrigin(sensor_voxel));

  FrameUpdate update;
  // A fresh set: dirty_ already holds every voxel that changed and has not
  // slid out, so a membership diff would miss a second flip (frame C {2,0,0}).
  std::unordered_set<Voxel, VoxelHash> changed_now;
  update.rays = local_.integrate(points_sensor, world_from_sensor, changed_now);
  dirty_.insert(changed_now.begin(), changed_now.end());
  refreshLiveBoundaries(changed_now);
  if (distanceFromCenter(sensor_m) > params_.slide_threshold_m) {
    const Voxel next = centeredOrigin(sensor_voxel);
    const Voxel current = local_.origin();
    if (next.x != current.x || next.y != current.y || next.z != current.z) {
      slideTo(next, update);
      update.slid = true;
      ++slide_count_;
    }
  }
  return update;
}

State IncrementalMapper::query(const Voxel& voxel) const { return combinedState(voxel); }

BoundaryMap IncrementalMapper::mergedBoundaryMap() const {
  BoundaryMap merged = global_;
  if (!local_.placed()) return merged;
  const Voxel origin = local_.origin();
  std::unordered_map<Voxel, std::optional<Boundary>, VoxelHash> desired;
  auto consider = [&](Voxel voxel) {
    if (desired.find(voxel) != desired.end()) return;
    desired.emplace(voxel, boundaryTag(voxel, [&](Voxel query) { return combinedState(query); }));
  };
  for (int dx = 0; dx < local_.sizeX(); ++dx) {
    for (int dy = 0; dy < local_.sizeY(); ++dy) {
      for (int dz = 0; dz < local_.sizeZ(); ++dz) {
        const Voxel voxel{origin.x + dx, origin.y + dy, origin.z + dz};
        consider(voxel);
        for (const Voxel& delta : kNeighbors) {
          Voxel neighbor{};
          if (shift(voxel, delta, neighbor)) consider(neighbor);
        }
      }
    }
  }
  for (const auto& [voxel, tag] : desired) {
    if (tag.has_value()) merged.insertAt(voxel, *tag);
    else merged.eraseAt(voxel);
  }
  return merged;
}

}  // namespace bdm
