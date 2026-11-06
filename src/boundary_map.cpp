#include "bdm/boundary_map.hpp"
#include <algorithm>
#include <array>
#include <cstdint>

namespace bdm {
namespace {
std::size_t mix(std::size_t h, std::uint32_t v) noexcept {
  return (h ^ static_cast<std::size_t>(v)) * static_cast<std::size_t>(1099511628211ULL);
}
constexpr std::array<Voxel, 6> kNeighbors{{
    {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}
}};
State lookup(const VoxelGrid& grid, Voxel v) {
  auto it = grid.find(v);
  return it == grid.end() ? State::Unknown : it->second;
}

// First stored boundary with z >= qz (z+), or the last with z <= qz (z-).
// Section 6.2. A missing column or no such entry is the null nearest boundary.
const Entry* nearestInDirection(const std::vector<Entry>& entries, int qz,
                                 SearchDirection direction) {
  if (entries.empty()) return nullptr;
  if (direction == SearchDirection::PositiveZ) {
    auto it = std::lower_bound(entries.begin(), entries.end(), qz,
        [](const Entry& e, int z) { return e.z < z; });
    return it == entries.end() ? nullptr : &*it;
  }
  auto it = std::upper_bound(entries.begin(), entries.end(), qz,
      [](int z, const Entry& e) { return z < e.z; });
  return it == entries.begin() ? nullptr : &*(it - 1);
}

// Algorithm 1. r_min = |b_nn.z - q.z|; r_min == 0 iff the z coordinates match
// because the column already shares (x, y) with the query.
State stateFromNearest(const Entry* nearest, int qz) {
  if (nearest == nullptr) return State::Unknown;
  if (nearest->boundary == Boundary::InteriorFree) return State::Free;
  if (nearest->boundary == Boundary::Occupied && nearest->z == qz) return State::Occupied;
  return State::Unknown;
}
}  // namespace
std::size_t VoxelHash::operator()(const Voxel& v) const noexcept {
  std::size_t h = 1469598103934665603ULL;
  h = mix(h, static_cast<std::uint32_t>(v.x));
  h = mix(h, static_cast<std::uint32_t>(v.y));
  return mix(h, static_cast<std::uint32_t>(v.z));
}
std::size_t ColumnHash::operator()(const Column& v) const noexcept {
  std::size_t h = 1469598103934665603ULL;
  h = mix(h, static_cast<std::uint32_t>(v.x));
  return mix(h, static_cast<std::uint32_t>(v.y));
}
void BoundaryMap::rebuild(const VoxelGrid& voxels) {
  columns_.clear();
  count_ = 0;
  for (const auto& [v, state] : voxels) {
    bool keep = (state == State::Occupied);
    if (state != State::Occupied) {
      for (const auto& d : kNeighbors) {
        const State neighbor = lookup(voxels, {v.x + d.x, v.y + d.y, v.z + d.z});
        // Interior free: free, with at least one 6-neighbor unknown or occupied.
        if (state == State::Free && neighbor != State::Free) keep = true;
        // Exterior unknown: unknown, and at least one 6-neighbor is free.
        if (state == State::Unknown && neighbor == State::Free) keep = true;
      }
    }
    if (!keep) continue;
    const auto tag = state == State::Occupied ? Boundary::Occupied :
                     state == State::Free ? Boundary::InteriorFree : Boundary::ExteriorUnknown;
    columns_[{v.x, v.y}].push_back({v.z, tag});
    ++count_;
  }
  // Unknown cells omitted from the input are still exterior boundaries
  // when they 6-neighbor a free cell.
  for (const auto& [v, state] : voxels) {
    if (state != State::Free) continue;
    for (const auto& d : kNeighbors) {
      Voxel n{v.x + d.x, v.y + d.y, v.z + d.z};
      if (lookup(voxels, n) != State::Unknown) continue;
      if (voxels.find(n) != voxels.end()) continue;  // Explicit unknown handled above.
      auto& col = columns_[{n.x, n.y}];
      if (std::none_of(col.begin(), col.end(), [&](const Entry& e) { return e.z == n.z; })) {
        col.push_back({n.z, Boundary::ExteriorUnknown});
        ++count_;
      }
    }
  }
  for (auto& [key, entries] : columns_) {
    (void)key;
    std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
      return a.z < b.z;
    });
  }
}
void BoundaryMap::eraseAt(const Voxel& voxel) {
  auto it = columns_.find({voxel.x, voxel.y});
  if (it == columns_.end()) return;
  auto& entries = it->second;
  auto pos = std::lower_bound(entries.begin(), entries.end(), voxel.z,
      [](const Entry& entry, int z) { return entry.z < z; });
  if (pos == entries.end() || pos->z != voxel.z) return;
  entries.erase(pos);
  --count_;
  if (entries.empty()) columns_.erase(it);
}
void BoundaryMap::insertAt(const Voxel& voxel, Boundary boundary) {
  auto& entries = columns_[{voxel.x, voxel.y}];
  auto pos = std::lower_bound(entries.begin(), entries.end(), voxel.z,
      [](const Entry& entry, int z) { return entry.z < z; });
  if (pos != entries.end() && pos->z == voxel.z) {
    pos->boundary = boundary;
    return;
  }
  entries.insert(pos, Entry{voxel.z, boundary});
  ++count_;
}
State BoundaryMap::query(const Voxel& v) const {
  return query(v, SearchDirection::PositiveZ);
}
State BoundaryMap::query(const Voxel& v, SearchDirection direction) const {
  auto it = columns_.find({v.x, v.y});
  if (it == columns_.end()) return State::Unknown;
  return stateFromNearest(nearestInDirection(it->second, v.z, direction), v.z);
}
std::size_t BoundaryMap::boundaryCount() const noexcept { return count_; }
}  // namespace bdm
