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
State lookup(const std::unordered_map<Voxel, State, VoxelHash>& grid, Voxel v) {
  auto it = grid.find(v);
  return it == grid.end() ? State::Unknown : it->second;
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
void BoundaryMap::rebuild(const std::unordered_map<Voxel, State, VoxelHash>& voxels) {
  columns_.clear();
  count_ = 0;
  for (const auto& [v, state] : voxels) {
    bool keep = (state == State::Occupied);
    if (state != State::Occupied) {
      for (const auto& d : kNeighbors) {
        State neighbor = lookup(voxels, {v.x+d.x,v.y+d.y,v.z+d.z});
        if (state == State::Free && neighbor == State::Unknown) keep = true;
        if (state == State::Unknown && neighbor == State::Free) keep = true;
      }
    }
    if (keep) {
      const auto tag = state == State::Occupied ? Boundary::Occupied :
                       state == State::Free ? Boundary::InteriorFree : Boundary::ExteriorUnknown;
      columns_[{v.x, v.y}].push_back({v.z, tag});
      ++count_;
    }
  }
  // Missing unknown voxels also need explicit exterior boundary entries.
  // Include just the 6-connected halo of known FREE cells.
  for (const auto& [v, state] : voxels) {
    if (state != State::Free) continue;
    for (const auto& d : kNeighbors) {
      Voxel n{v.x+d.x,v.y+d.y,v.z+d.z};
      if (lookup(voxels, n) != State::Unknown) continue;
      if (voxels.find(n) != voxels.end()) continue; // Explicit UNKNOWN handled above.
      auto& col = columns_[{n.x,n.y}];
      if (std::none_of(col.begin(), col.end(), [&](const Entry& e){return e.z == n.z;})) {
        col.push_back({n.z,Boundary::ExteriorUnknown});
        ++count_;
      }
    }
  }
  for (auto& [key, entries] : columns_) {
    (void)key;
    std::sort(entries.begin(), entries.end(), [](const Entry&a,const Entry&b){return a.z<b.z;});
  }
}
State BoundaryMap::query(const Voxel& v) const {
  auto it = columns_.find({v.x,v.y});
  if (it == columns_.end()) return State::Unknown;
  const auto& entries = it->second;
  auto lb = std::lower_bound(entries.begin(), entries.end(), v.z,
       [](const Entry& e, int z){return e.z < z;});
  if (lb != entries.end() && lb->z == v.z)
    return lb->boundary == Boundary::Occupied ? State::Occupied :
           lb->boundary == Boundary::InteriorFree ? State::Free : State::Unknown;
  // Interior FREE is inferred only across a matching enclosing pair in the same column.
  if (lb != entries.begin() && lb != entries.end()) {
    const auto& below = *(lb - 1);
    const auto& above = *lb;
    if (below.boundary == Boundary::InteriorFree && above.boundary == Boundary::InteriorFree)
      return State::Free;
  }
  return State::Unknown;
}
std::size_t BoundaryMap::boundaryCount() const noexcept { return count_; }
}  // namespace bdm
