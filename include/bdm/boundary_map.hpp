#pragma once
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace bdm {

enum class State : std::uint8_t { Unknown = 0, Free = 1, Occupied = 2 };
enum class Boundary : std::uint8_t { InteriorFree = 0, ExteriorUnknown = 1, Occupied = 2 };

// Search direction along the z projection axis (arXiv:2603.21774, Section 6.2).
enum class SearchDirection : std::uint8_t { PositiveZ = 0, NegativeZ = 1 };

struct Voxel {
  int x{}, y{}, z{};
  bool operator==(const Voxel& rhs) const noexcept {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }
};
struct VoxelHash {
  std::size_t operator()(const Voxel& v) const noexcept;
};
using VoxelGrid = std::unordered_map<Voxel, State, VoxelHash>;
struct Column {
  int x{}, y{};
  bool operator==(const Column& rhs) const noexcept { return x == rhs.x && y == rhs.y; }
};
struct ColumnHash {
  std::size_t operator()(const Column& v) const noexcept;
};
struct Entry { int z; Boundary boundary; };

// Static boundary map of one occupancy grid, projected along z.
//
// rebuild() stores boundary voxels only:
//   occupied cells;
//   free cells with a 6-neighbor that is unknown or occupied (interior free);
//   unknown cells with a free 6-neighbor (exterior unknown), including
//   cells that are absent from the input and therefore unknown.
// query(v) is Algorithm 1 with search direction z+.
// query(v, direction) selects z+ or z-.
// Column lookup is a hash plus a binary search, O(log k) in the column length.
// This is not the global-local updater, ray caster, or paper benchmark stack.
class BoundaryMap {
public:
  void rebuild(const VoxelGrid& voxels);
  State query(const Voxel& voxel) const;
  State query(const Voxel& voxel, SearchDirection direction) const;
  std::size_t boundaryCount() const noexcept;
  const std::unordered_map<Column, std::vector<Entry>, ColumnHash>& columns() const noexcept {
    return columns_;
  }
private:
  std::unordered_map<Column, std::vector<Entry>, ColumnHash> columns_;
  std::size_t count_{0};
};
}  // namespace bdm
