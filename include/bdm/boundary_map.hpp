#pragma once
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace bdm {

enum class State : std::uint8_t { Unknown = 0, Free = 1, Occupied = 2 };
enum class Boundary : std::uint8_t { InteriorFree = 0, ExteriorUnknown = 1, Occupied = 2 };

struct Voxel {
  int x{}, y{}, z{};
  bool operator==(const Voxel& rhs) const noexcept {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }
};
struct VoxelHash {
  std::size_t operator()(const Voxel& v) const noexcept;
};
struct Column {
  int x{}, y{};
  bool operator==(const Column& rhs) const noexcept { return x == rhs.x && y == rhs.y; }
};
struct ColumnHash {
  std::size_t operator()(const Column& v) const noexcept;
};
struct Entry { int z; Boundary boundary; };

// Reference implementation of a *static* boundary representation.
// Occupied cells and the six-connected FREE/UNKNOWN interface are retained.
// Out-of-bounds is UNKNOWN. This is not the paper's dynamic update algorithm.
class BoundaryMap {
public:
  void rebuild(const std::unordered_map<Voxel, State, VoxelHash>& voxels);
  State query(const Voxel& voxel) const;
  std::size_t boundaryCount() const noexcept;
  const std::unordered_map<Column, std::vector<Entry>, ColumnHash>& columns() const noexcept {
    return columns_;
  }
private:
  std::unordered_map<Column, std::vector<Entry>, ColumnHash> columns_;
  std::size_t count_{0};
};
}  // namespace bdm
