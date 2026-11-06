#include "bdm/consistency.hpp"

#include <limits>
#include <stdexcept>

namespace bdm {
namespace {

int expandLow(int value, int padding) {
  if (padding <= 0) return value;
  if (value < std::numeric_limits<int>::min() + padding) return std::numeric_limits<int>::min();
  return value - padding;
}

int expandHigh(int value, int padding) {
  if (padding <= 0) return value;
  if (value > std::numeric_limits<int>::max() - padding) return std::numeric_limits<int>::max();
  return value + padding;
}

std::uint64_t span(int low, int high) {
  return static_cast<std::uint64_t>(static_cast<long long>(high) - static_cast<long long>(low)) +
         1ull;
}

State expectedAt(const VoxelGrid& grid, Voxel voxel) {
  const auto it = grid.find(voxel);
  return it == grid.end() ? State::Unknown : it->second;
}

}  // namespace

ConsistencyReport compareWithDense(const BoundaryMap& map, const VoxelGrid& grid,
                                   CompareOptions options) {
  if (options.padding < 0) {
    throw std::invalid_argument("padding must be >= 0");
  }
  ConsistencyReport report;
  if (grid.empty() && map.boundaryCount() == 0) return report;

  auto consider = [&](Voxel voxel) {
    const State expected = expectedAt(grid, voxel);
    const State positive = map.query(voxel, SearchDirection::PositiveZ);
    const State negative = map.query(voxel, SearchDirection::NegativeZ);
    ++report.queries;
    if (positive == expected && negative == expected) {
      ++report.matched;
      return;
    }
    ++report.mismatched;
    if (report.examples.size() < options.max_examples) {
      report.examples.push_back(QueryMismatch{voxel, expected, positive, negative});
    }
  };

  if (!grid.empty()) {
    int min_x = 0, max_x = 0, min_y = 0, max_y = 0, min_z = 0, max_z = 0;
    bool any = false;
    for (const auto& [voxel, state] : grid) {
      (void)state;
      if (!any) {
        min_x = max_x = voxel.x;
        min_y = max_y = voxel.y;
        min_z = max_z = voxel.z;
        any = true;
      } else {
        if (voxel.x < min_x) min_x = voxel.x;
        if (voxel.x > max_x) max_x = voxel.x;
        if (voxel.y < min_y) min_y = voxel.y;
        if (voxel.y > max_y) max_y = voxel.y;
        if (voxel.z < min_z) min_z = voxel.z;
        if (voxel.z > max_z) max_z = voxel.z;
      }
    }
    min_x = expandLow(min_x, options.padding);
    min_y = expandLow(min_y, options.padding);
    min_z = expandLow(min_z, options.padding);
    max_x = expandHigh(max_x, options.padding);
    max_y = expandHigh(max_y, options.padding);
    max_z = expandHigh(max_z, options.padding);

    const std::uint64_t nx = span(min_x, max_x);
    const std::uint64_t ny = span(min_y, max_y);
    const std::uint64_t nz = span(min_z, max_z);
    bool overflow = false;
    std::uint64_t volume = 0;
    if (ny != 0 && nx > std::numeric_limits<std::uint64_t>::max() / ny) {
      overflow = true;
    } else {
      const std::uint64_t xy = nx * ny;
      if (nz != 0 && xy > std::numeric_limits<std::uint64_t>::max() / nz) overflow = true;
      else volume = xy * nz;
    }
    report.window_voxels = overflow ? std::numeric_limits<std::uint64_t>::max() : volume;
    if (!overflow && volume <= options.max_window_voxels) {
      report.used_bounding_window = true;
      for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
          for (int z = min_z; z <= max_z; ++z) consider({x, y, z});
        }
      }
      return report;
    }
  }

  report.window_skipped = true;
  for (const auto& [voxel, state] : grid) {
    (void)state;
    consider(voxel);
  }
  for (const auto& [column, entries] : map.columns()) {
    for (const Entry& entry : entries) {
      const Voxel voxel{column.x, column.y, entry.z};
      if (grid.find(voxel) == grid.end()) consider(voxel);
    }
  }
  return report;
}

}  // namespace bdm
