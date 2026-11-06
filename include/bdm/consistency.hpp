#pragma once

#include "bdm/boundary_map.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace bdm {

struct QueryMismatch {
  Voxel voxel;
  State expected = State::Unknown;
  State positive_z = State::Unknown;
  State negative_z = State::Unknown;
};

struct CompareOptions {
  // Extra unknown cells queried around the dense-grid bounding box.
  int padding = 1;
  // If the padded window is larger than this, compare stored voxels and
  // stored boundary cells only, and set window_skipped.
  std::uint64_t max_window_voxels = 20000000;
  std::size_t max_examples = 32;
};

struct ConsistencyReport {
  std::size_t queries = 0;
  std::size_t matched = 0;
  std::size_t mismatched = 0;
  bool used_bounding_window = false;
  bool window_skipped = false;
  std::uint64_t window_voxels = 0;
  std::vector<QueryMismatch> examples;
};

// Query every selected voxel along z+ and z-. A voxel matches when both
// directions equal the dense state (a missing key is Unknown).
// This checks the static map against the grid it was built from. It is not
// a paper Section 8 metric.
ConsistencyReport compareWithDense(const BoundaryMap& map, const VoxelGrid& grid,
                                   CompareOptions options = {});

}  // namespace bdm
