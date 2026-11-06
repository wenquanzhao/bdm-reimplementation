#include "bdm/boundary_map.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>

using namespace bdm;

namespace {
int g_failures = 0;

void fail(const std::string& message) {
  std::cerr << "FAIL: " << message << "\n";
  ++g_failures;
}

const char* stateName(State state) {
  switch (state) {
    case State::Unknown: return "unknown";
    case State::Free: return "free";
    case State::Occupied: return "occupied";
  }
  return "invalid";
}

using Grid = std::unordered_map<Voxel, State, VoxelHash>;

State denseAt(const Grid& grid, Voxel voxel) {
  auto it = grid.find(voxel);
  return it == grid.end() ? State::Unknown : it->second;
}

bool hasEntry(const BoundaryMap& map, Voxel voxel, Boundary boundary) {
  auto it = map.columns().find({voxel.x, voxel.y});
  if (it == map.columns().end()) return false;
  for (const Entry& entry : it->second) {
    if (entry.z == voxel.z && entry.boundary == boundary) return true;
  }
  return false;
}

bool neighborIs(const Grid& grid, Voxel voxel, State expected) {
  static constexpr int kDelta[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const auto& d : kDelta) {
    if (denseAt(grid, {voxel.x + d[0], voxel.y + d[1], voxel.z + d[2]}) == expected) return true;
  }
  return false;
}

bool hasNonFreeNeighbor(const Grid& grid, Voxel voxel) {
  static constexpr int kDelta[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const auto& d : kDelta) {
    if (denseAt(grid, {voxel.x + d[0], voxel.y + d[1], voxel.z + d[2]}) != State::Free) return true;
  }
  return false;
}

std::size_t countBoundary(const BoundaryMap& map, Boundary boundary) {
  std::size_t count = 0;
  for (const auto& [column, entries] : map.columns()) {
    (void)column;
    for (const Entry& entry : entries) {
      if (entry.boundary == boundary) ++count;
    }
  }
  return count;
}

void expectState(const BoundaryMap& map, Voxel voxel, State expected, const char* label) {
  for (SearchDirection direction : {SearchDirection::PositiveZ, SearchDirection::NegativeZ}) {
    const State got = map.query(voxel, direction);
    if (got != expected) {
      const char* which = direction == SearchDirection::PositiveZ ? "z+" : "z-";
      fail(std::string(label) + " query(" + std::to_string(voxel.x) + "," +
           std::to_string(voxel.y) + "," + std::to_string(voxel.z) + ") " + which +
           " got " + stateName(got) + " expected " + stateName(expected));
    }
  }
  if (map.query(voxel) != map.query(voxel, SearchDirection::PositiveZ)) {
    fail(std::string(label) + " default query is not z+");
  }
}

// Independent check of the Section 4 boundary definition and of Algorithm 1
// against the dense grid. Both search directions are required to agree.
void checkAgainstDense(const BoundaryMap& map, const Grid& grid, const char* label) {
  std::size_t stored = 0;
  for (const auto& [column, entries] : map.columns()) {
    int previous_z = 0;
    for (std::size_t i = 0; i < entries.size(); ++i) {
      const Entry& entry = entries[i];
      ++stored;
      if (i > 0 && entry.z <= previous_z) {
        fail(std::string(label) + " column (" + std::to_string(column.x) + "," +
             std::to_string(column.y) + ") is not strictly increasing at z=" +
             std::to_string(entry.z));
      }
      previous_z = entry.z;
      const Voxel voxel{column.x, column.y, entry.z};
      const State state = denseAt(grid, voxel);
      if (entry.boundary == Boundary::Occupied) {
        if (state != State::Occupied) {
          fail(std::string(label) + " occupied boundary is not occupied");
        }
      } else if (entry.boundary == Boundary::InteriorFree) {
        if (state != State::Free || !hasNonFreeNeighbor(grid, voxel)) {
          fail(std::string(label) + " interior-free boundary does not match the definition");
        }
      } else if (state != State::Unknown || !neighborIs(grid, voxel, State::Free)) {
        fail(std::string(label) + " exterior-unknown boundary does not match the definition");
      }
    }
  }
  if (stored != map.boundaryCount()) {
    fail(std::string(label) + " boundaryCount " + std::to_string(map.boundaryCount()) +
         " != stored " + std::to_string(stored));
  }

  int min_x = 0, max_x = 0, min_y = 0, max_y = 0, min_z = 0, max_z = 0;
  bool any = false;
  auto include = [&](Voxel voxel) {
    if (!any) {
      min_x = max_x = voxel.x;
      min_y = max_y = voxel.y;
      min_z = max_z = voxel.z;
      any = true;
      return;
    }
    min_x = std::min(min_x, voxel.x);
    max_x = std::max(max_x, voxel.x);
    min_y = std::min(min_y, voxel.y);
    max_y = std::max(max_y, voxel.y);
    min_z = std::min(min_z, voxel.z);
    max_z = std::max(max_z, voxel.z);
  };
  for (const auto& [voxel, state] : grid) {
    (void)state;
    include(voxel);
  }
  for (const auto& [column, entries] : map.columns()) {
    for (const Entry& entry : entries) include({column.x, column.y, entry.z});
  }

  for (const auto& [voxel, state] : grid) {
    if (state == State::Occupied && !hasEntry(map, voxel, Boundary::Occupied)) {
      fail(std::string(label) + " dropped an occupied voxel");
    }
    if (state == State::Free && hasNonFreeNeighbor(grid, voxel) &&
        !hasEntry(map, voxel, Boundary::InteriorFree)) {
      fail(std::string(label) + " dropped an interior-free voxel");
    }
    if (state == State::Free && !hasNonFreeNeighbor(grid, voxel) &&
        hasEntry(map, voxel, Boundary::InteriorFree)) {
      fail(std::string(label) + " stored a non-boundary free voxel");
    }
    if (state == State::Unknown && neighborIs(grid, voxel, State::Free) &&
        !hasEntry(map, voxel, Boundary::ExteriorUnknown)) {
      fail(std::string(label) + " dropped an explicit exterior-unknown voxel");
    }
  }
  for (const auto& [voxel, state] : grid) {
    if (state != State::Free) continue;
    static constexpr int kDelta[6][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (const auto& d : kDelta) {
      Voxel neighbor{voxel.x + d[0], voxel.y + d[1], voxel.z + d[2]};
      if (denseAt(grid, neighbor) == State::Unknown &&
          !hasEntry(map, neighbor, Boundary::ExteriorUnknown)) {
        fail(std::string(label) + " missing exterior unknown next to free");
      }
    }
  }

  if (!any) {
    expectState(map, {0, 0, 0}, State::Unknown, label);
    return;
  }
  for (int x = min_x - 2; x <= max_x + 2; ++x) {
    for (int y = min_y - 2; y <= max_y + 2; ++y) {
      for (int z = min_z - 2; z <= max_z + 2; ++z) {
        expectState(map, {x, y, z}, denseAt(grid, {x, y, z}), label);
      }
    }
  }
  expectState(map, {min_x - 30, min_y - 30, min_z - 30}, State::Unknown, label);
  expectState(map, {max_x + 30, max_y + 30, max_z + 30}, State::Unknown, label);
}

void rebuildAndCheck(const Grid& grid, const char* label) {
  BoundaryMap map;
  map.rebuild(grid);
  checkAgainstDense(map, grid, label);
}

void testSmokeCubeWithObstacle() {
  Grid grid;
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      for (int z = 0; z < 8; ++z) grid[{x, y, z}] = State::Free;
    }
  }
  grid[{2, 2, 4}] = State::Occupied;
  BoundaryMap map;
  map.rebuild(grid);
  checkAgainstDense(map, grid, "smoke");
  // (2,2,3) is free and touches the occupied cell, so it is interior free.
  // The pre-fix source never stored that tag, and this query aborted.
  if (!hasEntry(map, {2, 2, 3}, Boundary::InteriorFree)) {
    fail("smoke: free cell beside occupied is not interior free");
  }
  if (countBoundary(map, Boundary::InteriorFree) == 0) {
    fail("smoke: rebuild produced no interior-free boundaries");
  }
  // (1,1,2) is free and surrounded by free cells, so it is not stored.
  if (hasEntry(map, {1, 1, 2}, Boundary::InteriorFree) ||
      hasEntry(map, {1, 1, 2}, Boundary::Occupied) ||
      hasEntry(map, {1, 1, 2}, Boundary::ExteriorUnknown)) {
    fail("smoke: deep free cell was stored");
  }
  expectState(map, {2, 2, 4}, State::Occupied, "smoke");
  expectState(map, {2, 2, 3}, State::Free, "smoke");
  expectState(map, {1, 1, 2}, State::Free, "smoke");
  expectState(map, {2, 2, -2}, State::Unknown, "smoke");
  expectState(map, {2, 2, 9}, State::Unknown, "smoke");

  map.rebuild({});
  if (map.boundaryCount() != 0 || !map.columns().empty()) {
    fail("smoke: empty rebuild did not clear the map");
  }
  expectState(map, {2, 2, 3}, State::Unknown, "smoke-cleared");
  expectState(map, {2, 2, 4}, State::Unknown, "smoke-cleared");
}

void testOccupiedOnly() {
  Grid grid;
  grid[{0, 0, 0}] = State::Occupied;
  grid[{0, 0, 2}] = State::Occupied;
  grid[{1, -3, 4}] = State::Occupied;
  grid[{4, 4, -6}] = State::Occupied;
  BoundaryMap map;
  map.rebuild(grid);
  checkAgainstDense(map, grid, "occupied-only");
  if (countBoundary(map, Boundary::InteriorFree) != 0 ||
      countBoundary(map, Boundary::ExteriorUnknown) != 0) {
    fail("occupied-only: stored a non-occupied boundary");
  }
  if (map.boundaryCount() != grid.size()) fail("occupied-only: count != occupied cells");
  // Nearest boundary in z+ from just below an occupied cell is occupied with
  // r_min > 0, which Algorithm 1 classifies as unknown.
  expectState(map, {0, 0, 1}, State::Unknown, "occupied-only");
  expectState(map, {0, 0, 0}, State::Occupied, "occupied-only");
}

void testDisconnectedFreeRegions() {
  Grid grid;
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      for (int z = 0; z < 3; ++z) grid[{x, y, z}] = State::Free;
      for (int z = 8; z < 11; ++z) grid[{x, y, z}] = State::Free;
    }
  }
  BoundaryMap map;
  map.rebuild(grid);
  checkAgainstDense(map, grid, "disconnected");
  if (countBoundary(map, Boundary::InteriorFree) == 0) {
    fail("disconnected: no interior free after rebuild");
  }
  // Centers of each solid box are free and not on the boundary.
  if (hasEntry(map, {1, 1, 1}, Boundary::InteriorFree)) {
    fail("disconnected: stored the center of the lower box");
  }
  if (hasEntry(map, {1, 1, 9}, Boundary::InteriorFree)) {
    fail("disconnected: stored the center of the upper box");
  }
  expectState(map, {1, 1, 1}, State::Free, "disconnected");
  expectState(map, {1, 1, 9}, State::Free, "disconnected");
  expectState(map, {1, 1, 5}, State::Unknown, "disconnected");
  // The unknown gap's nearest z+ boundary is exterior unknown, not interior free.
  expectState(map, {1, 1, 4}, State::Unknown, "disconnected");
}

void testFreeSlabTouchingOccupied() {
  // 3x3x5 free slab with occupied caps. The center is free, not stored, and
  // its nearest z boundary is an interior-free cell (r_min > 0).
  Grid grid;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = 1; z <= 5; ++z) grid[{x, y, z}] = State::Free;
      grid[{x, y, 0}] = State::Occupied;
      grid[{x, y, 6}] = State::Occupied;
    }
  }
  BoundaryMap map;
  map.rebuild(grid);
  checkAgainstDense(map, grid, "slab");
  if (hasEntry(map, {0, 0, 3}, Boundary::InteriorFree)) {
    fail("slab: stored a fully enclosed free cell");
  }
  if (!hasEntry(map, {0, 0, 1}, Boundary::InteriorFree) ||
      !hasEntry(map, {0, 0, 5}, Boundary::InteriorFree)) {
    fail("slab: free/occupied interface was not kept as interior free");
  }
  expectState(map, {0, 0, 3}, State::Free, "slab");
  expectState(map, {0, 0, 0}, State::Occupied, "slab");
  expectState(map, {0, 0, -1}, State::Unknown, "slab");
}

void testExplicitUnknownAndNegative() {
  Grid grid;
  for (int z = -5; z <= -2; ++z) grid[{-4, 2, z}] = State::Free;
  grid[{-4, 2, -1}] = State::Unknown;
  grid[{-4, 2, 0}] = State::Unknown;
  grid[{-4, 2, 1}] = State::Free;
  grid[{-4, 2, 2}] = State::Occupied;
  grid[{-4, 2, 3}] = State::Free;
  BoundaryMap map;
  map.rebuild(grid);
  checkAgainstDense(map, grid, "explicit-unknown");
  if (!hasEntry(map, {-4, 2, -1}, Boundary::ExteriorUnknown)) {
    fail("explicit-unknown: explicit unknown beside free was dropped");
  }
  if (countBoundary(map, Boundary::InteriorFree) == 0) {
    fail("explicit-unknown: no interior free");
  }
  expectState(map, {-4, 2, -3}, State::Free, "explicit-unknown");
  expectState(map, {-4, 2, 0}, State::Unknown, "explicit-unknown");
  expectState(map, {-4, 2, 2}, State::Occupied, "explicit-unknown");
}

void testSingleCellsAndEmpty() {
  BoundaryMap map;
  map.rebuild({});
  checkAgainstDense(map, {}, "empty");

  Grid one_free;
  one_free[{0, 0, 0}] = State::Free;
  rebuildAndCheck(one_free, "single-free");

  Grid one_occupied;
  one_occupied[{7, -2, 3}] = State::Occupied;
  rebuildAndCheck(one_occupied, "single-occupied");

  Grid only_unknown;
  only_unknown[{1, 1, 1}] = State::Unknown;
  only_unknown[{1, 1, 2}] = State::Unknown;
  BoundaryMap unknown_map;
  unknown_map.rebuild(only_unknown);
  checkAgainstDense(unknown_map, only_unknown, "unknown-only");
  if (unknown_map.boundaryCount() != 0) fail("unknown-only: stored a boundary");
}

void testColumnAdversary() {
  Grid grid;
  // One column with occupied, free runs, and unknown holes, plus a lateral
  // occupied neighbor that makes a mid-run free cell interior free.
  const State pattern[] = {
      State::Occupied, State::Free, State::Free, State::Unknown, State::Unknown,
      State::Free, State::Occupied, State::Free, State::Free, State::Free,
      State::Unknown, State::Occupied, State::Free};
  for (int z = 0; z < 13; ++z) grid[{0, 0, z}] = pattern[z];
  grid[{1, 0, 8}] = State::Occupied;
  for (int z = 0; z < 13; ++z) {
    if (pattern[z] == State::Free) grid[{-1, 0, z}] = State::Free;
  }
  rebuildAndCheck(grid, "column");
}

void testRandomScenes() {
  auto next = [](std::uint32_t& seed) {
    seed = seed * 1664525u + 1013904223u;
    return seed;
  };
  auto run = [&](std::uint32_t seed, int x0, int y0, int z0, int nx, int ny, int nz,
                 bool explicit_unknown, const char* label) {
    Grid grid;
    std::uint32_t state = seed;
    for (int x = 0; x < nx; ++x) {
      for (int y = 0; y < ny; ++y) {
        for (int z = 0; z < nz; ++z) {
          const std::uint32_t roll = next(state) % 5;
          Voxel voxel{x0 + x, y0 + y, z0 + z};
          if (roll <= 1) {
            if (explicit_unknown) grid[voxel] = State::Unknown;
          } else if (roll <= 3) {
            grid[voxel] = State::Free;
          } else {
            grid[voxel] = State::Occupied;
          }
        }
      }
    }
    rebuildAndCheck(grid, label);
  };
  run(1u, 0, 0, 0, 8, 8, 8, false, "random-8-a");
  run(2u, -3, 4, -2, 8, 8, 8, false, "random-8-b");
  run(99u, 0, 0, 0, 8, 8, 8, true, "random-8-explicit");
  run(7u, 0, 0, 0, 10, 10, 10, false, "random-10");
  run(11u, -5, -5, 1, 10, 9, 8, true, "random-10-explicit");
  run(42u, 0, 0, 0, 14, 12, 8, false, "random-14");
  run(123456789u, -8, 0, -4, 6, 16, 12, false, "random-6x16x12");
}

void testRebuildReplacesPreviousScene() {
  Grid free_box;
  for (int z = 0; z < 4; ++z) free_box[{0, 0, z}] = State::Free;
  BoundaryMap map;
  map.rebuild(free_box);
  if (countBoundary(map, Boundary::InteriorFree) == 0) {
    fail("replace: free box produced no interior free");
  }
  Grid occupied;
  occupied[{2, 2, 2}] = State::Occupied;
  map.rebuild(occupied);
  checkAgainstDense(map, occupied, "replace");
  expectState(map, {0, 0, 1}, State::Unknown, "replace");
  expectState(map, {2, 2, 2}, State::Occupied, "replace");
}
}  // namespace

int main() {
  testSmokeCubeWithObstacle();
  testOccupiedOnly();
  testDisconnectedFreeRegions();
  testFreeSlabTouchingOccupied();
  testExplicitUnknownAndNegative();
  testSingleCellsAndEmpty();
  testColumnAdversary();
  testRandomScenes();
  testRebuildReplacesPreviousScene();
  if (g_failures != 0) {
    std::cerr << g_failures << " failure(s)\n";
    return 1;
  }
  std::cout << "Static boundary-map tests passed\n";
  return 0;
}
