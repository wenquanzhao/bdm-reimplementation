#pragma once

#include "bdm/boundary_map.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace bdm {

// Voxel index of a world point. Throws if the index does not fit in int.
inline Voxel voxelIndex(double x, double y, double z, double resolution) {
  const double inv = 1.0 / resolution;
  const double scaled[3] = {x * inv, y * inv, z * inv};
  const double lo = static_cast<double>(std::numeric_limits<int>::min());
  const double hi = static_cast<double>(std::numeric_limits<int>::max());
  int index[3];
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(scaled[axis]) || scaled[axis] < lo || scaled[axis] > hi) {
      throw std::runtime_error("voxel index is outside the int range");
    }
    index[axis] = static_cast<int>(std::floor(scaled[axis]));
  }
  return {index[0], index[1], index[2]};
}

// Amanatides & Woo traversal shared by the binary grid and the log-odds grid.
// t is 0 at the sensor and 1 at the return. Equal face-crossing times step x,
// then y, then z. A ray that only touches a corner may skip one of the two
// face-adjacent cells.
//
// on_miss is called for every voxel before the endpoint voxel.
// on_hit is always called for the endpoint, including a zero-length ray.
// The callback decides what a hit or a miss means. This function does not.
template <typename MissFn, typename HitFn>
void castVoxelRay(Voxel start, Voxel end, const std::array<double, 3>& origin,
                  const std::array<double, 3>& hit, double resolution, MissFn on_miss,
                  HitFn on_hit) {
  const double dir[3] = {hit[0] - origin[0], hit[1] - origin[1], hit[2] - origin[2]};
  const double length_sq = dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2];
  if (length_sq < 1e-18 || (start.x == end.x && start.y == end.y && start.z == end.z)) {
    on_hit(end);
    return;
  }

  const double inf = std::numeric_limits<double>::infinity();
  int step[3];
  double t_max[3];
  double t_delta[3];
  const int start_index[3] = {start.x, start.y, start.z};
  for (int axis = 0; axis < 3; ++axis) {
    if (dir[axis] > 0.0) {
      step[axis] = 1;
      const double boundary = (static_cast<double>(start_index[axis]) + 1.0) * resolution;
      t_max[axis] = (boundary - origin[axis]) / dir[axis];
      t_delta[axis] = resolution / dir[axis];
    } else if (dir[axis] < 0.0) {
      step[axis] = -1;
      const double boundary = static_cast<double>(start_index[axis]) * resolution;
      t_max[axis] = (boundary - origin[axis]) / dir[axis];
      t_delta[axis] = -resolution / dir[axis];
    } else {
      step[axis] = 0;
      t_max[axis] = inf;
      t_delta[axis] = inf;
    }
  }

  int x = start.x;
  int y = start.y;
  int z = start.z;
  const int span = std::abs(end.x - start.x) + std::abs(end.y - start.y) +
                   std::abs(end.z - start.z);
  const int max_steps = span + 3;
  int steps = 0;
  while (steps < max_steps && !(x == end.x && y == end.y && z == end.z)) {
    on_miss(Voxel{x, y, z});
    int axis = 0;
    if (t_max[1] < t_max[0]) axis = 1;
    if (t_max[2] < t_max[axis]) axis = 2;
    if (!(t_max[axis] <= 1.0) || step[axis] == 0) break;
    if (axis == 0) {
      x += step[0];
    } else if (axis == 1) {
      y += step[1];
    } else {
      z += step[2];
    }
    t_max[axis] += t_delta[axis];
    ++steps;
  }
  on_hit(end);
}

}  // namespace bdm
