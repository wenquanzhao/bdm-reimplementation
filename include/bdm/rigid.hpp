#pragma once

#include <array>

namespace bdm {

// Row-major 3x4 rigid transform. The implicit fourth row is [0 0 0 1].
// Composition is matrix product: composeRigid(A, B) applies B first, then A.
inline std::array<double, 12> composeRigid(const std::array<double, 12>& lhs,
                                           const std::array<double, 12>& rhs) {
  std::array<double, 12> out{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out[row * 4 + col] = lhs[row * 4 + 0] * rhs[0 * 4 + col] +
                           lhs[row * 4 + 1] * rhs[1 * 4 + col] +
                           lhs[row * 4 + 2] * rhs[2 * 4 + col];
    }
    out[row * 4 + 3] = lhs[row * 4 + 0] * rhs[0 * 4 + 3] +
                       lhs[row * 4 + 1] * rhs[1 * 4 + 3] +
                       lhs[row * 4 + 2] * rhs[2 * 4 + 3] + lhs[row * 4 + 3];
  }
  return out;
}

inline std::array<double, 3> transformPoint(const std::array<double, 12>& rigid,
                                           double x, double y, double z) {
  return {rigid[0] * x + rigid[1] * y + rigid[2] * z + rigid[3],
          rigid[4] * x + rigid[5] * y + rigid[6] * z + rigid[7],
          rigid[8] * x + rigid[9] * y + rigid[10] * z + rigid[11]};
}

}  // namespace bdm
