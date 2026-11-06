#include "bdm/boundary_map.hpp"
#include "bdm/incremental_mapper.hpp"
#include "bdm/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

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

bool near(double got, double expected) { return std::abs(got - expected) <= 1e-9; }

std::array<double, 12> translation(double x, double y, double z) {
  return {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z};
}

const std::array<double, 12> kPoseA = translation(0.5, 0.5, 0.5);

IncrementalMapperParams cube(int size, double threshold, bool strong_miss) {
  IncrementalMapperParams params;
  params.resolution_m = 1.0;
  params.min_range_m = 0.0;
  params.max_range_m = 30.0;
  params.size_x = size;
  params.size_y = size;
  params.size_z = size;
  params.slide_threshold_m = threshold;
  if (strong_miss) {
    // One miss reaches l_free. Section 8.2 uses p_miss = 0.48, which does not.
    params.odds.p_miss = 0.2;
  }
  return params;
}

void expectOccupancy(const IncrementalMapper& mapper, Voxel voxel, State expected,
                     const char* label) {
  const State got = mapper.query(voxel);
  if (got != expected) {
    fail(std::string(label) + " query(" + std::to_string(voxel.x) + "," +
         std::to_string(voxel.y) + "," + std::to_string(voxel.z) + ") got " + stateName(got) +
         " expected " + stateName(expected));
  }
}

void expectOdds(const IncrementalMapper& mapper, Voxel voxel, double expected, const char* label) {
  const double got = mapper.local().logOdds(voxel);
  if (!near(got, expected)) {
    fail(std::string(label) + " log-odds(" + std::to_string(voxel.x) + "," +
         std::to_string(voxel.y) + "," + std::to_string(voxel.z) + ") got " + std::to_string(got) +
         " expected " + std::to_string(expected));
  }
}

bool hasEntry(const BoundaryMap& map, Voxel voxel, Boundary boundary) {
  const auto it = map.columns().find(Column{voxel.x, voxel.y});
  if (it == map.columns().end()) return false;
  for (const Entry& entry : it->second) {
    if (entry.z == voxel.z && entry.boundary == boundary) return true;
  }
  return false;
}

void expectSorted(const BoundaryMap& map, const char* label) {
  std::size_t count = 0;
  for (const auto& [column, entries] : map.columns()) {
    (void)column;
    for (std::size_t i = 1; i < entries.size(); ++i) {
      if (!(entries[i - 1].z < entries[i].z)) {
        fail(std::string(label) + " column is not strictly increasing in z");
        return;
      }
    }
    count += entries.size();
  }
  if (count != map.boundaryCount()) {
    fail(std::string(label) + " boundaryCount does not match stored entries");
  }
}

bool sameColumns(const BoundaryMap& lhs, const BoundaryMap& rhs) {
  if (lhs.boundaryCount() != rhs.boundaryCount()) return false;
  if (lhs.columns().size() != rhs.columns().size()) return false;
  for (const auto& [column, entries] : lhs.columns()) {
    const auto it = rhs.columns().find(column);
    if (it == rhs.columns().end() || it->second.size() != entries.size()) return false;
    for (std::size_t i = 0; i < entries.size(); ++i) {
      if (entries[i].z != it->second[i].z || entries[i].boundary != it->second[i].boundary) {
        return false;
      }
    }
  }
  return true;
}

void expectOverlap(const IncrementalMapper& mapper, const char* label) {
  const BoundaryMap merged = mapper.mergedBoundaryMap();
  expectSorted(merged, label);
  const Voxel origin = mapper.local().origin();
  for (int dx = 0; dx < mapper.local().sizeX(); ++dx) {
    for (int dy = 0; dy < mapper.local().sizeY(); ++dy) {
      for (int dz = 0; dz < mapper.local().sizeZ(); ++dz) {
        const Voxel voxel{origin.x + dx, origin.y + dy, origin.z + dz};
        const State expected = mapper.local().occupancy(voxel);
        const State positive = merged.query(voxel, SearchDirection::PositiveZ);
        const State negative = merged.query(voxel, SearchDirection::NegativeZ);
        if (positive != expected || negative != expected) {
          fail(std::string(label) + " overlap (" + std::to_string(voxel.x) + "," +
               std::to_string(voxel.y) + "," + std::to_string(voxel.z) + ") local=" +
               stateName(expected) + " z+=" + stateName(positive) + " z-=" + stateName(negative));
          return;
        }
      }
    }
  }
}

void expectAgree(const IncrementalMapper& sliding, const IncrementalMapper& reference,
                 const char* label) {
  for (int x = -6; x <= 8; ++x) {
    for (int y = -4; y <= 4; ++y) {
      for (int z = -4; z <= 4; ++z) {
        const Voxel voxel{x, y, z};
        const State got = sliding.query(voxel);
        const State expected = reference.query(voxel);
        if (got != expected) {
          fail(std::string(label) + " (" + std::to_string(x) + "," + std::to_string(y) + "," +
               std::to_string(z) + ") sliding=" + stateName(got) +
               " reference=" + stateName(expected));
          return;
        }
      }
    }
  }
}

void testPaperDefaultOdds() {
  const LogOddsModel model;
  if (!near(model.l_hit(), std::log(0.8 / 0.2))) fail("default l_hit");
  if (!near(model.l_miss(), std::log(0.48 / 0.52))) fail("default l_miss");
  if (!near(model.l_min(), std::log(0.05 / 0.95))) fail("default l_min");
  if (!near(model.l_max(), std::log(0.97 / 0.03))) fail("default l_max");
  if (!near(model.l_free(), std::log(0.2 / 0.8))) fail("default l_free");
  if (!near(model.l_occ(), std::log(0.8 / 0.2))) fail("default l_occ");
  if (model.state(0.0) != State::Unknown) fail("prior should be unknown");
  if (model.state(model.l_hit()) != State::Occupied) fail("one default hit should be occupied");
  if (model.state(model.l_miss()) != State::Unknown) fail("one default miss should stay unknown");
}

void testPaperMissStaysUnknown() {
  IncrementalMapper mapper(cube(11, 1.0, false));
  const FrameUpdate update = mapper.integrate({{3.f, 0.f, 0.f}}, kPoseA);
  if (update.slid) fail("paper miss: unexpected slide");
  if (update.rays.points_integrated != 1) fail("paper miss: not integrated");
  const LogOddsModel& model = mapper.local().model();
  expectOdds(mapper, {0, 0, 0}, model.l_miss(), "paper miss sensor");
  expectOdds(mapper, {1, 0, 0}, model.l_miss(), "paper miss mid");
  expectOdds(mapper, {2, 0, 0}, model.l_miss(), "paper miss before hit");
  expectOdds(mapper, {3, 0, 0}, model.l_hit(), "paper hit");
  expectOccupancy(mapper, {2, 0, 0}, State::Unknown, "paper miss");
  expectOccupancy(mapper, {3, 0, 0}, State::Occupied, "paper hit");
  expectOverlap(mapper, "paper miss");

  OccupancyGrid binary(OccupancyGridParams{1.0, 0.0, 30.0});
  binary.integrate({{3.f, 0.f, 0.f}}, kPoseA);
  const auto mid = binary.voxels().find(Voxel{2, 0, 0});
  if (mid == binary.voxels().end() || mid->second != State::Free) {
    fail("binary grid should mark the same miss free");
  }
}

void testMissClearsOccupied() {
  OccupancyGrid binary(OccupancyGridParams{1.0, 0.0, 30.0});
  binary.integrate({{2.f, 0.f, 0.f}}, kPoseA);
  binary.integrate({{4.f, 0.f, 0.f}}, kPoseA);
  const auto sticky = binary.voxels().find(Voxel{2, 0, 0});
  if (sticky == binary.voxels().end() || sticky->second != State::Occupied) {
    fail("binary grid should keep the first hit");
  }

  IncrementalMapper mapper(cube(11, 1.0, true));
  mapper.integrate({{2.f, 0.f, 0.f}}, kPoseA);
  expectOccupancy(mapper, {2, 0, 0}, State::Occupied, "before clearing ray");
  mapper.integrate({{4.f, 0.f, 0.f}}, kPoseA);
  expectOccupancy(mapper, {2, 0, 0}, State::Unknown, "miss clears a single hit");
  expectOccupancy(mapper, {4, 0, 0}, State::Occupied, "farther hit");
  expectOdds(mapper, {2, 0, 0}, 0.0, "hit plus miss returns to the prior");
  if (mapper.slideCount() != 0) fail("clearing ray should not slide");
  expectOverlap(mapper, "cleared hit");
}

void testClamp() {
  IncrementalMapper mapper(cube(5, 1.0, true));
  std::vector<std::array<float, 3>> hits(10, {0.1f, 0.f, 0.f});
  mapper.integrate(hits, kPoseA);
  expectOdds(mapper, {0, 0, 0}, mapper.local().model().l_max(), "clamp");
  expectOccupancy(mapper, {0, 0, 0}, State::Occupied, "clamped hit");
}

void testEndpointOutsideWindow() {
  IncrementalMapper mapper(cube(5, 10.0, true));
  const FrameUpdate update = mapper.integrate({{10.f, 0.f, 0.f}}, kPoseA);
  if (update.rays.points_endpoint_outside != 1) fail("outside: endpoint counter");
  if (mapper.local().contains({10, 0, 0})) fail("outside: endpoint was stored");
  expectOccupancy(mapper, {2, 0, 0}, State::Free, "outside: last inside voxel is a miss");
  expectOccupancy(mapper, {0, 0, 0}, State::Free, "outside: sensor voxel is a miss");
}

void testBadParams() {
  auto throws_invalid = [](IncrementalMapperParams params) {
    try {
      IncrementalMapper mapper(params);
      (void)mapper;
      return false;
    } catch (const std::invalid_argument&) {
      return true;
    }
  };
  IncrementalMapperParams zero = cube(4, 1.0, false);
  zero.size_z = 0;
  if (!throws_invalid(zero)) fail("size 0 should throw");
  IncrementalMapperParams huge = cube(4, 1.0, false);
  huge.size_x = 400;
  huge.size_y = 400;
  huge.size_z = 400;
  if (!throws_invalid(huge)) fail("huge window should throw");
  IncrementalMapperParams odds = cube(4, 1.0, false);
  odds.odds.p_occ = 0.4;
  if (!throws_invalid(odds)) fail("p_occ below 0.5 should throw");
  IncrementalMapperParams threshold = cube(4, 1.0, false);
  threshold.slide_threshold_m = -0.1;
  if (!throws_invalid(threshold)) fail("negative threshold should throw");
}

void testColumnEditsStaySorted() {
  BoundaryMap map;
  map.insertAt({1, 2, 3}, Boundary::Occupied);
  map.insertAt({1, 2, 1}, Boundary::InteriorFree);
  map.insertAt({1, 2, 3}, Boundary::ExteriorUnknown);
  if (map.boundaryCount() != 2) fail("edit: count after replace");
  const auto it = map.columns().find(Column{1, 2});
  if (it == map.columns().end() || it->second.size() != 2) {
    fail("edit: column size");
    return;
  }
  if (it->second[0].z != 1 || it->second[0].boundary != Boundary::InteriorFree) {
    fail("edit: first entry");
  }
  if (it->second[1].z != 3 || it->second[1].boundary != Boundary::ExteriorUnknown) {
    fail("edit: replaced entry");
  }
  map.eraseAt({1, 2, 1});
  map.eraseAt({9, 9, 9});
  if (map.boundaryCount() != 1) fail("edit: count after erase");
  map.eraseAt({1, 2, 3});
  if (map.boundaryCount() != 0 || !map.columns().empty()) fail("edit: empty column remains");
}

bool storedAt(const BoundaryMap& map, Voxel voxel) {
  return hasEntry(map, voxel, Boundary::InteriorFree) ||
         hasEntry(map, voxel, Boundary::ExteriorUnknown) ||
         hasEntry(map, voxel, Boundary::Occupied);
}

void testFreeVolumeRoundTrip() {
  // A solid free cube has a center cell with six free neighbors. That cell is
  // not a boundary. Sliding it out and back has to rebuild it from the
  // interior-free faces (Section 7.2.3), not from a stored free voxel.
  IncrementalMapper sliding(cube(9, 1.0, true));
  IncrementalMapper reference(cube(32, 1.0e9, true));
  std::vector<std::array<float, 3>> rays;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      for (int z = -2; z <= 2; ++z) {
        if (std::max(std::abs(x), std::max(std::abs(y), std::abs(z))) != 2) continue;
        rays.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      }
    }
  }
  const FrameUpdate filled = sliding.integrate(rays, kPoseA);
  reference.integrate(rays, kPoseA);
  if (filled.slid) fail("volume: first scan slid");
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        expectOccupancy(sliding, {x, y, z}, State::Free, "volume fill");
      }
    }
  }
  if (g_failures != 0) return;

  const FrameUpdate slid = sliding.integrate({}, translation(6.5, 0.5, 0.5));
  reference.integrate({}, translation(6.5, 0.5, 0.5));
  if (!slid.slid || sliding.slideCount() != 1) fail("volume: slide");
  if (sliding.local().contains({0, 0, 0})) fail("volume: center stayed in the window");
  if (storedAt(sliding.globalMap(), {0, 0, 0})) {
    fail("volume: enclosed free cell was stored as a boundary");
  }
  expectOccupancy(sliding, {0, 0, 0}, State::Free, "volume center after slide-out");
  expectOccupancy(sliding, {1, 0, 0}, State::Free, "volume face after slide-out");
  expectAgree(sliding, reference, "volume slide-out");
  expectOverlap(sliding, "volume slide-out");

  const FrameUpdate returned = sliding.integrate({}, kPoseA);
  reference.integrate({}, kPoseA);
  if (!returned.slid || returned.slide_in_fused < 27) fail("volume: slide-in fusion");
  expectOdds(sliding, {0, 0, 0}, sliding.local().model().l_free(), "volume center re-entry");
  if (!(reference.local().logOdds({0, 0, 0}) < sliding.local().model().l_free() - 1e-9)) {
    fail("volume: reference was not more free than the fused threshold");
  }
  expectOccupancy(sliding, {0, 0, 0}, State::Free, "volume center after re-entry");
  expectAgree(sliding, reference, "volume re-entry");
  expectOverlap(sliding, "volume re-entry");
}

void testStreamingSlideAndReentry() {
  // Window 7, threshold 1 m. Pose A centers the window on voxel 0: x in [-3, 4).
  // Two hits at voxel -3 and one hit at voxel 2. Pose B is far enough to slide
  // so x in [0, 7) after the ray is cast into the old window. Pose C does not
  // slide. The return to pose A loads the slide-out blob back through equation (11).
  IncrementalMapper sliding(cube(7, 1.0, true));
  IncrementalMapper reference(cube(32, 1.0e9, true));
  const LogOddsModel& model = sliding.local().model();

  const std::vector<std::array<float, 3>> scan_a = {{-3.f, 0.f, 0.f}, {-3.f, 0.f, 0.f}, {2.f, 0.f, 0.f}};
  const FrameUpdate frame_a = sliding.integrate(scan_a, kPoseA);
  reference.integrate(scan_a, kPoseA);
  if (frame_a.slid || sliding.slideCount() != 0) fail("frame A slid");
  if (sliding.globalMap().boundaryCount() != 0) fail("frame A wrote the global map");
  expectOdds(sliding, {-3, 0, 0}, 2.0 * model.l_hit(), "frame A occupied odds");
  expectOdds(sliding, {-2, 0, 0}, 2.0 * model.l_miss(), "frame A free odds");
  expectOdds(sliding, {-1, 0, 0}, 2.0 * model.l_miss(), "frame A free odds -1");
  expectOdds(sliding, {0, 0, 0}, model.l_min(), "frame A sensor clamped");
  expectOdds(sliding, {2, 0, 0}, model.l_hit(), "frame A far hit");
  expectOccupancy(sliding, {-3, 0, 0}, State::Occupied, "frame A occupied");
  expectOccupancy(sliding, {-2, 0, 0}, State::Free, "frame A free");
  expectOverlap(sliding, "frame A");
  expectAgree(sliding, reference, "frame A");
  const double odds_before_slide = sliding.local().logOdds({2, 0, 0});

  const std::array<double, 12> pose_b = translation(3.5, 0.5, 0.5);
  const std::vector<std::array<float, 3>> scan_b = {{0.f, 0.f, 2.f}};
  const FrameUpdate frame_b = sliding.integrate(scan_b, pose_b);
  reference.integrate(scan_b, pose_b);
  if (!frame_b.slid || sliding.slideCount() != 1) fail("frame B should slide once");
  if (frame_b.committed_voxels < 3) fail("frame B committed too little");
  if (sliding.globalMap().boundaryCount() == 0) fail("frame B global map is empty");
  if (!hasEntry(sliding.globalMap(), {-3, 0, 0}, Boundary::Occupied)) {
    fail("frame B did not store the slid-out hit");
  }
  expectSorted(sliding.globalMap(), "frame B global");
  if (sliding.local().origin().x != 0 || sliding.local().origin().y != -3) {
    fail("frame B origin");
  }
  if (!sliding.local().contains({2, 0, 0}) || !sliding.local().contains({3, 0, 2})) {
    fail("frame B lost the overlap or the new hit");
  }
  if (!near(sliding.local().logOdds({2, 0, 0}), odds_before_slide)) {
    fail("slide changed an overlap cell the new ray did not touch");
  }
  if (!near(sliding.local().logOdds({2, 0, 0}), reference.local().logOdds({2, 0, 0}))) {
    fail("overlap log-odds diverged from the reference");
  }
  expectOccupancy(sliding, {-3, 0, 0}, State::Occupied, "slid-out hit");
  expectOccupancy(sliding, {-2, 0, 0}, State::Free, "slid-out free");
  expectOccupancy(sliding, {3, 0, 2}, State::Occupied, "frame B hit");
  expectOverlap(sliding, "frame B");
  expectAgree(sliding, reference, "frame B");
  const BoundaryMap global_after_slide = sliding.globalMap();

  const std::array<double, 12> pose_c = translation(3.6, 0.5, 0.5);
  const FrameUpdate frame_c = sliding.integrate({{-3.1f, 0.f, 0.f}}, pose_c);
  reference.integrate({{-3.1f, 0.f, 0.f}}, pose_c);
  if (frame_c.slid || sliding.slideCount() != 1) fail("frame C should not slide");
  if (!sameColumns(sliding.globalMap(), global_after_slide)) {
    fail("a scan that does not slide rewrote the global boundary map");
  }
  expectOccupancy(sliding, {2, 0, 0}, State::Unknown, "frame C cleared the overlap hit");
  expectOverlap(sliding, "frame C");
  expectAgree(sliding, reference, "frame C");

  const FrameUpdate frame_return = sliding.integrate({}, kPoseA);
  reference.integrate({}, kPoseA);
  if (!frame_return.slid || sliding.slideCount() != 2) fail("return should slide");
  if (frame_return.slide_in_fused < 3) fail("return fused too little");
  if (sliding.local().origin().x != -3) fail("return origin");
  expectOdds(sliding, {-3, 0, 0}, model.l_occ(), "re-entry occupied threshold");
  expectOdds(sliding, {-2, 0, 0}, model.l_free(), "re-entry free threshold");
  expectOdds(sliding, {-1, 0, 0}, model.l_free(), "re-entry free threshold -1");
  expectOdds(sliding, {-2, 0, 1}, 0.0, "re-entry did not fill unknown above the ray");
  if (near(sliding.local().logOdds({-3, 0, 0}), reference.local().logOdds({-3, 0, 0}))) {
    fail("re-entry kept the original accumulated log-odds");
  }
  expectOccupancy(sliding, {-3, 0, 0}, State::Occupied, "re-entry state");
  expectOccupancy(reference, {-3, 0, 0}, State::Occupied, "reference still occupied");
  expectOverlap(sliding, "return");
  expectAgree(sliding, reference, "return");
  if (!hasEntry(sliding.globalMap(), {-3, 0, 0}, Boundary::Occupied)) {
    fail("slide-in deleted the global occupied entry");
  }

  // A further miss changes the local cell and leaves the global entry stale
  // until the cell slides out. Section 7.3 reads the local grid inside the window.
  const FrameUpdate frame_edit = sliding.integrate({{-4.f, 0.f, 0.f}}, kPoseA);
  if (frame_edit.slid) fail("edit slid");
  if (frame_edit.rays.points_endpoint_outside != 1) fail("edit endpoint should leave the window");
  expectOdds(sliding, {-3, 0, 0}, 0.0, "local hit cleared back to the prior");
  expectOccupancy(sliding, {-3, 0, 0}, State::Unknown, "query follows the local grid");
  if (sliding.globalMap().query({-3, 0, 0}) != State::Occupied) {
    fail("global entry should still be the occupied boundary until the next slide");
  }
  expectOverlap(sliding, "stale global");

  const FrameUpdate frame_repair = sliding.integrate({}, translation(2.5, 0.5, 0.5));
  if (!frame_repair.slid || sliding.slideCount() != 3) fail("repair should slide");
  if (!hasEntry(sliding.globalMap(), {-3, 0, 0}, Boundary::ExteriorUnknown)) {
    fail("slide-out did not replace the stale occupied boundary");
  }
  if (sliding.globalMap().query({-3, 0, 0}) != State::Unknown) {
    fail("repaired global query");
  }
  expectOverlap(sliding, "repair");
}

}  // namespace

int main() {
  try {
    testPaperDefaultOdds();
    testPaperMissStaysUnknown();
    testMissClearsOccupied();
    testClamp();
    testEndpointOutsideWindow();
    testBadParams();
    testColumnEditsStaySorted();
    testStreamingSlideAndReentry();
    testFreeVolumeRoundTrip();
  } catch (const std::exception& ex) {
    fail(std::string("exception: ") + ex.what());
  }
  if (g_failures != 0) {
    std::cerr << g_failures << " failure(s)\n";
    return 1;
  }
  std::cout << "ok\n";
  return 0;
}
