#include "bdm/boundary_map.hpp"
#include "bdm/consistency.hpp"
#include "bdm/kitti_loader.hpp"
#include "bdm/occupancy_grid.hpp"
#include "bdm/rigid.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef BDM_FIXTURE_ROOT
#error BDM_FIXTURE_ROOT must be the tests/fixtures/kitti_mini directory
#endif

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

std::array<double, 12> translation(double x, double y, double z) {
  return {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z};
}

bool near(double got, double expected) { return std::abs(got - expected) < 1e-9; }

void expectState(const VoxelGrid& grid, Voxel voxel, State expected, const char* label) {
  const auto it = grid.find(voxel);
  const State got = it == grid.end() ? State::Unknown : it->second;
  if (got != expected) {
    fail(std::string(label) + " voxel (" + std::to_string(voxel.x) + "," +
         std::to_string(voxel.y) + "," + std::to_string(voxel.z) + ") got " + stateName(got) +
         " expected " + stateName(expected));
  }
}

void expectConsistent(const OccupancyGrid& grid, const char* label) {
  BoundaryMap map;
  map.rebuild(grid.voxels());
  const ConsistencyReport report = compareWithDense(map, grid.voxels(), CompareOptions{});
  if (!report.used_bounding_window) fail(std::string(label) + " did not use the bounding window");
  if (report.mismatched != 0) {
    fail(std::string(label) + " mismatches " + std::to_string(report.mismatched));
    for (const QueryMismatch& mismatch : report.examples) {
      std::cerr << "  mismatch (" << mismatch.voxel.x << "," << mismatch.voxel.y << ","
                << mismatch.voxel.z << ") dense=" << stateName(mismatch.expected)
                << " z+=" << stateName(mismatch.positive_z)
                << " z-=" << stateName(mismatch.negative_z) << "\n";
    }
  }
  if (report.queries == 0) fail(std::string(label) + " compared no voxels");
}

void expectExact(const VoxelGrid& grid, const std::vector<std::pair<Voxel, State>>& expected,
                 const char* label) {
  if (grid.size() != expected.size()) {
    fail(std::string(label) + " voxel count " + std::to_string(grid.size()) + " expected " +
         std::to_string(expected.size()));
  }
  for (const auto& [voxel, state] : expected) expectState(grid, voxel, state, label);
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

class TempDir {
 public:
  TempDir() {
    static int seq = 0;
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("bdm-kitti-" + std::to_string(ticks) + "-" + std::to_string(seq++));
    std::filesystem::create_directories(path_);
  }
  ~TempDir() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

void writeText(const std::filesystem::path& path, const std::string& text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  out << text;
  if (!out) fail("cannot write " + path.string());
}

void writeBin(const std::filesystem::path& path, const std::vector<std::array<float, 3>>& points) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::binary);
  for (const auto& point : points) {
    const float record[4] = {point[0], point[1], point[2], 0.25f};
    out.write(reinterpret_cast<const char*>(record), sizeof(record));
  }
  if (!out) fail("cannot write " + path.string());
}

void testAxisAlignedHitMiss() {
  OccupancyGrid grid(OccupancyGridParams{1.0, 0.1, 20.0});
  const auto stats = grid.integrate({{5.f, 0.f, 0.f}}, translation(0.5, 0.5, 0.5));
  if (stats.points_integrated != 1) fail("axis: integrated count");
  expectExact(grid.voxels(),
              {{{0, 0, 0}, State::Free},
               {{1, 0, 0}, State::Free},
               {{2, 0, 0}, State::Free},
               {{3, 0, 0}, State::Free},
               {{4, 0, 0}, State::Free},
               {{5, 0, 0}, State::Occupied}},
              "axis");
  expectState(grid.voxels(), {6, 0, 0}, State::Unknown, "axis-beyond");
  expectConsistent(grid, "axis");

  BoundaryMap map;
  map.rebuild(grid.voxels());
  if (countBoundary(map, Boundary::Occupied) == 0) fail("axis: no occupied boundary");
  if (countBoundary(map, Boundary::InteriorFree) == 0) fail("axis: no interior free");
  if (countBoundary(map, Boundary::ExteriorUnknown) == 0) fail("axis: no exterior unknown");
  if (map.query({5, 0, 0}) != State::Occupied) fail("axis: query occupied");
  if (map.query({2, 0, 0}, SearchDirection::NegativeZ) != State::Free) fail("axis: query free z-");
}

void testOccupiedSticky() {
  OccupancyGrid grid(OccupancyGridParams{1.0, 0.1, 20.0});
  grid.integrate({{5.f, 0.f, 0.f}}, translation(0.5, 0.5, 0.5));
  grid.integrate({{8.f, 0.f, 0.f}}, translation(0.5, 0.5, 0.5));
  expectExact(grid.voxels(),
              {{{0, 0, 0}, State::Free},
               {{1, 0, 0}, State::Free},
               {{2, 0, 0}, State::Free},
               {{3, 0, 0}, State::Free},
               {{4, 0, 0}, State::Free},
               {{5, 0, 0}, State::Occupied},
               {{6, 0, 0}, State::Free},
               {{7, 0, 0}, State::Free},
               {{8, 0, 0}, State::Occupied}},
              "sticky");
  expectConsistent(grid, "sticky");
}

void testRangeAndNonFinite() {
  OccupancyGrid grid(OccupancyGridParams{1.0, 1.0, 10.0});
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto stats = grid.integrate(
      {{0.2f, 0.f, 0.f}, {20.f, 0.f, 0.f}, {nan, 1.f, 0.f}, {4.f, 0.f, 0.f}},
      translation(0.5, 0.5, 0.5));
  if (stats.points_in != 4) fail("range: points_in");
  if (stats.points_rejected_range != 2) fail("range: rejected range");
  if (stats.points_rejected_nonfinite != 1) fail("range: rejected nonfinite");
  if (stats.points_integrated != 1) fail("range: integrated");
  expectExact(grid.voxels(),
              {{{0, 0, 0}, State::Free},
               {{1, 0, 0}, State::Free},
               {{2, 0, 0}, State::Free},
               {{3, 0, 0}, State::Free},
               {{4, 0, 0}, State::Occupied}},
              "range");
}

void testDiagonalRay() {
  // Sensor at the center of voxel (0,0,0), return at the center of (3,2,0).
  // Face crossings: x at t=1/6, y at 1/4, x at 1/2, y at 3/4, x at 5/6.
  OccupancyGrid grid(OccupancyGridParams{1.0, 0.1, 20.0});
  grid.integrate({{3.f, 2.f, 0.f}}, translation(0.5, 0.5, 0.5));
  expectExact(grid.voxels(),
              {{{0, 0, 0}, State::Free},
               {{1, 0, 0}, State::Free},
               {{1, 1, 0}, State::Free},
               {{2, 1, 0}, State::Free},
               {{2, 2, 0}, State::Free},
               {{3, 2, 0}, State::Occupied}},
              "diagonal");
  expectConsistent(grid, "diagonal");
}

void testNegativeIndices() {
  OccupancyGrid grid(OccupancyGridParams{1.0, 0.1, 20.0});
  // Origin (-1.5, 0.5, 0.5) is voxel (-2,0,0). Sensor point (3,0,0) lands on
  // world (1.5, 0.5, 0.5), voxel (1,0,0).
  grid.integrate({{3.f, 0.f, 0.f}}, translation(-1.5, 0.5, 0.5));
  expectExact(grid.voxels(),
              {{{-2, 0, 0}, State::Free},
               {{-1, 0, 0}, State::Free},
               {{0, 0, 0}, State::Free},
               {{1, 0, 0}, State::Occupied}},
              "negative");
  expectConsistent(grid, "negative");
  if (!grid.bounds().any || !(grid.bounds().min_x < 0.0)) fail("negative: bounds");
}

double distanceToSegment(const std::array<double, 3>& origin, const std::array<double, 3>& hit,
                         const std::array<double, 3>& point) {
  const double ab[3] = {hit[0] - origin[0], hit[1] - origin[1], hit[2] - origin[2]};
  const double ap[3] = {point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
  const double ab2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
  double t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / ab2;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  const double d[3] = {origin[0] + ab[0] * t - point[0], origin[1] + ab[1] * t - point[1],
                       origin[2] + ab[2] * t - point[2]};
  return std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

void testSkewRayStaysOnSegment() {
  OccupancyGridParams params{0.5, 0.1, 30.0};
  OccupancyGrid grid(params);
  const std::array<float, 3> sensor_point{2.2f, -1.4f, 1.7f};
  const auto world_from_sensor = translation(0.5, 0.25, -0.75);
  grid.integrate({sensor_point}, world_from_sensor);
  const auto origin = transformPoint(world_from_sensor, 0, 0, 0);
  const auto hit = transformPoint(world_from_sensor, sensor_point[0], sensor_point[1], sensor_point[2]);
  if (grid.count(State::Occupied) != 1) fail("skew: occupied count");
  if (grid.count(State::Free) == 0) fail("skew: no free voxels");
  const double limit = params.resolution_m * std::sqrt(3.0) * 0.5 + 1e-6;
  for (const auto& [voxel, state] : grid.voxels()) {
    const std::array<double, 3> center{
        (static_cast<double>(voxel.x) + 0.5) * params.resolution_m,
        (static_cast<double>(voxel.y) + 0.5) * params.resolution_m,
        (static_cast<double>(voxel.z) + 0.5) * params.resolution_m};
    if (distanceToSegment(origin, hit, center) > limit) {
      fail("skew: voxel far from the ray");
      break;
    }
    if (state == State::Occupied) {
      const double inv = 1.0 / params.resolution_m;
      const Voxel end{static_cast<int>(std::floor(hit[0] * inv)),
                      static_cast<int>(std::floor(hit[1] * inv)),
                      static_cast<int>(std::floor(hit[2] * inv))};
      if (!(voxel == end)) fail("skew: occupied voxel is not the endpoint");
    }
  }
  expectConsistent(grid, "skew");
}

void testBadGridParams() {
  bool threw = false;
  try {
    OccupancyGrid grid(OccupancyGridParams{0.0, 0.0, 1.0});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  if (!threw) fail("resolution 0 should throw");
  threw = false;
  try {
    OccupancyGrid grid(OccupancyGridParams{0.5, 5.0, 1.0});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  if (!threw) fail("max_range < min_range should throw");
  threw = false;
  try {
    BoundaryMap map;
    compareWithDense(map, {}, CompareOptions{-1, 10, 1});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  if (!threw) fail("negative padding should throw");
}

void testWindowFallbackStillChecksStoredCells() {
  VoxelGrid grid;
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      for (int z = 0; z < 3; ++z) grid[{x, y, z}] = State::Free;
    }
  }
  BoundaryMap map;
  map.rebuild(grid);
  CompareOptions options;
  options.padding = 1;
  options.max_window_voxels = 10;
  const ConsistencyReport skipped = compareWithDense(map, grid, options);
  if (!skipped.window_skipped) fail("fallback: window was not skipped");
  if (skipped.used_bounding_window) fail("fallback: used the window");
  if (skipped.mismatched != 0) fail("fallback: stored cells mismatched");
  if (map.query({1, 1, 1}) != State::Free) fail("fallback: interior cell is not free");

  options.max_window_voxels = 100000;
  const ConsistencyReport full = compareWithDense(map, grid, options);
  if (!full.used_bounding_window || full.mismatched != 0) fail("fallback: full window mismatch");
}

void testPoseTimesTrOrder() {
  // 90 degree yaw: velodyne +x becomes camera +y. Pose then translates.
  const std::array<double, 12> tr = {0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0};
  const std::array<double, 12> pose = translation(1.5, 0.5, 0.5);
  const auto world_from_velo = velodynePoseToWorld(pose, tr);
  const auto hit = transformPoint(world_from_velo, 4.0, 0.0, 0.0);
  if (!near(hit[0], 1.5) || !near(hit[1], 4.5) || !near(hit[2], 0.5)) {
    fail("pose*Tr point is in the wrong place");
  }
  // The swapped product Tr*pose lands elsewhere. Guard the multiply order.
  const auto swapped = transformPoint(composeRigid(tr, pose), 4.0, 0.0, 0.0);
  if (near(swapped[0], hit[0]) && near(swapped[1], hit[1])) fail("pose*Tr matched Tr*pose");

  OccupancyGrid grid(OccupancyGridParams{1.0, 0.1, 20.0});
  grid.integrate({{4.f, 0.f, 0.f}}, world_from_velo);
  expectState(grid.voxels(), {1, 4, 0}, State::Occupied, "pose*Tr");
  expectState(grid.voxels(), {1, 0, 0}, State::Free, "pose*Tr-origin");
  expectConsistent(grid, "pose*Tr");
}

void testLoaderRoundTripAndFilters() {
  TempDir dir;
  const std::filesystem::path root = dir.path();
  writeText(root / "sequences" / "02" / "calib.txt",
            "P0: 1 0 0 0 0 1 0 0 0 0 1 0\n"
            "Tr: 1 0 0 0.25 0 1 0 0 0 0 1 0\n");
  writeText(root / "poses" / "02.txt",
            "1 0 0 0 0 1 0 0 0 0 1 0\n"
            "1 0 0 1 0 1 0 0 0 0 1 0\n"
            "1 0 0 2 0 1 0 0 0 0 1 0\n"
            "1 0 0 3 0 1 0 0 0 0 1 0\n");
  writeBin(root / "sequences" / "02" / "velodyne" / "000000.bin", {{1.f, 2.f, 3.f}});
  writeBin(root / "sequences" / "02" / "velodyne" / "000001.bin", {{-4.f, 0.5f, 0.25f}});
  writeBin(root / "sequences" / "02" / "velodyne" / "000002.bin", {{8.f, 0.f, 0.f}});
  writeBin(root / "sequences" / "02" / "velodyne" / "000003.bin", {{}});

  const auto points = loadVelodyneBin(root / "sequences" / "02" / "velodyne" / "000001.bin");
  if (points.size() != 1 || !near(points[0][0], -4.0) || !near(points[0][1], 0.5) ||
      !near(points[0][2], 0.25)) {
    fail("bin round trip");
  }

  KittiLoadOptions options;
  options.stride = 2;
  options.max_frames = 10;
  const KittiSequence sequence = loadKittiSequence(root, "2", options);
  if (sequence.sequence != "02") fail("sequence normalize");
  if (sequence.scans_on_disk != 4) fail("scans on disk");
  if (sequence.scans.size() != 2 || sequence.scans[0].index != 0 || sequence.scans[1].index != 2) {
    fail("stride filter");
  }
  if (!near(sequence.tr_cam_from_velo[3], 0.25)) fail("Tr translation");

  options.stride = 1;
  options.max_frames = 1;
  const KittiSequence first = loadKittiSequence(root, "02", options);
  if (first.scans.size() != 1 || first.scans[0].index != 0) fail("max_frames");

  writeText(root / "sequences" / "02" / "velodyne" / "bad.bin", "abcd");
  // bad.bin is not a numeric stem; listing throws before the stride filter.
  bool threw = false;
  try {
    loadKittiSequence(root, "02", {});
  } catch (const std::runtime_error&) {
    threw = true;
  }
  if (!threw) fail("non-numeric bin name should throw");
  std::filesystem::remove(root / "sequences" / "02" / "velodyne" / "bad.bin");

  writeBin(root / "sequences" / "02" / "velodyne" / "000009.bin", {{1.f, 0.f, 0.f}});
  threw = false;
  try {
    loadKittiSequence(root, "02", {});
  } catch (const std::runtime_error&) {
    threw = true;
  }
  if (!threw) fail("frame past the pose file should throw");
  std::filesystem::remove(root / "sequences" / "02" / "velodyne" / "000009.bin");

  std::ofstream truncated(root / "sequences" / "02" / "velodyne" / "000000.bin",
                          std::ios::binary | std::ios::trunc);
  const char junk[20] = {};
  truncated.write(junk, sizeof(junk));
  truncated.close();
  threw = false;
  try {
    loadVelodyneBin(root / "sequences" / "02" / "velodyne" / "000000.bin");
  } catch (const std::runtime_error&) {
    threw = true;
  }
  if (!threw) fail("truncated bin should throw");

  TempDir other;
  writeText(other.path() / "sequences" / "00" / "calib.txt", "P0: 1 0 0 0 0 1 0 0 0 0 1 0\n");
  writeText(other.path() / "poses" / "00.txt", "1 0 0 0 0 1 0 0 0 0 1 0\n");
  writeBin(other.path() / "sequences" / "00" / "velodyne" / "000000.bin", {{1.f, 0.f, 0.f}});
  threw = false;
  try {
    loadKittiSequence(other.path(), "00", {});
  } catch (const std::runtime_error&) {
    threw = true;
  }
  if (!threw) fail("missing Tr should throw");

  writeText(other.path() / "sequences" / "01" / "calib.txt",
            "Tr_velo_to_cam: 0 -1 0 0 1 0 0 0 0 0 1 0\n");
  writeText(other.path() / "sequences" / "01" / "poses.txt", "1 0 0 0 0 1 0 0 0 0 1 0\n");
  writeBin(other.path() / "sequences" / "01" / "velodyne" / "000000.bin", {{}});
  const KittiSequence alt = loadKittiSequence(other.path(), "01", {});
  if (!near(alt.tr_cam_from_velo[1], -1.0)) fail("Tr_velo_to_cam was not read");
  if (alt.scans.size() != 1) fail("poses.txt fallback");
}

void testCommittedFixture() {
  const std::filesystem::path root = BDM_FIXTURE_ROOT;
  const KittiSequence sequence = loadKittiSequence(root, "0", {});
  if (sequence.scans.size() != 2 || sequence.scans[0].index != 0 || sequence.scans[1].index != 1) {
    fail("fixture frames");
  }

  auto run = [&](double resolution, bool check_voxels) {
    OccupancyGrid grid(OccupancyGridParams{resolution, 0.1, 30.0});
    for (const KittiScan& scan : sequence.scans) {
      const auto world_from_velo =
          velodynePoseToWorld(sequence.poses[static_cast<std::size_t>(scan.index)],
                              sequence.tr_cam_from_velo);
      grid.integrate(loadVelodyneBin(scan.path), world_from_velo);
    }
    if (grid.count(State::Free) == 0 || grid.count(State::Occupied) == 0) {
      fail("fixture produced an empty class");
    }
    expectConsistent(grid, "fixture");
    if (!check_voxels) return;
    expectState(grid.voxels(), {5, 0, 0}, State::Occupied, "fixture-frame0");
    expectState(grid.voxels(), {6, 0, 0}, State::Occupied, "fixture-frame1");
    expectState(grid.voxels(), {0, 4, 0}, State::Occupied, "fixture-y");
    expectState(grid.voxels(), {0, 0, 3}, State::Occupied, "fixture-z");
    expectState(grid.voxels(), {4, 0, 0}, State::Free, "fixture-free");
    expectState(grid.voxels(), {0, 0, 1}, State::Free, "fixture-vertical-free");
  };
  run(1.0, true);
  run(0.5, false);
}

}  // namespace

int main() {
  testAxisAlignedHitMiss();
  testOccupiedSticky();
  testRangeAndNonFinite();
  testDiagonalRay();
  testNegativeIndices();
  testSkewRayStaysOnSegment();
  testBadGridParams();
  testWindowFallbackStillChecksStoredCells();
  testPoseTimesTrOrder();
  testLoaderRoundTripAndFilters();
  testCommittedFixture();
  if (g_failures != 0) {
    std::cerr << g_failures << " failure(s)\n";
    return 1;
  }
  std::cout << "KITTI dense-grid tests passed\n";
  return 0;
}
