#include "bdm/incremental_mapper.hpp"
#include "bdm/kitti_loader.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct Options {
  bool help = false;
  bool synthetic = false;
  bool kitti = false;
  std::string root;
  std::string sequence = "00";
  int max_frames = 5;
  int stride = 1;
  int point_stride = 1;
  int size_x = 16;
  int size_y = 16;
  int size_z = 16;
  int max_examples = 8;
  double resolution = 1.0;
  double min_range = 0.0;
  double max_range = 30.0;
  double slide_threshold = 1.0;
  bool strong_miss = false;
};

void printUsage(std::ostream& out) {
  out << "Usage: bdm_incremental [--synthetic | --root <dataset>] [options]\n"
         "\n"
         "Stream posed scans into the Section 7 skeleton: a sliding log-odds\n"
         "local grid, ray casting with clamping, and an incremental BoundaryMap\n"
         "update when the window slides. The overlap check compares a merged\n"
         "boundary snapshot with the local grid. It is not a Section 8 metric.\n"
         "\n"
         "With no mode flag, --synthetic is used. That scene sets p_miss so one\n"
         "miss becomes free. --root uses the Section 8.2 probabilities\n"
         "(p_hit=0.8, p_miss=0.48, p_min=0.05, p_max=0.97, p_free=0.2, p_occ=0.8)\n"
         "and reads a KITTI odometry layout. It does not download KITTI.\n"
         "\n"
         "Options:\n"
         "  --synthetic               Built-in two-pose scene\n"
         "  --root PATH               KITTI root containing sequences/ and poses/\n"
         "  --sequence NN             Sequence id (default 00)\n"
         "  --max-frames N            KITTI frames to integrate (default 5, 0 means all)\n"
         "  --stride N                Keep KITTI frame indices divisible by N\n"
         "  --point-stride N          Keep every Nth KITTI point (default 1)\n"
         "  --resolution M            Voxel size in meters (default 1)\n"
         "  --min-range M             Ignore closer returns (default 0)\n"
         "  --max-range M             Ignore farther returns (default 30)\n"
         "  --size N                  Cubic local window in voxels (default 16)\n"
         "  --size-x N --size-y N --size-z N\n"
         "                            Window size per axis, after --size if both are set\n"
         "  --slide-threshold M       Slide past this distance from the window center\n"
         "                            (default 1 meter)\n"
         "  --max-examples N          Overlap mismatches to print (default 8)\n"
         "  --help                    Show this text\n"
         "\n"
         "Exit status: 0 overlap matched, 1 one or more mismatches,\n"
         "2 usage or I/O error. See docs/INCREMENTAL.md.\n";
}

std::string needValue(int& index, int argc, char** argv, const char* flag) {
  if (index + 1 >= argc) throw std::runtime_error(std::string(flag) + " needs a value");
  ++index;
  return argv[index];
}

int parseInt(const std::string& text, const char* flag) {
  try {
    std::size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != text.size()) throw std::runtime_error(std::string(flag) + " is not an integer");
    return value;
  } catch (const std::runtime_error&) {
    throw;
  } catch (const std::exception&) {
    throw std::runtime_error(std::string(flag) + " is not an integer");
  }
}

double parseDouble(const std::string& text, const char* flag) {
  try {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size()) throw std::runtime_error(std::string(flag) + " is not a number");
    return value;
  } catch (const std::runtime_error&) {
    throw;
  } catch (const std::exception&) {
    throw std::runtime_error(std::string(flag) + " is not a number");
  }
}

Options parseArgs(int argc, char** argv) {
  Options options;
  bool size_set = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      options.help = true;
    } else if (arg == "--synthetic") {
      options.synthetic = true;
    } else if (arg == "--root") {
      options.root = needValue(i, argc, argv, "--root");
      options.kitti = true;
    } else if (arg == "--sequence") {
      options.sequence = needValue(i, argc, argv, "--sequence");
    } else if (arg == "--max-frames") {
      options.max_frames = parseInt(needValue(i, argc, argv, "--max-frames"), "--max-frames");
    } else if (arg == "--stride") {
      options.stride = parseInt(needValue(i, argc, argv, "--stride"), "--stride");
    } else if (arg == "--point-stride") {
      options.point_stride = parseInt(needValue(i, argc, argv, "--point-stride"), "--point-stride");
    } else if (arg == "--resolution") {
      options.resolution = parseDouble(needValue(i, argc, argv, "--resolution"), "--resolution");
    } else if (arg == "--min-range") {
      options.min_range = parseDouble(needValue(i, argc, argv, "--min-range"), "--min-range");
    } else if (arg == "--max-range") {
      options.max_range = parseDouble(needValue(i, argc, argv, "--max-range"), "--max-range");
    } else if (arg == "--size") {
      const int size = parseInt(needValue(i, argc, argv, "--size"), "--size");
      options.size_x = options.size_y = options.size_z = size;
      size_set = true;
    } else if (arg == "--size-x") {
      options.size_x = parseInt(needValue(i, argc, argv, "--size-x"), "--size-x");
      size_set = true;
    } else if (arg == "--size-y") {
      options.size_y = parseInt(needValue(i, argc, argv, "--size-y"), "--size-y");
      size_set = true;
    } else if (arg == "--size-z") {
      options.size_z = parseInt(needValue(i, argc, argv, "--size-z"), "--size-z");
      size_set = true;
    } else if (arg == "--slide-threshold") {
      options.slide_threshold =
          parseDouble(needValue(i, argc, argv, "--slide-threshold"), "--slide-threshold");
    } else if (arg == "--max-examples") {
      options.max_examples = parseInt(needValue(i, argc, argv, "--max-examples"), "--max-examples");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (options.synthetic && options.kitti) {
    throw std::runtime_error("use either --synthetic or --root");
  }
  if (!options.help && !options.synthetic && !options.kitti) options.synthetic = true;
  if (options.synthetic) {
    options.strong_miss = true;
    if (!size_set) {
      options.size_x = options.size_y = options.size_z = 7;
    }
  }
  if (options.point_stride < 1) throw std::runtime_error("--point-stride must be >= 1");
  if (options.stride < 1) throw std::runtime_error("--stride must be >= 1");
  if (options.max_frames < 0) throw std::runtime_error("--max-frames must be >= 0");
  if (options.max_examples < 0) throw std::runtime_error("--max-examples must be >= 0");
  return options;
}

const char* stateName(bdm::State state) {
  switch (state) {
    case bdm::State::Unknown: return "unknown";
    case bdm::State::Free: return "free";
    case bdm::State::Occupied: return "occupied";
  }
  return "invalid";
}

std::size_t countOverlapMismatches(const bdm::IncrementalMapper& mapper, int max_examples,
                                   std::size_t& queries) {
  queries = 0;
  if (!mapper.local().placed()) return 0;
  const bdm::BoundaryMap merged = mapper.mergedBoundaryMap();
  const bdm::Voxel origin = mapper.local().origin();
  std::size_t mismatched = 0;
  for (int dx = 0; dx < mapper.local().sizeX(); ++dx) {
    for (int dy = 0; dy < mapper.local().sizeY(); ++dy) {
      for (int dz = 0; dz < mapper.local().sizeZ(); ++dz) {
        const bdm::Voxel voxel{origin.x + dx, origin.y + dy, origin.z + dz};
        const bdm::State expected = mapper.local().occupancy(voxel);
        const bdm::State positive = merged.query(voxel, bdm::SearchDirection::PositiveZ);
        const bdm::State negative = merged.query(voxel, bdm::SearchDirection::NegativeZ);
        ++queries;
        if (positive == expected && negative == expected) continue;
        ++mismatched;
        if (mismatched <= static_cast<std::size_t>(max_examples)) {
          std::cout << "mismatch: x=" << voxel.x << " y=" << voxel.y << " z=" << voxel.z
                    << " local=" << stateName(expected) << " z+=" << stateName(positive)
                    << " z-=" << stateName(negative) << "\n";
        }
      }
    }
  }
  return mismatched;
}

std::vector<std::array<float, 3>> thinPoints(const std::vector<std::array<float, 3>>& points,
                                            int stride) {
  if (stride <= 1) return points;
  std::vector<std::array<float, 3>> kept;
  for (std::size_t i = 0; i < points.size(); i += static_cast<std::size_t>(stride)) {
    kept.push_back(points[i]);
  }
  return kept;
}

bdm::IncrementalMapperParams makeParams(const Options& options) {
  bdm::IncrementalMapperParams params;
  params.resolution_m = options.resolution;
  params.min_range_m = options.min_range;
  params.max_range_m = options.max_range;
  params.size_x = options.size_x;
  params.size_y = options.size_y;
  params.size_z = options.size_z;
  params.slide_threshold_m = options.slide_threshold;
  if (options.strong_miss) params.odds.p_miss = 0.2;
  return params;
}

struct RunTotals {
  int frames = 0;
  int slides = 0;
  std::size_t points_in = 0;
  std::size_t points_integrated = 0;
  std::size_t points_rejected_range = 0;
  std::size_t points_rejected_nonfinite = 0;
  std::size_t points_endpoint_outside = 0;
};

void integrateFrame(bdm::IncrementalMapper& mapper, const std::vector<std::array<float, 3>>& points,
                    const std::array<double, 12>& pose, RunTotals& totals) {
  const bdm::FrameUpdate update = mapper.integrate(points, pose);
  ++totals.frames;
  if (update.slid) ++totals.slides;
  totals.points_in += update.rays.points_in;
  totals.points_integrated += update.rays.points_integrated;
  totals.points_rejected_range += update.rays.points_rejected_range;
  totals.points_rejected_nonfinite += update.rays.points_rejected_nonfinite;
  totals.points_endpoint_outside += update.rays.points_endpoint_outside;
}

RunTotals runSynthetic(bdm::IncrementalMapper& mapper) {
  RunTotals totals;
  const std::array<double, 12> pose_a = {1, 0, 0, 0.5, 0, 1, 0, 0.5, 0, 0, 1, 0.5};
  const std::array<double, 12> pose_b = {1, 0, 0, 3.5, 0, 1, 0, 0.5, 0, 0, 1, 0.5};
  integrateFrame(mapper, {{-3.f, 0.f, 0.f}, {-3.f, 0.f, 0.f}, {2.f, 0.f, 0.f}}, pose_a, totals);
  integrateFrame(mapper, {{0.f, 0.f, 2.f}}, pose_b, totals);
  return totals;
}

RunTotals runKitti(bdm::IncrementalMapper& mapper, const Options& options) {
  bdm::KittiLoadOptions load;
  load.max_frames = options.max_frames;
  load.stride = options.stride;
  const bdm::KittiSequence sequence = bdm::loadKittiSequence(options.root, options.sequence, load);
  RunTotals totals;
  for (const bdm::KittiScan& scan : sequence.scans) {
    const std::array<double, 12> world_from_velo =
        bdm::velodynePoseToWorld(sequence.poses[static_cast<std::size_t>(scan.index)],
                                 sequence.tr_cam_from_velo);
    const auto loaded = bdm::loadVelodyneBin(scan.path);
    integrateFrame(mapper, thinPoints(loaded, options.point_stride), world_from_velo, totals);
  }
  return totals;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseArgs(argc, argv);
    if (options.help) {
      printUsage(std::cout);
      return 0;
    }
    bdm::IncrementalMapper mapper(makeParams(options));
    const RunTotals totals = options.kitti ? runKitti(mapper, options) : runSynthetic(mapper);
    std::size_t queries = 0;
    const std::size_t mismatched = countOverlapMismatches(mapper, options.max_examples, queries);

    std::cout << "scope: section 7 skeleton; log-odds local grid; sliding window; "
                 "incremental boundary update; not section 8 metrics\n";
    std::cout << "mode: " << (options.kitti ? "kitti" : "synthetic") << "\n";
    if (options.kitti) {
      std::cout << "root: " << options.root << "\n";
      std::cout << "sequence: " << options.sequence << "\n";
      std::cout << "log_odds_preset: section 8.2\n";
    } else {
      std::cout << "log_odds_preset: synthetic; p_miss=0.2 so one miss is free; "
                   "not section 8.2 p_miss=0.48\n";
    }
    std::cout << "frames: " << totals.frames << "\n";
    std::cout << "slides: " << totals.slides << "\n";
    std::cout << "points_read: " << totals.points_in << "\n";
    std::cout << "points_integrated: " << totals.points_integrated << "\n";
    std::cout << "points_rejected_range: " << totals.points_rejected_range << "\n";
    std::cout << "points_rejected_nonfinite: " << totals.points_rejected_nonfinite << "\n";
    std::cout << "points_endpoint_outside: " << totals.points_endpoint_outside << "\n";
    std::cout << "resolution_m: " << options.resolution << "\n";
    std::cout << "window_voxels: " << options.size_x << " " << options.size_y << " "
              << options.size_z << "\n";
    std::cout << "slide_threshold_m: " << options.slide_threshold << "\n";
    std::cout << "global_boundaries: " << mapper.globalMap().boundaryCount() << "\n";
    std::cout << "overlap_queries: " << queries << "\n";
    std::cout << "overlap_mismatched: " << mismatched << "\n";
    const bool committed = totals.slides > 0 && mapper.globalMap().boundaryCount() > 0 &&
                           mismatched == 0;
    std::cout << "incremental_commit: " << (committed ? "yes" : "no") << "\n";
    std::cout << "binary_grid: false\n";
    std::cout << "log_odds: true\n";
    return mismatched == 0 ? 0 : 1;
  } catch (const std::exception& ex) {
    std::cerr << "error: " << ex.what() << "\n";
    return 2;
  }
}
