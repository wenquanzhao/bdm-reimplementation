#include "bdm/boundary_map.hpp"
#include "bdm/consistency.hpp"
#include "bdm/kitti_loader.hpp"
#include "bdm/occupancy_grid.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct Options {
  bool help = false;
  std::string root;
  std::string sequence = "00";
  int max_frames = 0;
  int stride = 1;
  int point_stride = 1;
  int padding = 1;
  int max_examples = 20;
  double resolution = 0.5;
  double min_range = 1.0;
  double max_range = 30.0;
  std::uint64_t max_window_voxels = 20000000;
};

void printUsage(std::ostream& out) {
  out << "Usage: bdm_kitti --root <dataset> [options]\n"
         "\n"
         "Accumulate KITTI odometry velodyne scans into a dense Free/Occupied/\n"
         "Unknown grid and compare BoundaryMap rebuild/query (z+ and z-) with it.\n"
         "\n"
         "This is not the paper's Section 7 log-odds mapper and it does not\n"
         "report Section 8 metrics.\n"
         "\n"
         "Options:\n"
         "  --root PATH               Dataset root containing sequences/ and poses/\n"
         "  --sequence NN             Sequence id (default 00)\n"
         "  --max-frames N            Stop after N selected frames; 0 means all on disk\n"
         "  --stride N                Keep frame indices divisible by N (default 1)\n"
         "  --point-stride N          Keep every Nth point in file order (default 1)\n"
         "  --resolution M            Voxel size in meters (default 0.5)\n"
         "  --min-range M             Ignore closer returns, meters (default 1)\n"
         "  --max-range M             Ignore farther returns, meters (default 30)\n"
         "  --padding N               Query this many cells past the grid bbox (default 1)\n"
         "  --max-window-voxels N     If the padded window is larger, compare stored\n"
         "                            cells and boundary entries only (default 20000000)\n"
         "  --max-examples N          Mismatches to print (default 20)\n"
         "  --help                    Show this text\n"
         "\n"
         "Exit status: 0 every query matched, 1 one or more mismatches,\n"
         "2 usage or I/O error. See docs/KITTI_PIPELINE.md.\n";
}

std::string needValue(int& index, int argc, char** argv, const char* flag) {
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string(flag) + " needs a value");
  }
  ++index;
  return argv[index];
}

int parseInt(const std::string& text, const char* flag) {
  try {
    std::size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != text.size()) {
      throw std::runtime_error(std::string(flag) + " is not an integer: " + text);
    }
    return value;
  } catch (const std::runtime_error&) {
    throw;
  } catch (const std::exception&) {
    throw std::runtime_error(std::string(flag) + " is not an integer: " + text);
  }
}

double parseDouble(const std::string& text, const char* flag) {
  try {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size()) {
      throw std::runtime_error(std::string(flag) + " is not a number: " + text);
    }
    return value;
  } catch (const std::runtime_error&) {
    throw;
  } catch (const std::exception&) {
    throw std::runtime_error(std::string(flag) + " is not a number: " + text);
  }
}

std::uint64_t parseUint64(const std::string& text, const char* flag) {
  try {
    std::size_t used = 0;
    const unsigned long long value = std::stoull(text, &used);
    if (used != text.size()) {
      throw std::runtime_error(std::string(flag) + " is not an integer: " + text);
    }
    return static_cast<std::uint64_t>(value);
  } catch (const std::runtime_error&) {
    throw;
  } catch (const std::exception&) {
    throw std::runtime_error(std::string(flag) + " is not an integer: " + text);
  }
}

Options parseArgs(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      options.help = true;
    } else if (arg == "--root") {
      options.root = needValue(i, argc, argv, "--root");
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
    } else if (arg == "--padding") {
      options.padding = parseInt(needValue(i, argc, argv, "--padding"), "--padding");
    } else if (arg == "--max-window-voxels") {
      options.max_window_voxels =
          parseUint64(needValue(i, argc, argv, "--max-window-voxels"), "--max-window-voxels");
    } else if (arg == "--max-examples") {
      options.max_examples = parseInt(needValue(i, argc, argv, "--max-examples"), "--max-examples");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (!options.help && options.root.empty()) {
    throw std::runtime_error("--root is required");
  }
  if (options.point_stride < 1) {
    throw std::runtime_error("--point-stride must be >= 1");
  }
  if (options.max_examples < 0) {
    throw std::runtime_error("--max-examples must be >= 0");
  }
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

std::vector<std::array<float, 3>> thinPoints(const std::vector<std::array<float, 3>>& points,
                                            int stride) {
  if (stride == 1) return points;
  std::vector<std::array<float, 3>> kept;
  kept.reserve((points.size() + static_cast<std::size_t>(stride) - 1) /
               static_cast<std::size_t>(stride));
  for (std::size_t i = 0; i < points.size(); i += static_cast<std::size_t>(stride)) {
    kept.push_back(points[i]);
  }
  return kept;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseArgs(argc, argv);
    if (options.help) {
      printUsage(std::cout);
      return 0;
    }

    bdm::KittiLoadOptions load_options;
    load_options.max_frames = options.max_frames;
    load_options.stride = options.stride;
    const bdm::KittiSequence sequence =
        bdm::loadKittiSequence(options.root, options.sequence, load_options);

    bdm::OccupancyGridParams grid_params;
    grid_params.resolution_m = options.resolution;
    grid_params.min_range_m = options.min_range;
    grid_params.max_range_m = options.max_range;
    bdm::OccupancyGrid grid(grid_params);

    bdm::IntegrateStats total;
    for (std::size_t n = 0; n < sequence.scans.size(); ++n) {
      const bdm::KittiScan& scan = sequence.scans[n];
      const auto& pose = sequence.poses[static_cast<std::size_t>(scan.index)];
      const std::array<double, 12> world_from_velo =
          bdm::velodynePoseToWorld(pose, sequence.tr_cam_from_velo);
      const auto loaded = bdm::loadVelodyneBin(scan.path);
      const bdm::IntegrateStats stats =
          options.point_stride == 1
              ? grid.integrate(loaded, world_from_velo)
              : grid.integrate(thinPoints(loaded, options.point_stride), world_from_velo);
      total.points_in += stats.points_in;
      total.points_integrated += stats.points_integrated;
      total.points_rejected_range += stats.points_rejected_range;
      total.points_rejected_nonfinite += stats.points_rejected_nonfinite;
      if (n == 0 || n + 1 == sequence.scans.size() || (n + 1) % 25 == 0) {
        std::cerr << "integrated frame " << scan.index << " (" << (n + 1) << "/"
                  << sequence.scans.size() << ")\n";
      }
    }

    bdm::BoundaryMap map;
    map.rebuild(grid.voxels());
    std::size_t interior_free = 0;
    std::size_t exterior_unknown = 0;
    std::size_t occupied_boundaries = 0;
    for (const auto& [column, entries] : map.columns()) {
      (void)column;
      for (const bdm::Entry& entry : entries) {
        if (entry.boundary == bdm::Boundary::InteriorFree) ++interior_free;
        else if (entry.boundary == bdm::Boundary::ExteriorUnknown) ++exterior_unknown;
        else ++occupied_boundaries;
      }
    }

    std::cerr << "rebuilt " << map.boundaryCount() << " boundary entries, comparing\n";
    bdm::CompareOptions compare_options;
    compare_options.padding = options.padding;
    compare_options.max_window_voxels = options.max_window_voxels;
    compare_options.max_examples = static_cast<std::size_t>(options.max_examples);
    const bdm::ConsistencyReport report =
        bdm::compareWithDense(map, grid.voxels(), compare_options);

    std::cout << "scope: dense-grid consistency; not section 7 log-odds; not section 8 metrics\n";
    std::cout << "sequence: " << sequence.sequence << "\n";
    std::cout << "root: " << options.root << "\n";
    std::cout << "scans_on_disk: " << sequence.scans_on_disk << "\n";
    std::cout << "frames_integrated: " << sequence.scans.size() << "\n";
    std::cout << "first_frame: " << sequence.scans.front().index << "\n";
    std::cout << "last_frame: " << sequence.scans.back().index << "\n";
    std::cout << "points_read: " << total.points_in << "\n";
    std::cout << "points_integrated: " << total.points_integrated << "\n";
    std::cout << "points_rejected_range: " << total.points_rejected_range << "\n";
    std::cout << "points_rejected_nonfinite: " << total.points_rejected_nonfinite << "\n";
    std::cout << "resolution_m: " << options.resolution << "\n";
    std::cout << "min_range_m: " << options.min_range << "\n";
    std::cout << "max_range_m: " << options.max_range << "\n";
    std::cout << "point_stride: " << options.point_stride << "\n";
    std::cout << "hit_policy: in-range endpoint voxel is occupied\n";
    std::cout << "miss_policy: other voxels crossed by that ray are free unless already occupied\n";
    std::cout << "range_policy: points nearer than min_range or farther than max_range are ignored\n";
    std::cout << "occupied_sticky: true\n";
    std::cout << "log_odds: false\n";
    std::cout << "world_frame: kitti odometry camera 0 at frame 0 (x right, y down, z forward)\n";
    const bdm::WorldBounds& bounds = grid.bounds();
    if (bounds.any) {
      std::cout.setf(std::ios::fixed);
      std::cout.precision(3);
      std::cout << "world_min_m: " << bounds.min_x << " " << bounds.min_y << " " << bounds.min_z
                << "\n";
      std::cout << "world_max_m: " << bounds.max_x << " " << bounds.max_y << " " << bounds.max_z
                << "\n";
      std::cout.unsetf(std::ios::fixed);
      std::cout.precision(6);
    } else {
      std::cout << "world_min_m: none\n";
      std::cout << "world_max_m: none\n";
    }
    std::cout << "free_voxels: " << grid.count(bdm::State::Free) << "\n";
    std::cout << "occupied_voxels: " << grid.count(bdm::State::Occupied) << "\n";
    std::cout << "boundary_entries: " << map.boundaryCount() << "\n";
    std::cout << "boundary_interior_free: " << interior_free << "\n";
    std::cout << "boundary_exterior_unknown: " << exterior_unknown << "\n";
    std::cout << "boundary_occupied: " << occupied_boundaries << "\n";
    std::cout << "compare_scope: "
              << (report.used_bounding_window ? "bounding_window" : "stored_and_boundary") << "\n";
    std::cout << "window_voxels: " << report.window_voxels << "\n";
    std::cout << "window_skipped: " << (report.window_skipped ? "true" : "false") << "\n";
    std::cout << "queries: " << report.queries << "\n";
    std::cout << "matched: " << report.matched << "\n";
    std::cout << "mismatched: " << report.mismatched << "\n";
    if (report.queries == 0 || report.mismatched == 0) {
      std::cout << "match_rate: 1\n";
    } else {
      std::cout.setf(std::ios::fixed);
      std::cout.precision(6);
      std::cout << "match_rate: "
                << (static_cast<double>(report.matched) / static_cast<double>(report.queries))
                << "\n";
      std::cout.unsetf(std::ios::fixed);
      std::cout.precision(6);
    }
    for (const bdm::QueryMismatch& mismatch : report.examples) {
      std::cout << "mismatch: x=" << mismatch.voxel.x << " y=" << mismatch.voxel.y
                << " z=" << mismatch.voxel.z << " dense=" << stateName(mismatch.expected)
                << " z+=" << stateName(mismatch.positive_z)
                << " z-=" << stateName(mismatch.negative_z) << "\n";
    }
    if (report.mismatched > report.examples.size()) {
      std::cout << "mismatches_shown: " << report.examples.size() << "\n";
    }
    return report.mismatched == 0 ? 0 : 1;
  } catch (const std::exception& ex) {
    std::cerr << "error: " << ex.what() << "\n";
    return 2;
  }
}
