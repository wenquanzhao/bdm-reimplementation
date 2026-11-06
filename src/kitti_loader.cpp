#include "bdm/kitti_loader.hpp"

#include "bdm/rigid.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace bdm {
namespace {

std::string normalizeSequence(std::string sequence) {
  if (sequence.size() == 1 && std::isdigit(static_cast<unsigned char>(sequence[0]))) {
    sequence.insert(sequence.begin(), '0');
  }
  if (sequence.size() != 2 || !std::isdigit(static_cast<unsigned char>(sequence[0])) ||
      !std::isdigit(static_cast<unsigned char>(sequence[1]))) {
    throw std::runtime_error(
        "sequence id must be two digits, for example 00 or 02 (got '" + sequence + "')");
  }
  return sequence;
}

std::string trim(std::string text) {
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) text.pop_back();
  std::size_t start = 0;
  while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start]))) ++start;
  return text.substr(start);
}

std::array<double, 12> parseTwelve(const std::string& line, const std::string& what) {
  std::istringstream in(line);
  std::array<double, 12> values{};
  for (double& value : values) {
    if (!(in >> value)) {
      throw std::runtime_error(what + " expected 12 numbers, got: " + line);
    }
  }
  std::string extra;
  if (in >> extra) {
    throw std::runtime_error(what + " has extra tokens: " + line);
  }
  return values;
}

std::array<double, 12> loadTr(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("missing calibration file: " + path.string() +
                             " (expected sequences/<seq>/calib.txt with a Tr row). "
                             "See docs/KITTI_PIPELINE.md");
  }
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    line = trim(line);
    if (line.empty()) continue;
    std::istringstream row(line);
    std::string token;
    if (!(row >> token)) continue;
    if (!token.empty() && token.back() == ':') token.pop_back();
    if (token != "Tr" && token != "Tr_velo_to_cam") continue;
    std::string rest;
    std::getline(row, rest);
    return parseTwelve(trim(rest), path.string());
  }
  throw std::runtime_error(path.string() +
                           " has no Tr or Tr_velo_to_cam row (velodyne to camera 0)");
}

std::vector<std::array<double, 12>> loadPoses(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("missing poses: " + path.string() +
                             " (expected poses/<seq>.txt, one 3x4 pose per frame). "
                             "See docs/KITTI_PIPELINE.md");
  }
  std::vector<std::array<double, 12>> poses;
  std::string line;
  int line_number = 0;
  while (std::getline(in, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    line = trim(line);
    if (line.empty()) continue;
    poses.push_back(parseTwelve(line, path.string() + ":" + std::to_string(line_number)));
  }
  if (poses.empty()) {
    throw std::runtime_error(path.string() + " contains no poses");
  }
  return poses;
}

int frameIndexFromStem(const std::string& stem) {
  if (stem.empty()) {
    throw std::runtime_error("velodyne filename is not a numeric frame index");
  }
  for (unsigned char c : stem) {
    if (!std::isdigit(c)) {
      throw std::runtime_error("velodyne filename is not a numeric frame index: " + stem);
    }
  }
  try {
    return std::stoi(stem);
  } catch (const std::exception&) {
    throw std::runtime_error("velodyne frame index out of range: " + stem);
  }
}

void requireLittleEndian() {
  const std::uint16_t value = 1;
  unsigned char bytes[2];
  std::memcpy(bytes, &value, sizeof(value));
  if (bytes[0] != 1) {
    throw std::runtime_error("KITTI velodyne loader requires a little-endian host");
  }
}

}  // namespace

std::array<double, 12> velodynePoseToWorld(const std::array<double, 12>& pose_world_from_cam,
                                           const std::array<double, 12>& tr_cam_from_velo) {
  return composeRigid(pose_world_from_cam, tr_cam_from_velo);
}

KittiSequence loadKittiSequence(const std::filesystem::path& root, const std::string& sequence_text,
                                KittiLoadOptions options) {
  if (options.stride < 1) {
    throw std::runtime_error("stride must be >= 1");
  }
  if (options.max_frames < 0) {
    throw std::runtime_error("max_frames must be >= 0");
  }
  KittiSequence sequence;
  sequence.sequence = normalizeSequence(sequence_text);
  const std::filesystem::path seq_dir = root / "sequences" / sequence.sequence;
  const std::filesystem::path calib = seq_dir / "calib.txt";
  sequence.velodyne_dir = seq_dir / "velodyne";
  std::filesystem::path poses = root / "poses" / (sequence.sequence + ".txt");
  const std::filesystem::path poses_alt = seq_dir / "poses.txt";
  if (!std::filesystem::exists(poses) && std::filesystem::exists(poses_alt)) poses = poses_alt;

  sequence.tr_cam_from_velo = loadTr(calib);
  sequence.poses = loadPoses(poses);

  if (!std::filesystem::is_directory(sequence.velodyne_dir)) {
    throw std::runtime_error("missing velodyne directory: " + sequence.velodyne_dir.string() +
                             ". See docs/KITTI_PIPELINE.md");
  }

  std::vector<KittiScan> found;
  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(sequence.velodyne_dir)) {
    if (!entry.is_regular_file()) continue;
    const std::filesystem::path path = entry.path();
    if (path.extension() != ".bin") continue;
    found.push_back({frameIndexFromStem(path.stem().string()), path});
  }
  std::sort(found.begin(), found.end(),
            [](const KittiScan& a, const KittiScan& b) { return a.index < b.index; });
  for (std::size_t i = 1; i < found.size(); ++i) {
    if (found[i].index == found[i - 1].index) {
      throw std::runtime_error("duplicate velodyne frame index " +
                               std::to_string(found[i].index));
    }
  }
  sequence.scans_on_disk = found.size();

  for (const KittiScan& scan : found) {
    if (options.stride > 1 && (scan.index % options.stride) != 0) continue;
    if (scan.index < 0 || static_cast<std::size_t>(scan.index) >= sequence.poses.size()) {
      throw std::runtime_error("pose file has " + std::to_string(sequence.poses.size()) +
                               " poses but velodyne frame " + std::to_string(scan.index) +
                               " is present");
    }
    sequence.scans.push_back(scan);
    if (options.max_frames > 0 &&
        sequence.scans.size() == static_cast<std::size_t>(options.max_frames)) {
      break;
    }
  }
  if (sequence.scans.empty()) {
    throw std::runtime_error("no velodyne frames selected in " + sequence.velodyne_dir.string());
  }
  return sequence;
}

std::vector<std::array<float, 3>> loadVelodyneBin(const std::filesystem::path& path) {
  requireLittleEndian();
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot read velodyne bin: " + path.string());
  }
  in.seekg(0, std::ios::end);
  const std::streamoff bytes = in.tellg();
  if (bytes < 0) {
    throw std::runtime_error("cannot size velodyne bin: " + path.string());
  }
  constexpr std::streamoff kMaxBytes = 64 * 1024 * 1024;
  if (bytes > kMaxBytes) {
    throw std::runtime_error("velodyne bin is larger than 64 MiB: " + path.string());
  }
  if (bytes % 16 != 0) {
    throw std::runtime_error("velodyne bin size is not a multiple of 16 bytes: " + path.string());
  }
  in.seekg(0, std::ios::beg);
  const std::size_t count = static_cast<std::size_t>(bytes / 16);
  std::vector<std::array<float, 3>> points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    float record[4];
    if (!in.read(reinterpret_cast<char*>(record), sizeof(record))) {
      throw std::runtime_error("short read in velodyne bin: " + path.string());
    }
    points.push_back({record[0], record[1], record[2]});
  }
  return points;
}

}  // namespace bdm
