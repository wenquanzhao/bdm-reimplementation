#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace bdm {

struct KittiScan {
  int index = 0;
  std::filesystem::path path;
};

struct KittiLoadOptions {
  // 0 keeps every scan that survives the stride filter.
  int max_frames = 0;
  // Keep scans whose frame index is divisible by stride.
  int stride = 1;
};

// One KITTI odometry sequence on disk:
//   <root>/sequences/<seq>/calib.txt          (Tr: velodyne -> camera 0)
//   <root>/sequences/<seq>/velodyne/NNNNNN.bin
//   <root>/poses/<seq>.txt                    (or sequences/<seq>/poses.txt)
// Pose line i is the 3x4 world-from-camera-0 pose of frame i.
// World is the KITTI odometry frame: camera 0 at frame 0.
struct KittiSequence {
  std::string sequence;
  std::array<double, 12> tr_cam_from_velo{};
  std::vector<std::array<double, 12>> poses;
  std::filesystem::path velodyne_dir;
  std::size_t scans_on_disk = 0;
  std::vector<KittiScan> scans;
};

// pose_world_from_cam * tr_cam_from_velo. Point order matches the KITTI devkit:
// p_world = pose * Tr * p_velo.
std::array<double, 12> velodynePoseToWorld(const std::array<double, 12>& pose_world_from_cam,
                                           const std::array<double, 12>& tr_cam_from_velo);

KittiSequence loadKittiSequence(const std::filesystem::path& root,
                                const std::string& sequence,
                                KittiLoadOptions options = {});

// Little-endian float32 records (x, y, z, reflectance). Reflectance is dropped.
// Throws if the file size is not a multiple of 16 bytes.
std::vector<std::array<float, 3>> loadVelodyneBin(const std::filesystem::path& path);

}  // namespace bdm
