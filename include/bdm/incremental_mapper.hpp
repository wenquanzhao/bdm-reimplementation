#pragma once

#include "bdm/boundary_map.hpp"
#include "bdm/local_grid.hpp"
#include "bdm/log_odds.hpp"

#include <array>
#include <cstddef>
#include <unordered_set>

namespace bdm {

// Knobs for the Section 7 skeleton. Defaults for the odds are the Section 8.2
// probabilities. The window size and the slide threshold are not given as a
// single number in the paper; Table 1 lists per-dataset window sizes.
struct IncrementalMapperParams {
  double resolution_m = 1.0;
  double min_range_m = 0.0;
  double max_range_m = 30.0;
  int size_x = 16;
  int size_y = 16;
  int size_z = 16;
  // Slide when the sensor is farther than this from the window center.
  // Algorithm 2 uses ||x_k - o|| > t, with o the map center (Section 7.1).
  double slide_threshold_m = 1.0;
  LogOddsSpec odds{};
};

struct FrameUpdate {
  LocalIntegrateStats rays;
  bool slid = false;
  // Dirty voxels that left the window and were removed from the difference log.
  std::size_t committed_voxels = 0;
  // Those voxels plus their 6-neighbors: the set U in Algorithm 2.
  std::size_t boundary_touched = 0;
  // Voxels in the slide-in region whose log-odds were set from a global state.
  std::size_t slide_in_fused = 0;
};

// Section 7 skeleton: a sliding log-odds window plus a global BoundaryMap.
//
// integrate() follows Algorithm 2's order. The first pose only centers the
// window. Later poses ray-cast into the current window, then slide if the
// sensor has moved past slide_threshold_m:
//   1. New scan ray casting on the local grid (Section 7.2.1).
//   2. Incremental boundary update of the slide-out region (Section 7.2.2).
//   3. Load and fuse the slide-in region (Section 7.2.3, equation (11)).
//
// query() follows Section 7.3. Inside the window it reads the local grid.
// Outside, it reads the global boundary map along z+ or z- chosen so the
// search does not step into the window. x± and y± are not implemented.
//
// globalMap() is the Algorithm 2 store. It is updated on slides, not on every
// ray. A cell inside the window can therefore disagree with a boundary entry
// left behind from an earlier visit, until that cell slides out again.
// mergedBoundaryMap() rebuilds a snapshot that is consistent with the local
// grid on the current window: global entries, with the window and its
// one-cell shell rewritten from the current states. That snapshot is the
// overlap check. It is not a second online map, and it is not a Section 8 score.
//
// Not implemented: the paper hash and packed entries, projection axes other
// than z, ROS 2, and any memory or timing claim. Rays that miss the current
// window are not written into the global map; the paper sizes the window to
// cover the sensor, and this skeleton does not grow it on the fly.
class IncrementalMapper {
 public:
  explicit IncrementalMapper(IncrementalMapperParams params = {});

  const IncrementalMapperParams& params() const noexcept { return params_; }
  const LocalGrid& local() const noexcept { return local_; }
  const BoundaryMap& globalMap() const noexcept { return global_; }
  std::size_t slideCount() const noexcept { return slide_count_; }

  FrameUpdate integrate(const std::vector<std::array<float, 3>>& points_sensor,
                        const std::array<double, 12>& world_from_sensor);

  State query(const Voxel& voxel) const;
  BoundaryMap mergedBoundaryMap() const;

 private:
  bool inBox(Voxel voxel, Voxel origin) const;
  Voxel centeredOrigin(Voxel sensor_voxel) const;
  double distanceFromCenter(const std::array<double, 3>& sensor_m) const;
  State queryGlobalAvoiding(Voxel voxel, Voxel window_origin) const;
  State combinedState(Voxel voxel) const;
  void slideTo(Voxel new_origin, FrameUpdate& update);
  std::size_t fuseSlideIn(Voxel old_origin, Voxel new_origin);
  std::size_t fuseColumnInterval(int x, int y, int z0, int count, Voxel old_origin);
  void fuseVoxel(Voxel voxel, State state);

  IncrementalMapperParams params_;
  LogOddsModel model_;
  LocalGrid local_;
  BoundaryMap global_;
  std::unordered_set<Voxel, VoxelHash> dirty_;
  std::size_t slide_count_ = 0;
};

}  // namespace bdm
