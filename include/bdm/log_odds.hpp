#pragma once

#include "bdm/boundary_map.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace bdm {

// Probabilities from Section 8.2. Log-odds below use the natural log, the
// same base for every threshold, so the base cancels in the comparisons.
// This is the sensor model of the local grid. It is not the binary hit/miss
// grid in OccupancyGrid.
struct LogOddsSpec {
  double p_hit = 0.8;
  double p_miss = 0.48;
  double p_min = 0.05;
  double p_max = 0.97;
  double p_free = 0.2;
  double p_occ = 0.8;
};

// L(n) and the clamps in Section 7.2.1, equations (7)–(10).
class LogOddsModel {
 public:
  explicit LogOddsModel(LogOddsSpec spec = {}) : spec_(spec) {
    l_hit_ = logit(spec_.p_hit, "p_hit");
    l_miss_ = logit(spec_.p_miss, "p_miss");
    l_min_ = logit(spec_.p_min, "p_min");
    l_max_ = logit(spec_.p_max, "p_max");
    l_free_ = logit(spec_.p_free, "p_free");
    l_occ_ = logit(spec_.p_occ, "p_occ");
    if (!(spec_.p_min <= spec_.p_free && spec_.p_free < spec_.p_occ &&
          spec_.p_occ <= spec_.p_max)) {
      throw std::invalid_argument(
          "require p_min <= p_free < p_occ <= p_max so clamping cannot flip a fused state");
    }
    if (!(spec_.p_free < 0.5 && spec_.p_occ > 0.5)) {
      throw std::invalid_argument("p_free must be < 0.5 and p_occ > 0.5 so the prior is unknown");
    }
  }

  const LogOddsSpec& spec() const noexcept { return spec_; }
  double l_hit() const noexcept { return l_hit_; }
  double l_miss() const noexcept { return l_miss_; }
  double l_min() const noexcept { return l_min_; }
  double l_max() const noexcept { return l_max_; }
  double l_free() const noexcept { return l_free_; }
  double l_occ() const noexcept { return l_occ_; }

  double clamp(double log_odds) const noexcept {
    if (log_odds < l_min_) return l_min_;
    if (log_odds > l_max_) return l_max_;
    return log_odds;
  }

  // Equation (10). Prior log-odds 0 is unknown when p_free < 0.5 < p_occ.
  State state(double log_odds) const noexcept {
    if (log_odds <= l_free_) return State::Free;
    if (log_odds >= l_occ_) return State::Occupied;
    return State::Unknown;
  }

 private:
  static double logit(double probability, const char* name) {
    if (!(probability > 0.0 && probability < 1.0) || !std::isfinite(probability)) {
      throw std::invalid_argument(std::string(name) + " must be in (0, 1)");
    }
    return std::log(probability / (1.0 - probability));
  }

  LogOddsSpec spec_{};
  double l_hit_ = 0;
  double l_miss_ = 0;
  double l_min_ = 0;
  double l_max_ = 0;
  double l_free_ = 0;
  double l_occ_ = 0;
};

}  // namespace bdm
