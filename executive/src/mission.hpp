// Mission sequencer, a line-for-line port of hwil/mission.py.
//
// Copyright 2026 The open-hwil-testbed authors. Licensed under Apache-2.0.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "config.hpp"
#include "fdm.hpp"

namespace hwil {

enum Phase { PREFLIGHT = 0, TAKEOFF = 1, HOLD = 2, LAND = 3, LANDED = 4, GOTO = 5, HOLD_END = 6 };

struct Setpoint {
  Phase phase;
  bool armed;
  Vec3 pos;
  bool has_vel_down;
  double vel_down;
};

class Mission {
 public:
  explicit Mission(std::vector<Item> phases) : phases_(std::move(phases)) {}
  Setpoint update(const State& s);

  Phase phase() const { return phase_; }
  int index() const { return idx_; }
  double alt_target() const { return alt_target_; }
  double north_target() const { return ne_[0]; }
  double east_target() const { return ne_[1]; }
  const std::vector<std::pair<double, std::string>>& events() const { return events_; }

 private:
  void advance(double t);
  Vec3 target() const { return {ne_[0], ne_[1], -alt_target_}; }

  std::vector<Item> phases_;
  int idx_ = -1;
  Phase phase_ = PREFLIGHT;
  double t_phase_ = 0, alt_target_ = 0;
  double ne_[2] = {0, 0};
  Vec3 goto_start_{};
  double goto_speed_ = 3.0, goto_radius_ = 1.0, goto_timeout_ = 90.0;
  bool landed_timer_ = false;
  double landed_since_ = 0;
  std::vector<std::pair<double, std::string>> events_;
};

const char* phase_name(Phase p);

}  // namespace hwil
