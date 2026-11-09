// Builtin reference autopilot, a port of hwil/controllers/builtin.py.
//
// Copyright 2026 The open-hwil-testbed authors. Licensed under Apache-2.0.
#pragma once

#include <array>
#include <cmath>

#include "fdm.hpp"
#include "mission.hpp"

namespace hwil {

struct Pid {
  double kp, ki = 0, kd = 0, i_limit = 1;
  double integ = 0, prev = 0;
  bool has_prev = false;
  double update(double err, double dt);
  void reset() { integ = 0; has_prev = false; }
};

class BuiltinController {
 public:
  explicit BuiltinController(Mission& m);
  std::array<double, 4> update(const State& s, double dt);

 private:
  Mission& mission_;
  Pid vn_, ve_, vd_, rp_, rq_, rr_;
  bool have_yaw_sp_ = false;
  double yaw_sp_ = 0;
};

}  // namespace hwil
