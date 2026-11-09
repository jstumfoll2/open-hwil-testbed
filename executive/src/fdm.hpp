// JSBSim wrapper: SI units, NED frame, flat-earth position from home.
// Mirrors hwil/fdm.py so both executives log the same quantities.
//
// Copyright 2026 The open-hwil-testbed authors. Licensed under Apache-2.0.
#pragma once

#include <array>
#include <memory>
#include <string>

#include "config.hpp"

namespace JSBSim {
class FGFDMExec;
}  // namespace JSBSim
class SGPropertyNode;

namespace hwil {

using Vec3 = std::array<double, 3>;

struct State {
  double t = 0;
  Vec3 pos_ned{}, vel_ned{}, euler{}, rates{}, accel{}, wind{};
  double lat = 0, lon = 0, alt_msl = 0, alt_agl = 0;
  double alt() const { return -pos_ned[2]; }
};

class Fdm {
 public:
  explicit Fdm(const Config& c);
  ~Fdm();
  void set_motors(const std::array<double, 4>& m);
  void step();
  State state() const;
  double dt() const { return dt_; }
  const std::array<double, 4>& motors() const { return motors_; }

 private:
  using Node = SGPropertyNode;
  Node* node(const std::string& name) const;
  double get(Node* n) const;

  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  double dt_, home_lat_, home_lon_, home_alt_msl_ = 0;
  std::array<double, 4> motors_{};
  // Property nodes looked up once, read every frame.
  Node *lat_, *lon_, *hsl_, *hagl_, *vn_, *ve_, *vd_, *phi_, *theta_, *psi_, *p_, *q_, *r_;
  Node *fbx_, *fby_, *fbz_, *mass_, *wn_, *we_, *wd_, *t_;
  std::array<Node*, 4> esc_{};
};

}  // namespace hwil
