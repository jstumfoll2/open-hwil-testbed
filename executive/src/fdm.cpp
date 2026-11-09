// Copyright 2026 The open-hwil-testbed authors. Licensed under Apache-2.0.
#include "fdm.hpp"

#include <FGFDMExec.h>
#include <input_output/FGPropertyManager.h>
#include <simgear/misc/sg_path.hxx>
#include <simgear/props/props.hxx>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hwil {

namespace {
constexpr double FT = 0.3048;
constexpr double LBF = 4.4482216152605;
constexpr double SLUG = 14.593902937;
constexpr double EARTH_R = 6378137.0;
constexpr double DEG = M_PI / 180.0;
const char* MOTOR_INPUTS[4] = {"fcs/cmdFR_ext_nd", "fcs/cmdAL_ext_nd", "fcs/cmdFL_ext_nd", "fcs/cmdAR_ext_nd"};
}  // namespace

Fdm::Fdm(const Config& c) {
  dt_ = 1.0 / c.num("rate_hz");
  home_lat_ = c.num("lat_deg");
  home_lon_ = c.num("lon_deg");

  fdm_ = std::make_unique<JSBSim::FGFDMExec>();
  fdm_->SetDebugLevel(0);
  fdm_->SetRootDir(SGPath(c.str("jsbsim_root")));
  fdm_->SetAircraftPath(SGPath(c.str("aircraft_path")));
  fdm_->SetEnginePath(SGPath("engine"));
  fdm_->SetSystemsPath(SGPath("systems"));
  if (!fdm_->LoadModel(c.str("model"))) throw std::runtime_error("JSBSim could not load model " + c.str("model"));
  fdm_->Setdt(dt_);

  // Same order as hwil/fdm.py: seed, initial conditions, run_ic, then wind.
  auto set = [&](const char* p, double v) { fdm_->SetPropertyValue(p, v); };
  set("simulation/randomseed", static_cast<double>(static_cast<long>(c.num("seed")) & 0x7FFFFFFF));
  set("ic/lat-geod-deg", home_lat_);
  set("ic/long-gc-deg", home_lon_);
  set("ic/terrain-elevation-ft", c.num("ground_alt_msl_m") / FT);
  set("ic/h-agl-ft", 0.5);
  set("ic/psi-true-deg", c.num("heading_deg"));
  set("ic/u-fps", 0.0);
  set("ic/v-fps", 0.0);
  set("ic/w-fps", 0.0);
  if (!fdm_->RunIC()) throw std::runtime_error("JSBSim RunIC failed");
  const double wn = c.num("wind_north_mps"), we = c.num("wind_east_mps");
  set("atmosphere/wind-north-fps", wn / FT);
  set("atmosphere/wind-east-fps", we / FT);
  set("atmosphere/wind-down-fps", c.num("wind_down_mps") / FT);
  const int sev = static_cast<int>(c.num("turbulence_severity"));
  if (sev > 0) {
    set("atmosphere/turb-type", 3);
    set("atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps", std::hypot(wn, we) / FT);
    set("atmosphere/turbulence/milspec/severity", sev);
  }
  for (int e = 0; e < 4; ++e) set(("propulsion/engine[" + std::to_string(e) + "]/set-running").c_str(), 1);

  lat_ = node("position/lat-geod-deg");
  lon_ = node("position/long-gc-deg");
  hsl_ = node("position/h-sl-meters");
  hagl_ = node("position/h-agl-ft");
  vn_ = node("velocities/v-north-fps");
  ve_ = node("velocities/v-east-fps");
  vd_ = node("velocities/v-down-fps");
  phi_ = node("attitude/phi-rad");
  theta_ = node("attitude/theta-rad");
  psi_ = node("attitude/psi-rad");
  p_ = node("velocities/p-rad_sec");
  q_ = node("velocities/q-rad_sec");
  r_ = node("velocities/r-rad_sec");
  fbx_ = node("forces/fbx-total-lbs");
  fby_ = node("forces/fby-total-lbs");
  fbz_ = node("forces/fbz-total-lbs");
  mass_ = node("inertia/mass-slugs");
  wn_ = node("atmosphere/total-wind-north-fps");
  we_ = node("atmosphere/total-wind-east-fps");
  wd_ = node("atmosphere/total-wind-down-fps");
  t_ = node("simulation/sim-time-sec");
  for (int i = 0; i < 4; ++i) esc_[i] = node(MOTOR_INPUTS[i]);
  home_alt_msl_ = get(hsl_);
}

Fdm::~Fdm() = default;

Fdm::Node* Fdm::node(const std::string& name) const {
  auto* n = fdm_->GetPropertyManager()->GetNode(name);
  if (!n) throw std::runtime_error("JSBSim property not found: " + name);
  return n;
}

double Fdm::get(Node* n) const { return n->getDoubleValue(); }

void Fdm::set_motors(const std::array<double, 4>& m) {
  for (int i = 0; i < 4; ++i) {
    motors_[i] = std::clamp(m[i], 0.0, 1.0);
    esc_[i]->setDoubleValue(motors_[i]);
  }
}

void Fdm::step() {
  if (!fdm_->Run()) throw std::runtime_error("JSBSim Run() returned false");
}

State Fdm::state() const {
  State s;
  s.t = get(t_);
  s.lat = get(lat_);
  s.lon = get(lon_);
  s.alt_msl = get(hsl_);
  s.alt_agl = get(hagl_) * FT;
  s.pos_ned = {(s.lat - home_lat_) * DEG * EARTH_R,
               (s.lon - home_lon_) * DEG * EARTH_R * std::cos(home_lat_ * DEG),
               -(s.alt_msl - home_alt_msl_)};
  s.vel_ned = {get(vn_) * FT, get(ve_) * FT, get(vd_) * FT};
  s.euler = {get(phi_), get(theta_), get(psi_)};
  s.rates = {get(p_), get(q_), get(r_)};
  const double m = get(mass_) * SLUG;
  s.accel = {get(fbx_) * LBF / m, get(fby_) * LBF / m, get(fbz_) * LBF / m};
  s.wind = {get(wn_) * FT, get(we_) * FT, get(wd_) * FT};
  return s;
}

}  // namespace hwil
