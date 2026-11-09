// Copyright 2026 The open-hwil-testbed authors. Licensed under Apache-2.0.
#include "mission.hpp"

#include <algorithm>
#include <cmath>

namespace hwil {

namespace {
constexpr double TAKEOFF_CLIMB_MPS = 1.5;
constexpr double LAND_DESCENT_MPS = 0.7;
constexpr double PREFLIGHT_S = 1.0;
constexpr double LANDED_CONFIRM_S = 1.0;

double norm(const Vec3& a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
}  // namespace

const char* phase_name(Phase p) {
  switch (p) {
    case PREFLIGHT: return "preflight";
    case TAKEOFF: return "takeoff";
    case HOLD: return "hold";
    case LAND: return "land";
    case LANDED: return "landed";
    case GOTO: return "goto";
    case HOLD_END: return "hold_end";
  }
  return "?";
}

void Mission::advance(double t) {
  ++idx_;
  if (idx_ >= static_cast<int>(phases_.size())) {
    phase_ = (phase_ == LANDED) ? LANDED : HOLD_END;
    return;
  }
  const Item& ph = phases_[idx_];
  t_phase_ = t;
  if (ph.type == "takeoff") {
    phase_ = TAKEOFF;
    alt_target_ = ph.get("alt_m", 0);
  } else if (ph.type == "goto") {
    phase_ = GOTO;
    goto_start_ = target();
    ne_[0] = ph.get("north_m", ne_[0]);
    ne_[1] = ph.get("east_m", ne_[1]);
    alt_target_ = ph.get("alt_m", alt_target_);
    goto_speed_ = ph.get("speed_mps", 3.0);
    goto_radius_ = ph.get("accept_radius_m", 1.0);
    goto_timeout_ = ph.get("timeout_s", 90.0);
  } else if (ph.type == "hold") {
    phase_ = HOLD;
  } else {
    phase_ = LAND;
  }
  events_.emplace_back(t, phase_name(phase_));
}

Setpoint Mission::update(const State& s) {
  const double t = s.t;
  if (phase_ == PREFLIGHT && t >= PREFLIGHT_S) {
    advance(t);
  } else if (phase_ == TAKEOFF) {
    if (std::fabs(s.alt() - alt_target_) < 0.5 && std::fabs(s.vel_ned[2]) < 0.5) advance(t);
  } else if (phase_ == GOTO) {
    const Vec3 tg = target();
    const Vec3 err{s.pos_ned[0] - tg[0], s.pos_ned[1] - tg[1], s.pos_ned[2] - tg[2]};
    if (norm(err) < goto_radius_ && norm(s.vel_ned) < 0.5) {
      advance(t);
    } else if (t - t_phase_ >= goto_timeout_) {
      events_.emplace_back(t, "goto_timeout");
      advance(t);
    }
  } else if (phase_ == HOLD) {
    if (t - t_phase_ >= phases_[idx_].get("duration_s", 0)) advance(t);
  } else if (phase_ == LAND) {
    if (s.alt_agl < 0.25 && std::fabs(s.vel_ned[2]) < 0.2) {
      if (!landed_timer_) {
        landed_timer_ = true;
        landed_since_ = t;
      } else if (t - landed_since_ >= LANDED_CONFIRM_S) {
        phase_ = LANDED;
        events_.emplace_back(t, "landed");
      }
    } else {
      landed_timer_ = false;
    }
  }

  const Vec3 tg = target();
  switch (phase_) {
    case PREFLIGHT:
      return {PREFLIGHT, false, {0, 0, 0}, false, 0};
    case TAKEOFF: {
      const double ramp = (t - t_phase_) * TAKEOFF_CLIMB_MPS;
      return {TAKEOFF, true, {tg[0], tg[1], -std::min(alt_target_, ramp)}, false, 0};
    }
    case GOTO: {
      const Vec3 leg{tg[0] - goto_start_[0], tg[1] - goto_start_[1], tg[2] - goto_start_[2]};
      const double dist = norm(leg);
      const double travelled = (t - t_phase_) * goto_speed_;
      const double f = dist < 1e-6 ? 1.0 : std::min(1.0, travelled / dist);
      return {GOTO, true, {goto_start_[0] + f * leg[0], goto_start_[1] + f * leg[1], goto_start_[2] + f * leg[2]},
              false, 0};
    }
    case HOLD:
    case HOLD_END:
      return {HOLD, true, tg, false, 0};
    case LAND:
      return {LAND, true, {tg[0], tg[1], 0}, true, LAND_DESCENT_MPS};
    case LANDED:
      break;
  }
  return {LANDED, false, {tg[0], tg[1], 0}, false, 0};
}

}  // namespace hwil
