// Copyright 2026 The open-hwil-testbed authors. Licensed under Apache-2.0.
#include "controller.hpp"

#include <algorithm>

namespace hwil {

namespace {
constexpr double G = 9.80665;
// Gains: keep identical to hwil/controllers/builtin.py (Gains dataclass).
constexpr double POS_P = 0.95, VEL_XY_P = 1.8, VEL_XY_I = 0.4, VEL_Z_P = 4.0, VEL_Z_I = 2.0;
constexpr double ATT_P = 6.5, YAW_P = 2.8;
constexpr double RATE_RP_P = 0.15, RATE_RP_I = 0.05, RATE_RP_D = 0.003, RATE_Y_P = 0.3, RATE_Y_I = 0.05;
constexpr double HOVER = 0.42, MAX_VEL_XY = 3.0, MAX_CLIMB = 1.5, MAX_DESCENT = 1.0;
const double MAX_TILT = 25.0 * M_PI / 180.0, MAX_RATE_RP = 200.0 * M_PI / 180.0;
// Quad-x mixer, PX4 motor order FR, AL, FL, AR; columns roll, pitch, yaw.
constexpr double MIX[4][3] = {{-1, 1, 1}, {1, -1, 1}, {1, 1, -1}, {-1, -1, -1}};

double clip(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
double wrap(double a) {
  // Python's (a + pi) % (2 pi) - pi, which is always >= -pi.
  double r = std::fmod(a + M_PI, 2 * M_PI);
  if (r < 0) r += 2 * M_PI;
  return r - M_PI;
}
}  // namespace

double Pid::update(double err, double dt) {
  integ = clip(integ + err * dt * ki, -i_limit, i_limit);
  const double deriv = has_prev ? (err - prev) / dt : 0.0;
  prev = err;
  has_prev = true;
  return kp * err + integ + kd * deriv;
}

BuiltinController::BuiltinController(Mission& m)
    : mission_(m),
      vn_{VEL_XY_P, VEL_XY_I, 0, 2.0},
      ve_{VEL_XY_P, VEL_XY_I, 0, 2.0},
      vd_{VEL_Z_P, VEL_Z_I, 0, 4.0},
      rp_{RATE_RP_P, RATE_RP_I, RATE_RP_D, 0.1},
      rq_{RATE_RP_P, RATE_RP_I, RATE_RP_D, 0.1},
      rr_{RATE_Y_P, RATE_Y_I, 0, 0.1} {}

std::array<double, 4> BuiltinController::update(const State& s, double dt) {
  const Setpoint sp = mission_.update(s);
  if (!sp.armed) {
    for (Pid* p : {&vn_, &ve_, &vd_, &rp_, &rq_, &rr_}) p->reset();
    return {0, 0, 0, 0};
  }
  if (!have_yaw_sp_) {
    yaw_sp_ = s.euler[2];
    have_yaw_sp_ = true;
  }

  double v_sp[3] = {POS_P * (sp.pos[0] - s.pos_ned[0]), POS_P * (sp.pos[1] - s.pos_ned[1]),
                    sp.has_vel_down ? sp.vel_down : POS_P * (sp.pos[2] - s.pos_ned[2])};
  const double hspeed = std::hypot(v_sp[0], v_sp[1]);
  if (hspeed > MAX_VEL_XY) {
    v_sp[0] *= MAX_VEL_XY / hspeed;
    v_sp[1] *= MAX_VEL_XY / hspeed;
  }
  v_sp[2] = clip(v_sp[2], -MAX_CLIMB, MAX_DESCENT);

  const bool on_ground = s.alt_agl < 0.3 && (sp.phase == TAKEOFF || sp.phase == LAND);
  const double a_n = on_ground ? 0.0 : vn_.update(v_sp[0] - s.vel_ned[0], dt);
  const double a_e = on_ground ? 0.0 : ve_.update(v_sp[1] - s.vel_ned[1], dt);
  const double a_d = vd_.update(v_sp[2] - s.vel_ned[2], dt);

  const double yaw = s.euler[2];
  const double a_fwd = std::cos(yaw) * a_n + std::sin(yaw) * a_e;
  const double a_right = -std::sin(yaw) * a_n + std::cos(yaw) * a_e;
  const double lift = std::max(G - a_d, 1.0);
  const double pitch_sp = clip(-std::atan2(a_fwd, lift), -MAX_TILT, MAX_TILT);
  const double roll_sp = clip(std::atan2(a_right * std::cos(pitch_sp), lift), -MAX_TILT, MAX_TILT);
  const double tilt = std::max(std::cos(s.euler[0]) * std::cos(s.euler[1]), 0.5);
  const double thrust = HOVER * std::sqrt(lift / G / tilt);

  const double p_sp = clip(ATT_P * (roll_sp - s.euler[0]), -MAX_RATE_RP, MAX_RATE_RP);
  const double q_sp = clip(ATT_P * (pitch_sp - s.euler[1]), -MAX_RATE_RP, MAX_RATE_RP);
  const double r_sp = clip(YAW_P * wrap(yaw_sp_ - yaw), -1.0, 1.0);

  const double tau[3] = {clip(rp_.update(p_sp - s.rates[0], dt), -0.25, 0.25),
                         clip(rq_.update(q_sp - s.rates[1], dt), -0.25, 0.25),
                         clip(rr_.update(r_sp - s.rates[2], dt), -0.25, 0.25)};
  std::array<double, 4> m{};
  for (int i = 0; i < 4; ++i)
    m[i] = clip(thrust + MIX[i][0] * tau[0] + MIX[i][1] * tau[1] + MIX[i][2] * tau[2], 0.0, 1.0);
  return m;
}

}  // namespace hwil
