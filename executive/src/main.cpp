// hwil_exec: compiled fixed-rate executive for the open HWIL testbed.
//
// Runs JSBSim and the builtin reference controller on a fixed frame and writes
// the same log.csv / timing.csv columns as the Python executive
// (hwil/executive.py), so the pytest + RTAMT harness grades both unchanged.
// It is normally started by `hwil run --executive cpp`, which writes the
// config file and does the grading, plots and manifest afterwards.
//
//   hwil_exec --config run.cfg --out <dir> [--clock afap|realtime]
//             [--rt-priority N] [--cpu N] [--spin-us N]
//
// Copyright 2026 The open-hwil-testbed authors. Licensed under Apache-2.0.
#include <sched.h>
#include <sys/prctl.h>
#include <sys/mman.h>
#include <time.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "config.hpp"
#include "controller.hpp"
#include "fdm.hpp"
#include "mission.hpp"

using namespace hwil;

namespace {

constexpr int LOG_COLS = 34;
const char* LOG_HEADER =
    "t,phase,mission_index,n_m,e_m,d_m,alt_m,alt_cmd_m,alt_agl_m,vn_mps,ve_mps,vd_mps,"
    "roll_rad,pitch_rad,yaw_rad,p_rps,q_rps,r_rps,ax_mps2,ay_mps2,az_mps2,m0,m1,m2,m3,"
    "wind_n_mps,wind_e_mps,wind_d_mps,lat_deg,lon_deg,baro_alt_m,gps_valid,n_cmd_m,e_cmd_m";

int64_t now_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// Sleep until `spin_ns` before the deadline, then spin. Same strategy as the
// Python executive, so the two lateness figures are comparable. --spin-us 0
// gives a pure clock_nanosleep wait, the usual choice on PREEMPT_RT.
void sleep_until(int64_t t_ns, int64_t spin_ns) {
  const int64_t wake = t_ns - spin_ns;
  timespec ts{static_cast<time_t>(wake / 1000000000LL), static_cast<long>(wake % 1000000000LL)};
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
  while (now_ns() < t_ns) {
  }
}

bool sane(const State& s) {
  for (const Vec3* v : {&s.pos_ned, &s.vel_ned, &s.euler, &s.rates, &s.accel})
    for (double x : *v)
      if (!std::isfinite(x) || std::fabs(x) >= 1e5) return false;
  return true;
}

bool fault_active(const Item& f, double t) {
  const double start = f.get("t_s", 0.0);
  return t >= start && (!f.has("duration_s") || t < start + f.get("duration_s", 0.0));
}

int phase_id(Phase p) { return p == HOLD_END ? HOLD : static_cast<int>(p); }

void usage() {
  std::fprintf(stderr, "usage: hwil_exec --config FILE --out DIR [--clock afap|realtime] [--rt-priority N] [--cpu N] [--spin-us N]\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string cfg_path, out_dir, clock_mode = "afap";
  int rt_priority = -1, cpu = -1, spin_us = 150;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
      return argv[++i];
    };
    if (a == "--config") cfg_path = next();
    else if (a == "--out") out_dir = next();
    else if (a == "--clock") clock_mode = next();
    else if (a == "--rt-priority") rt_priority = std::stoi(next());
    else if (a == "--cpu") cpu = std::stoi(next());
    else if (a == "--spin-us") spin_us = std::stoi(next());
    else { usage(); return 2; }
  }
  if (cfg_path.empty() || out_dir.empty()) { usage(); return 2; }
  const bool realtime = clock_mode == "realtime";

  try {
    const Config cfg = load_config(cfg_path);
    Fdm fdm(cfg);
    Mission mission(cfg.mission);
    BuiltinController ctrl(mission);
    const double dt = fdm.dt();
    const double duration = cfg.num("duration_s");
    const double stop_after = cfg.num("stop_after_landed_s");
    const size_t n_max = static_cast<size_t>(std::ceil(duration / dt));

    std::vector<double> log(n_max * LOG_COLS);
    std::vector<double> tim(n_max * 6);

    // Real-time setup happens after all allocation, before the loop.
    std::string rt_note;
    if (realtime) {
      if (cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cpu, &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0) rt_note += std::string("affinity:") + std::strerror(errno) + ";";
      }
      if (rt_priority > 0) {
        sched_param sp{};
        sp.sched_priority = rt_priority;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) rt_note += std::string("sched_fifo:") + std::strerror(errno) + ";";
      }
      // Default timer slack is 50 us for normal threads; ask for the minimum.
      if (prctl(PR_SET_TIMERSLACK, 1UL) != 0) rt_note += std::string("timerslack:") + std::strerror(errno) + ";";
      if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) rt_note += std::string("mlockall:") + std::strerror(errno) + ";";
    }

    const int64_t period = static_cast<int64_t>(std::llround(dt * 1e9));
    const int64_t t0 = now_ns();
    size_t n = 0;
    bool landed_seen = false;
    double landed_at = 0;
    std::string abort_msg;
    for (; n < n_max; ++n) {
      const int64_t deadline = t0 + static_cast<int64_t>(n) * period;
      if (realtime) sleep_until(deadline, int64_t(spin_us) * 1000);
      const int64_t start = now_ns();
      const double late_us = realtime ? (start - deadline) / 1e3 : 0.0;

      const State s = fdm.state();
      if (!sane(s)) {
        char buf[128];
        std::snprintf(buf, sizeof buf, "vehicle state became invalid at t=%.3f s (simulation diverged)", s.t);
        abort_msg = buf;
        break;
      }
      std::array<double, 4> m = ctrl.update(s, dt);
      double gps_valid = 1.0;
      for (const Item& f : cfg.faults) {
        if (!fault_active(f, s.t)) continue;
        if (f.type == "motor_efficiency") m[static_cast<int>(f.get("motor", 0))] *= f.get("factor", 1.0);
        else if (f.type == "gps_dropout") gps_valid = 0.0;
      }
      fdm.set_motors(m);
      fdm.step();

      const Phase ph = mission.phase();
      const bool has_alt_cmd = ph == TAKEOFF || ph == GOTO || ph == HOLD || ph == HOLD_END;
      const auto& mc = fdm.motors();
      double* row = &log[n * LOG_COLS];
      const double vals[LOG_COLS] = {
          s.t, double(phase_id(ph)), double(mission.index()),
          s.pos_ned[0], s.pos_ned[1], s.pos_ned[2], s.alt(), has_alt_cmd ? mission.alt_target() : 0.0, s.alt_agl,
          s.vel_ned[0], s.vel_ned[1], s.vel_ned[2], s.euler[0], s.euler[1], s.euler[2],
          s.rates[0], s.rates[1], s.rates[2], s.accel[0], s.accel[1], s.accel[2],
          mc[0], mc[1], mc[2], mc[3], s.wind[0], s.wind[1], s.wind[2], s.lat, s.lon,
          s.alt_msl,  // noise-free: the compiled executive has no sensor models yet
          gps_valid, mission.north_target(), mission.east_target()};
      std::copy(vals, vals + LOG_COLS, row);

      const int64_t end = now_ns();
      const double compute_us = (end - start) / 1e3;
      const double overrun = (realtime && end > deadline + period) ? 1.0 : 0.0;
      const double trow[6] = {double(n), s.t, late_us, compute_us, 0.0, overrun};
      std::copy(trow, trow + 6, &tim[n * 6]);

      if (ph == LANDED) {
        if (!landed_seen) { landed_seen = true; landed_at = s.t; }
        if (s.t - landed_at >= stop_after) { ++n; break; }
      }
    }
    const double wall_s = (now_ns() - t0) / 1e9;

    auto open = [&](const char* name) {
      FILE* f = std::fopen((out_dir + "/" + name).c_str(), "w");
      if (!f) throw std::runtime_error(std::string("cannot write ") + name);
      return f;
    };
    FILE* f = open("log.csv");
    std::fprintf(f, "%s\n", LOG_HEADER);
    for (size_t i = 0; i < n; ++i) {
      for (int c = 0; c < LOG_COLS; ++c) std::fprintf(f, c ? ",%.9g" : "%.9g", log[i * LOG_COLS + c]);
      std::fputc('\n', f);
    }
    std::fclose(f);
    f = open("timing.csv");
    std::fprintf(f, "frame,t_sim,late_us,compute_us,io_wait_us,overrun\n");
    for (size_t i = 0; i < n; ++i) {
      for (int c = 0; c < 6; ++c) std::fprintf(f, c ? ",%.9g" : "%.9g", tim[i * 6 + c]);
      std::fputc('\n', f);
    }
    std::fclose(f);
    f = open("events.csv");
    std::fprintf(f, "t,phase\n");
    for (const auto& e : mission.events()) std::fprintf(f, "%.9g,%s\n", e.first, e.second.c_str());
    std::fclose(f);
    f = open("exec_summary.txt");
    std::fprintf(f, "landed %d\nwall_time_s %.6f\nrt_note %s\naborted %s\n", mission.phase() == LANDED ? 1 : 0, wall_s,
                 rt_note.c_str(), abort_msg.c_str());
    std::fclose(f);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "hwil_exec: %s\n", e.what());
    return 1;
  }
  return 0;
}
