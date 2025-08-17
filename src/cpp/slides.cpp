#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <vector>

const int STATE_SIZE = 3;

typedef std::array<double, STATE_SIZE> State;

struct SystemInfo {
  double kt;
  double L;
  double R;
  double V;
  double J;
  double a;
  double b;
};

State xdot(SystemInfo info, State x, double u) {
  return State {
    (-x[0]*info.R - x[1]*info.kt + u*info.V)/info.L,
    (x[0]*info.kt - x[1]*info.b - std::copysign(1,x[1])*info.a)/info.J,
    x[1]
  };
}

struct PolicyConfig {
  double sim_dt;
  int max_step;
  SystemInfo info;
};

double optimize(State x0, State target, PolicyConfig config) {
  double initial_power = x0[2]<target[2] ? -1.0 : 1.0;
  double slow_power = -initial_power;
  double p0 = x0[2];
  double pt = target[2];

  int slow_step = 0;

  int lower_bound = 0;
  int upper_bound = config.max_step;

  while(lower_bound!=upper_bound) {
    State x_cur = State{x0};
    int cur_step = 0;

    while(true) {
      double power = initial_power;
      if(cur_step >= slow_step) {
        power = slow_power;
      }

      double old_v = x_cur[1];
      double old_p = x_cur[2];

      auto d = xdot(config.info, x_cur, power);

      for (size_t i = 0; i < x_cur.size(); ++i) {
          x_cur[i] += d[i] * config.sim_dt;
      }

      // if slowing down and turned around, record stopping point.
      if (power == slow_power && std::copysign(1, old_v) != std::copysign(1, x_cur[1])) {
        double stop_p = (x_cur[2]+old_p)/2.0;

        if(std::fabs(p0-pt) < std::fabs(p0-stop_p)) {
          // overshot
          upper_bound = slow_step;
        } else {
          // undershot
          lower_bound = slow_step;
        }

        break;
      }

      cur_step++;
    }

    slow_step = std::min((lower_bound+upper_bound)/2,std::max(1,slow_step*2));
  }

  double stop_time = lower_bound*config.sim_dt;


  return stop_time;
}

void optimize2(State x0, State target, PolicyConfig config) {
  double initial_power = x0[2]<target[2] ? -1.0 : 1.0;
  double slow_power = -initial_power;

  auto polyline1 = std::vector<std::array<double, 2>>(config.max_step);
  auto polyline2 = std::vector<std::array<double, 2>>(config.max_step);

  auto x_cur = State{x0};
  for (int i=0; i<config.max_step; i++) {
    auto d = xdot(config.info, x_cur, initial_power);

    for (size_t i = 0; i < x_cur.size(); ++i) {
        x_cur[i] += d[i] * config.sim_dt;
    }
    polyline1.push_back(std::array {x_cur[1], x_cur[2]});
  }

  x_cur = State{x0};
  for (int i=0; i<config.max_step; i++) {
    auto d = xdot(config.info, x_cur, initial_power);

    for (size_t i = 0; i < x_cur.size(); ++i) {
        x_cur[i] += d[i] * -config.sim_dt;
    }
    polyline2.push_back(std::array {x_cur[1], x_cur[2]});
  }
}
