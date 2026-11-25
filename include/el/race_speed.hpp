#pragma once
#include "el/core.hpp"
#include <algorithm>
#include <cmath>

namespace el {
// Maximum speed whose latency travel plus braking distance fits the forward scan.
inline float race_sight_speed_limit(const Scan& scan, float deceleration) {
  constexpr float latency_s = .20f;
  constexpr float nose_reserve_m = .35f;
  constexpr float cone_rad = .12f;
  float distance = scan.range_max;
  if (!std::isfinite(distance) || distance <= 0 || !(deceleration > 0)) return 0;
  int covered = 0;
  float first_angle = 0, last_angle = 0;
  for (int i = 0; i < beams; ++i) {
    const float angle = scan.explicit_angles ? scan.angles[i] :
        scan.angle_min + (scan.angle_max - scan.angle_min) * i / (beams - 1);
    if (!std::isfinite(angle)) return 0;
    if (std::abs(angle) > cone_rad) continue;
    if (!(scan.valid[i] >= .5f) || !std::isfinite(scan.ranges[i]) || scan.ranges[i] <= 0) return 0;
    if (covered && (angle <= last_angle || angle - last_angle > .065f)) return 0;
    if (!covered) first_angle = angle;
    last_angle = angle;
    ++covered;
    distance = std::min(distance, scan.ranges[i] * std::cos(angle));
  }
  if (covered < 3 || first_angle > -.06f || last_angle < .06f) return 0;
  const float lag = deceleration * latency_s;
  return std::sqrt(lag * lag + 2 * deceleration * std::max(0.f, distance - nose_reserve_m)) - lag;
}

// Invert the straight-boost gate: 0.35 m + 0.25 s * v + v² / (2 * 1.8 m/s²).
inline float straight_boost_speed_limit(const Scan& scan){
  float nearest=INFINITY;
  int rays=0;
  for(int b=0;b<beams;++b){
    const float angle=scan.explicit_angles?scan.angles[b]:
      scan.angle_min+(scan.angle_max-scan.angle_min)*b/(beams-1);
    if(!std::isfinite(angle))return 0.f;
    if(fabsf(angle)>.12f)continue;
    if(!(scan.valid[b]>=.5f) || !std::isfinite(scan.ranges[b]) || !(scan.ranges[b]>.35f))return 0.f;
    nearest=std::min(nearest,scan.ranges[b]);
    ++rays;
  }
  if(rays<3)return 0.f;
  constexpr float braking=1.8f,reaction=.25f;
  const float loss=braking*reaction;
  return std::nextafter(sqrtf(loss*loss+2.f*braking*(nearest-.35f))-loss,0.f);
}
}
