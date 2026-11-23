#include "el/scan_localizer.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <random>

using namespace el;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

Scan synthesize(const Memory& walls, const State& truth, float noise, std::mt19937& random) {
  Scan scan;
  scan.range_max = 18;
  std::normal_distribution<float> measurement_noise(0, noise);
  const float x = truth.x + .27f * cosf(truth.yaw);
  const float y = truth.y + .27f * sinf(truth.yaw);
  for (int b = 0; b < beams; ++b) {
    float angle = truth.yaw + scan.angle_min + b * (scan.angle_max-scan.angle_min)/(beams-1);
    float dx = cosf(angle), dy = sinf(angle), r = .01f;
    while (r < scan.range_max && !walls.at(x + r*dx, y + r*dy)) r += .01f;
    scan.ranges[b] = clamp(r + measurement_noise(random), .05f, scan.range_max);
    scan.valid[b] = 1;
  }
  return scan;
}

void synthetic(const Track& track, const Memory& walls) {
  std::mt19937 random(1543);
  ScanLocalizer estimator(walls);
  require(!estimator.update(Scan{}, track, {}).has_value(), "empty scan accepted");
  for (int k = 0; k < 4; ++k) {
    estimator.reset();
    const auto& w = track.points[(2*k+1)*track.points.size()/8];
    State truth{w.x, w.y, wrap(w.yaw + (k%2 ? pi : 0.f))};
    Scan sample = synthesize(walls, truth, .01f, random);
    sample.pose = {900.f, -800.f, 1.f, 9.f};
    const auto result = estimator.update(sample, track, {});
    require(result.has_value(), "failed to acquire cold pose");
    require(hypotf(result->x-truth.x, result->y-truth.y) < .12f, "position error");
    require(fabsf(wrap(result->yaw-truth.yaw)) < .06f, "heading error");
    sample.stamp = .151;
    require(!estimator.update(sample, track, {}).has_value(), "accepted 151 ms sensor gap");
  }
}

void replay(const Track& track, const Memory& walls, const char* filename) {
  ScanLocalizer estimator(walls);
  std::ifstream input(filename);
  require(bool(input), "cannot open replay");
  std::vector<float> xy, yaw, speed;
  Control previous{};
  for (std::string line; std::getline(input, line); ) {
    auto frame = nlohmann::json::parse(line);
    Scan scan;
    scan.range_max = 18;
    scan.stamp = frame.at("t");
    scan.ranges = frame.at("scan").get<decltype(scan.ranges)>();
    scan.explicit_angles = true;
    scan.pose = {1000.f, -2000.f, -2.f, 10.f};
    for (int i = 0; i < beams; ++i) {
      const int index = std::lround(double(i)*1079/(beams-1));
      scan.angles[i] = scan.angle_min + (scan.angle_max-scan.angle_min)*index/1079;
      scan.valid[i] = std::isfinite(scan.ranges[i]) && scan.ranges[i] > 0;
    }
    const auto pose = estimator.update(scan, track, previous);
    require(pose.has_value(), "lost pose during replay");
    const auto truth = frame.at("car").get<std::vector<float>>();
    xy.push_back(hypotf(pose->x-truth[0], pose->y-truth[1]));
    yaw.push_back(fabsf(wrap(pose->yaw-truth[2])));
    speed.push_back(fabsf(pose->v-truth[3]));
    previous = {frame.at("command").at(0), frame.at("command").at(1)};
  }
  require(!xy.empty(), "empty replay");
  auto p95 = [](std::vector<float>& data) {
    std::sort(data.begin(), data.end());
    return data[(data.size()-1)*95/100];
  };
  nlohmann::json report = {{"frames",xy.size()}, {"position_p95_m",p95(xy)},
                           {"heading_p95_rad",p95(yaw)}, {"speed_p95_mps",p95(speed)}};
  std::cout << report.dump() << '\n';
}

int main(int argc, char** argv) {
  try {
    require(argc == 3 || argc == 4, "provide map YAML, waypoints and optional ticks file");
    Memory walls(argv[1]);
    Track track(argv[2]);
    if (argc == 4) replay(track, walls, argv[3]);
    else synthetic(track, walls);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
