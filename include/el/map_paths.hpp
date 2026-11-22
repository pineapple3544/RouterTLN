#pragma once
#include <filesystem>
#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace el {
inline constexpr char default_map_dir[] = "/workspace/maps/berlin";

struct RouterSettings {
  std::string race_checkpoint="/workspace/models/race-teachers-expanded-v5.pt";
  bool lap_planning=false;
  float plan_lead_time=.25f,plan_speed_fraction=.75f;
  float scout_speed_cap=2.2f;
  float avoid_speed_cap=10.f;
  // Optional closed-loop stress test of Avoid on flying laps, in the local simulator.
  float sim_avoid_min=0, sim_avoid_max=0;
  void validate_sim_band(float cap, bool simulator) const {
    if (sim_avoid_min==0 && sim_avoid_max==0) return;
    if (!simulator || !lap_planning || !(sim_avoid_min>=0 && sim_avoid_max>0 &&
                                         sim_avoid_min<=sim_avoid_max && sim_avoid_max<=cap))
      throw std::runtime_error("invalid simulator Avoid speed band");
  }
  float lookahead_scale=1.f;
  float race_speed_scale=1.f;
};

inline RouterSettings map_router_settings(const std::filesystem::path& directory) {
  RouterSettings settings;
  const auto path = directory / "router.yaml";
  if (!std::filesystem::exists(path)) return settings;
  const auto cfg = YAML::LoadFile(path.string());
  settings.race_checkpoint=cfg["race_checkpoint"].as<std::string>(settings.race_checkpoint);
  settings.avoid_speed_cap=cfg["avoid_speed_cap_mps"].as<float>(10.f);
  settings.lookahead_scale=cfg["avoid_lookahead_scale"].as<float>(1.f);
  settings.race_speed_scale=cfg["race_speed_scale"].as<float>(1.f);
  if (!(settings.avoid_speed_cap>0 && settings.avoid_speed_cap<=10 &&
        settings.lookahead_scale>0 && settings.lookahead_scale<=2 &&
        settings.race_speed_scale>0 && settings.race_speed_scale<=3))
    throw std::runtime_error("invalid router settings: " + path.string());
  return settings;
}

inline std::string map_stem(const std::filesystem::path& directory) {
  const auto stem = directory / directory.filename();
  const auto original = stem.string() + "_orig";
  return std::filesystem::exists(original + ".yaml") ? original : stem.string();
}
}
