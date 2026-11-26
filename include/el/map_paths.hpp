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
  bool sim_section_speed_profile=false;
  bool sim_race_only=false;
  bool straight_boost=false;
  bool race_sight_speed=false;
  bool race_exit_speed=false;
  bool race_lap_speed=false;
  float race_brake_preview_s=.18f;
  // Compute budget for the Avoid fallback command search, ms since the router step began
  // (0 = unlimited). When exhausted, a guard-checked brake command replaces the latched stop.
  float fallback_budget_ms=0;
  // Stop with the last published steering held (what the command guard's braking rollout
  // assumes) instead of the steering estimated from yaw rate / speed. Default off.
  bool stop_hold_steering=false;
  // Run the fallback command search in Race mode too before latching a stop. Default off.
  bool race_fallback=false;
  void validate_sim_band(float cap, bool simulator) const {
    if(race_lap_speed && (race_sight_speed || race_exit_speed))
      throw std::runtime_error("choose either the scout speed plan or the earlier Race envelope");
    if((race_lap_speed || race_sight_speed || race_exit_speed) && (!sim_race_only || straight_boost))
      throw std::runtime_error("Race envelope requires Race-only planning without straight_boost");
    if(straight_boost && !sim_race_only)throw std::runtime_error("straight boost requires Race-only speed profile");
    if(sim_race_only && !sim_section_speed_profile)
      throw std::runtime_error("Race-only speed control requires the section speed profile");
    if(!(race_brake_preview_s>=0 && race_brake_preview_s<=.5f))
      throw std::runtime_error("invalid Race braking preview duration");
    if(sim_section_speed_profile && !(simulator && lap_planning && sim_avoid_min>0 &&
                                     sim_avoid_max>sim_avoid_min && cap<=10.f))
      throw std::runtime_error("adaptive speed requires a local simulator and a nonempty speed band");
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
