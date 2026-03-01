// SafeBoxForest — JSON I/O for Robot, Scene, Results
#pragma once

#include "sbf/core/robot.h"
#include "sbf/core/types.h"
#include <string>
#include <vector>

namespace sbf {

// Load robot from JSON config file (panda.json / iiwa14.json format)
Robot load_robot_json(const std::string& path);

// Load obstacles from JSON file
std::vector<Obstacle> load_obstacles_json(const std::string& path);

// Save obstacles to JSON file
void save_obstacles_json(const std::string& path,
                          const std::vector<Obstacle>& obstacles);

// Save planning result to JSON
void save_result_json(const std::string& path,
                       const PlanningResult& result);

// Load planning result from JSON
PlanningResult load_result_json(const std::string& path);

} // namespace sbf
