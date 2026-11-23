#pragma once

#include <string>

#include "planner_experiments/experiment_types.hpp"

namespace planner_experiments {

class ResultWriter {
 public:
  bool WriteRunDirectory(const std::string& directory,
                         const PlanningResult& result,
                         std::string* error) const;

  static std::string StableRunKey(const PlanningRequest& request,
                                  const std::string& configuration_hash);
  static std::string Sha256File(const std::string& path);
};

}  // namespace planner_experiments
