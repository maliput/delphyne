/**
 * @file src/agents/trajectory_agent.h
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "delphyne/mi6/agent_base.h"
#include "systems/trajectory.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Interfaces
 *****************************************************************************/

/// @brief Trajectory following agents
class TrajectoryAgent : public delphyne::Agent {
 public:
  TrajectoryAgent(const std::string& name, const std::vector<double>& times,
                  const std::vector<double>& headings,
                  const std::vector<std::vector<double>>& translations);

  std::unique_ptr<DiagramBundle> BuildDiagram() const;

 private:
  const double initial_time_{};
  std::unique_ptr<Trajectory> trajectory_{};
};

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace delphyne
