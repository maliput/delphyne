/**
 * @file src/agents/trajectory_agent.h
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef delphyne_AGENTS_TRAJECTORY_AGENT_H_
#define delphyne_AGENTS_TRAJECTORY_AGENT_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>

#include <drake/automotive/car_vis_applicator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include "delphyne/agent_base.h"

#include "systems/trajectory_follower.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief Trajectory following agents
 *
 * @TODO(daniel.stonier) add agent type (for visualisation purpose only)
 */
class TrajectoryAgent : public delphyne::Agent {

public:
 // DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryAgent)

 TrajectoryAgent();

 int Configure(const std::string& name, const int& id,
               drake::systems::DiagramBuilder<double>* builder,
               drake::systems::rendering::PoseAggregator<double>* aggregator,
               drake::automotive::CarVisApplicator<double>* car_vis_applicator)
     override;

 int Initialize(drake::systems::Context<double>* context) override;

 drake::systems::System<double>* get_system() const;

private:
 drake::automotive::TrajectoryFollower<double>* trajectory_follower_system_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace delphyne

#endif /* delphyne_AGENTS_TRAJECTORY_AGENT_H_ */
