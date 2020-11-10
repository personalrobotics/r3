#include "aikido/planner/SequenceConfigurationToConfigurationMetaPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
SequenceConfigurationToConfigurationMetaPlanner::
    SequenceConfigurationToConfigurationMetaPlanner(
        statespace::ConstStateSpacePtr stateSpace,
        const std::vector<PlannerPtr>& planners,
        common::RNG* rng)
  : SequenceMetaPlanner(stateSpace, std::move(planners))
  , ConfigurationToConfigurationPlanner(stateSpace, std::move(rng))
{
  for (const auto& planner : mPlanners)
  {
    auto castedPlanner
        = std::dynamic_pointer_cast<ConfigurationToConfigurationPlannerPtr>(
            planner);

    if (!castedPlanner)
      throw std::invalid_argument(
          "One of the planners is not ConfigurationToConfigurationPlanner.");
  }
}

//==============================================================================
trajectory::TrajectoryPtr SequenceConfigurationToConfigurationMetaPlanner::plan(
    const SolvableProblem& problem, Result* result)
{
  return SequenceMetaPlanner::plan(problem, result);
}

} // namespace planner
} // namespace aikido
