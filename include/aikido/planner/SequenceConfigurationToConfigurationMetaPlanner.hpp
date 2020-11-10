#ifndef AIKIDO_PLANNER_SEQUENCECONFIGURATIONTOCONFIGURATIONMETAPLANNER_HPP_
#define AIKIDO_PLANNER_SEQUENCECONFIGURATIONTOCONFIGURATIONMETAPLANNER_HPP_

#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/SequenceMetaPlanner.hpp"

namespace aikido {
namespace planner {

/// A meta planner that solves ConfigurationToConfiguration using the sub
/// planners one-by-one sequentially and returns the first successfully planned
/// trajectory.
class SequenceConfigurationToConfigurationMetaPlanner
  : public SequenceMetaPlanner
  , public ConfigurationToConfigurationPlanner
{
public:
  /// Constructs given list of planners.
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] planners Planners to be used by this planner.
  /// \param[in] rng RNG that planner uses. If nullptr, a default is created.
  /// \throw If any of \c planners are not ConfigurationToConfigurationPlanner.
  SequenceConfigurationToConfigurationMetaPlanner(
      statespace::ConstStateSpacePtr stateSpace,
      const std::vector<PlannerPtr>& planners = std::vector<PlannerPtr>(),
      common::RNG* rng = nullptr);

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr) override;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_SEQUENCECONFIGURATIONTOCONFIGURATIONMETAPLANNER_HPP_
