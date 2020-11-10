#ifndef AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONPLANNER_HPP_

#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/planner/SingleProblemPlanner.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

AIKIDO_DECLARE_POINTERS(ConfigurationToConfigurationPlanner)

/// Base planner class for ConfigurationToConfiguration planning problem.
class ConfigurationToConfigurationPlanner
  : public SingleProblemPlanner<
        ConfigurationToConfigurationPlanner,
        ConfigurationToConfiguration>
{
public:
  // Expose the implementation of Planner::plan(const Problem&, Result*) in
  // SingleProblemPlanner. Note that plan() of the base class takes Problem
  // while the virtual function defined in this class takes SolvableProblem,
  // which is simply ConfigurationToConfiguration.
  using SingleProblemPlanner::plan;

  /// \copydoc Planner::Planner
  explicit ConfigurationToConfigurationPlanner(
      statespace::ConstStateSpacePtr stateSpace, common::RNG* rng = nullptr);

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  virtual trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr)
      = 0;
  // Note: SolvableProblem is defined in SingleProblemPlanner.
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
