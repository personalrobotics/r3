#ifndef AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYVARIABLE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYVARIABLE_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/planner/optimization/Variable.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class TrajectoryVariable : public Variable
{
public:
  /// Constructor
  TrajectoryVariable();

  /// Destructor
  virtual ~TrajectoryVariable() = default;

  /// Returns the dimension of optimization variables.
  std::size_t getDimension() const override;

  /// Returns const Trajectory
  ///
  /// We only provide const version of this function because we don't intend
  /// the structure and parameters of spline to be changed by the caller.
  virtual const trajectory::Trajectory& getTrajectory() const = 0;

  // Documentation inherited.
  bool isTrajectoryVariable() const override final;

  // Documentation inherited.
  TrajectoryVariable* asTrajectoryVariable() override final;

  // Documentation inherited.
  const TrajectoryVariable* asTrajectoryVariable() const override final;

protected:
  /// Updates the dimension of this optimization variable.
  virtual void updateDimension() const = 0;

  bool mNeedDimensionUpdate;

  mutable std::size_t mDimension;
};

using TrajectoryVariablePtr = std::shared_ptr<TrajectoryVariable>;
using ConstTrajectoryVariablePtr = std::shared_ptr<const TrajectoryVariable>;

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYVARIABLE_HPP_