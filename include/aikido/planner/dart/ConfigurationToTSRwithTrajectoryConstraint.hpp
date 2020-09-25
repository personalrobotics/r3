#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOTSRWITHTRAJECTORYCONSTRAINT_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOTSRWITHTRAJECTORYCONSTRAINT_HPP_

#include "aikido/planner/dart/ConfigurationToTSR.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Planning problem to plan to a given single Task Space Region (TSR).
class ConfigurationToTSRwithTrajectoryConstraint : public ConfigurationToTSR
{
public:
  /// Constructor. Note that this constructor takes the start state from the
  /// current state of the passed MetaSkeleton.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] metaSkeleton MetaSkeleton that getStartState will return the
  /// current state of when called.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// TSR.
  /// \param[in] goalTSR Goal TSR.
  /// \param[in] constraintTsr The constraint TSR for the trajectory.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToTSRwithTrajectoryConstraint(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      constraint::dart::ConstTSRPtr goalTSR,
      constraint::dart::ConstTSRPtr constraintTSR,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Constructor. Note that this constructor sets the start state on
  /// construction.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] startState Start state to plan from.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// TSR.
  /// \param[in] maxSamples Maximum number of TSR samples to plan to.
  /// \param[in] goalTSR Goal TSR.
  /// \param[in] constraintTsr The constraint TSR for the trajectory
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToTSRwithTrajectoryConstraint(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      const statespace::dart::MetaSkeletonStateSpace::State* startState,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      constraint::dart::ConstTSRPtr goalTSR,
      constraint::dart::ConstTSRPtr constraintTSR,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Return the trajectory constraint TSR
  constraint::dart::ConstTSRPtr getConstraintTSR() const;

protected:
  /// Constraint TSR
  const constraint::dart::ConstTSRPtr mConstraintTSR;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOTSRWITHTRAJECTORYCONSTRAINT_HPP_
