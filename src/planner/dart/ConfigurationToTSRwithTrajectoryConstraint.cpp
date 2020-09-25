#include "aikido/planner/dart/ConfigurationToTSRwithTrajectoryConstraint.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToTSRwithTrajectoryConstraint::
    ConfigurationToTSRwithTrajectoryConstraint(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
        ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
        constraint::dart::ConstTSRPtr goalTSR,
        constraint::dart::ConstTSRPtr constraintTSR,
        constraint::ConstTestablePtr constraint)
  : ConfigurationToTSR(
        std::move(stateSpace),
        std::move(metaSkeleton),
        std::move(endEffectorBodyNode),
        0, // This parameter is not used in this class.
        std::move(goalTSR),
        std::move(constraint))
  , mConstraintTSR(constraintTSR)
{
  // Do nothing.
}

//==============================================================================
ConfigurationToTSRwithTrajectoryConstraint::
    ConfigurationToTSRwithTrajectoryConstraint(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        const statespace::dart::MetaSkeletonStateSpace::State* startState,
        ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
        constraint::dart::ConstTSRPtr goalTSR,
        constraint::dart::ConstTSRPtr constraintTSR,
        constraint::ConstTestablePtr constraint)
  : ConfigurationToTSR(
        std::move(stateSpace),
        startState,
        std::move(endEffectorBodyNode),
        0, // This parameter is not used in this class.
        std::move(goalTSR),
        std::move(constraint))
  , mConstraintTSR(constraintTSR)
{
  // Do nothing.
}

//==============================================================================
constraint::dart::ConstTSRPtr
ConfigurationToTSRwithTrajectoryConstraint::getConstraintTSR() const
{
  return mConstraintTSR;
}

} // namespace dart
} // namespace planner
} // namespace aikido
