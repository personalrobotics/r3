#include "aikido/planner/ConfigurationToConfigurations.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfigurations::ConfigurationToConfigurations(
    statespace::StateSpacePtr stateSpace,
    const statespace::StateSpace::State* startState,
    const std::unordered_set<statespace::StateSpace::State*>& goalStates,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mStartState(startState)
  , mGoalStates(goalStates)
  , mConstraint(std::move(constraint))
{
  if (mStateSpace != mConstraint->getStateSpace())
  {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
}

//==============================================================================
const std::string& ConfigurationToConfigurations::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToConfigurations::getStaticType()
{
  static std::string name("ConfigurationToConfigurations");
  return name;
}

//==============================================================================
const statespace::StateSpace::State*
ConfigurationToConfigurations::getStartState() const
{
  return mStartState;
}

//==============================================================================
const std::unordered_set<statespace::StateSpace::State*>&
ConfigurationToConfigurations::getGoalStates() const
{
  return mGoalStates;
}

//==============================================================================
constraint::ConstTestablePtr ConfigurationToConfigurations::getConstraint()
    const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
