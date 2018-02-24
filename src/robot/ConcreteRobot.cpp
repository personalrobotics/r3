#include "aikido/robot/ConcreteRobot.hpp"
#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"

namespace aikido {
namespace robot {

using common::RNG;
using constraint::dart::CollisionFreePtr;
using constraint::dart::TSR;
using constraint::dart::TSRPtr;
using constraint::TestablePtr;
using planner::parabolic::ParabolicSmoother;
using planner::parabolic::ParabolicTimer;
using statespace::dart::MetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSaver;
using statespace::StateSpace;
using statespace::StateSpacePtr;
using trajectory::TrajectoryPtr;
using trajectory::Interpolated;
using trajectory::InterpolatedPtr;
using trajectory::Spline;
using common::cloneRNGFrom;

using dart::collision::FCLCollisionDetector;
using dart::common::make_unique;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::ChainPtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;

// TODO: Temporary constants for planning calls.
// These should be defined when we construct planner adapter classes
static const double timelimit = 3.0;
static const double maxNumTrials = 10;
static const double collisionResolution = 0.1;
static const double asymmetryTolerance = 1e-3;

namespace {

// TODO: These may not generalize to many robots.
Eigen::VectorXd getSymmetricLimits(
    const MetaSkeleton& metaSkeleton,
    const Eigen::VectorXd& lowerLimits,
    const Eigen::VectorXd& upperLimits,
    const std::string& limitName,
    double asymmetryTolerance)
{
  const auto numDofs = metaSkeleton.getNumDofs();
  assert(static_cast<std::size_t>(lowerLimits.size()) == numDofs);
  assert(static_cast<std::size_t>(upperLimits.size()) == numDofs);

  Eigen::VectorXd symmetricLimits(numDofs);
  for (size_t iDof = 0; iDof < numDofs; ++iDof)
  {
    symmetricLimits[iDof] = std::min(-lowerLimits[iDof], upperLimits[iDof]);
    if (std::abs(lowerLimits[iDof] + upperLimits[iDof]) > asymmetryTolerance)
    {
      dtwarn << "MetaSkeleton '" << metaSkeleton.getName()
             << "' has asymmetric " << limitName << " limits ["
             << lowerLimits[iDof] << ", " << upperLimits[iDof]
             << "] for DegreeOfFreedom '"
             << metaSkeleton.getDof(iDof)->getName() << "' (index: " << iDof
             << "). Using a conservative limit of" << symmetricLimits[iDof]
             << ".";
    }
  }
  return symmetricLimits;
}

Eigen::VectorXd getSymmetricVelocityLimits(
    const MetaSkeleton& metaSkeleton, double asymmetryTolerance)
{
  return getSymmetricLimits(
      metaSkeleton,
      metaSkeleton.getVelocityLowerLimits(),
      metaSkeleton.getVelocityUpperLimits(),
      "velocity",
      asymmetryTolerance);
}

Eigen::VectorXd getSymmetricAccelerationLimits(
    const MetaSkeleton& metaSkeleton, double asymmetryTolerance)
{
  return getSymmetricLimits(
      metaSkeleton,
      metaSkeleton.getAccelerationLowerLimits(),
      metaSkeleton.getAccelerationUpperLimits(),
      "acceleration",
      asymmetryTolerance);
}
}
//==============================================================================
ConcreteRobot::ConcreteRobot(
    const std::string& name,
    MetaSkeletonPtr robot,
    MetaSkeletonStateSpacePtr statespace,
    bool simulation,
    std::unique_ptr<aikido::common::RNG> rng,
    std::shared_ptr<control::TrajectoryExecutor> trajectoryExecutor,
    dart::collision::CollisionDetectorPtr collisionDetector,
    dart::collision::CollisionGroupPtr collideWith,
    std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
        selfCollisionFilter)
  : mRootRobot(this)
  , mName(name)
  , mRobot(robot)
  , mStateSpace(statespace)
  , mParentRobot(mRobot->getBodyNode(0)->getSkeleton())
  , mSimulation(simulation)
  , mRng(std::move(rng))
  , mTrajectoryExecutor(std::move(trajectoryExecutor))
  , mCollisionResolution(collisionResolution)
  , mCollisionDetector(collisionDetector)
  , mCollideWith(collideWith)
  , mSelfCollisionFilter(selfCollisionFilter)
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> ConcreteRobot::smoothPath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path,
    const constraint::TestablePtr& constraint)
{
  Eigen::VectorXd velocityLimits = getVelocityLimits(*metaSkeleton);
  Eigen::VectorXd accelerationLimits = getAccelerationLimits(*metaSkeleton);
  auto smoother
      = std::make_shared<ParabolicSmoother>(velocityLimits, accelerationLimits);

  auto interpolated = dynamic_cast<const Interpolated*>(path);
  if (interpolated)
    return smoother->postprocess(
        *interpolated, *(cloneRNG().get()), constraint);

  auto spline = dynamic_cast<const Spline*>(path);
  if (spline)
    return smoother->postprocess(*spline, *(cloneRNG().get()), constraint);

  throw std::invalid_argument("Path should be either Spline or Interpolated.");
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> ConcreteRobot::retimePath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path)
{
  Eigen::VectorXd velocityLimits = getVelocityLimits(*metaSkeleton);
  Eigen::VectorXd accelerationLimits = getAccelerationLimits(*metaSkeleton);
  auto retimer
      = std::make_shared<ParabolicTimer>(velocityLimits, accelerationLimits);

  auto interpolated = dynamic_cast<const Interpolated*>(path);
  if (interpolated)
    return retimer->postprocess(*interpolated, *(cloneRNG().get()));

  auto spline = dynamic_cast<const Spline*>(path);
  if (spline)
    return retimer->postprocess(*spline, *(cloneRNG().get()));

  throw std::invalid_argument("Path should be either Spline or Interpolated.");
}

//==============================================================================
void ConcreteRobot::executeTrajectory(const TrajectoryPtr& trajectory)
{
  mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
boost::optional<Eigen::VectorXd> ConcreteRobot::getNamedConfiguration(
    const std::string& name) const
{
  auto configuration = mNamedConfigurations.find(name);
  if (configuration == mNamedConfigurations.end())
    return boost::none;

  return configuration->second;
}

//==============================================================================
void ConcreteRobot::setNamedConfigurations(
    std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations)
{
  mNamedConfigurations = namedConfigurations;
}

//==============================================================================
std::string ConcreteRobot::getName() const
{
  return mName;
}

//==============================================================================
MetaSkeletonPtr ConcreteRobot::getMetaSkeleton()
{
  return mRobot;
}

//==============================================================================
MetaSkeletonStateSpacePtr ConcreteRobot::getStateSpace()
{
  return mStateSpace;
}

//=============================================================================
void ConcreteRobot::setRoot(Robot* robot)
{
  if (robot == nullptr)
    throw std::invalid_argument("ConcreteRobot is null.");

  mRootRobot = robot;
}

//==============================================================================
void ConcreteRobot::step(const std::chrono::system_clock::time_point& timepoint)
{
  // Assumes that the parent robot is locked
  mTrajectoryExecutor->step(timepoint);
}

//==============================================================================
Eigen::VectorXd ConcreteRobot::getVelocityLimits(
    const MetaSkeleton& metaSkeleton) const
{
  return getSymmetricVelocityLimits(metaSkeleton, asymmetryTolerance);
}

//==============================================================================
Eigen::VectorXd ConcreteRobot::getAccelerationLimits(
    const MetaSkeleton& metaSkeleton) const
{
  return getSymmetricAccelerationLimits(metaSkeleton, asymmetryTolerance);
}

// ==============================================================================
CollisionFreePtr ConcreteRobot::getSelfCollisionConstraint(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton)
{
  using constraint::dart::CollisionFree;

  if (mRootRobot != this)
    return mRootRobot->getSelfCollisionConstraint(space, metaSkeleton);

  mParentRobot->enableSelfCollisionCheck();
  mParentRobot->disableAdjacentBodyCheck();

  auto collisionDetector = FCLCollisionDetector::create();

  // TODO: Switch to PRIMITIVE once this is fixed in DART.
  // collisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  auto collisionOption
      = dart::collision::CollisionOption(false, 1, mSelfCollisionFilter);
  auto collisionFreeConstraint = std::make_shared<CollisionFree>(
      space, metaSkeleton, collisionDetector, collisionOption);
  collisionFreeConstraint->addSelfCheck(
      collisionDetector->createCollisionGroupAsSharedPtr(mRobot.get()));
  return collisionFreeConstraint;
}

//=============================================================================
TestablePtr ConcreteRobot::getFullCollisionConstraint(
    const MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const CollisionFreePtr& collisionFree)
{
  using constraint::TestableIntersection;

  if (mRootRobot != this)
    return mRootRobot->getFullCollisionConstraint(
        space, metaSkeleton, collisionFree);

  auto selfCollisionFree = getSelfCollisionConstraint(space, metaSkeleton);

  if (!collisionFree)
    return selfCollisionFree;

  // Make testable constraints for collision check
  std::vector<TestablePtr> constraints;
  constraints.reserve(2);
  constraints.emplace_back(selfCollisionFree);
  if (collisionFree)
  {
    if (collisionFree->getStateSpace() != space)
    {
      throw std::runtime_error("CollisionFree has incorrect statespace.");
    }
    constraints.emplace_back(collisionFree);
  }

  return std::make_shared<TestableIntersection>(space, constraints);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToConfiguration(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const StateSpace::State* goalState,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  auto collisionConstraint
      = getFullCollisionConstraint(stateSpace, metaSkeleton, collisionFree);

  return util::planToConfiguration(
      stateSpace,
      metaSkeleton,
      goalState,
      collisionConstraint,
      cloneRNG().get(),
      timelimit);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToConfiguration(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const Eigen::VectorXd& goal,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  auto goalState = stateSpace->createState();
  stateSpace->convertPositionsToState(goal, goalState);

  return planToConfiguration(
      stateSpace, metaSkeleton, goalState, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToConfigurations(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const std::vector<StateSpace::State*>& goalStates,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  return util::planToConfigurations(
      stateSpace,
      metaSkeleton,
      goalStates,
      collisionFree,
      cloneRNG().get(),
      timelimit);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToConfigurations(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const std::vector<Eigen::VectorXd>& goals,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  std::vector<StateSpace::State*> goalStates;
  for (const auto goal : goals)
  {
    auto goalState = stateSpace->createState();
    stateSpace->convertPositionsToState(goal, goalState);
  }

  return planToConfigurations(
      stateSpace, metaSkeleton, goalStates, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToTSR(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bn,
    const TSRPtr& tsr,
    const CollisionFreePtr& collisionFree,
    double timelimit,
    size_t maxNumTrials)
{
  auto collisionConstraint
      = getFullCollisionConstraint(stateSpace, metaSkeleton, collisionFree);

  return util::planToTSR(
      stateSpace,
      metaSkeleton,
      bn,
      tsr,
      collisionConstraint,
      cloneRNG().get(),
      timelimit,
      maxNumTrials);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToTSRwithTrajectoryConstraint(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const BodyNodePtr& bodyNode,
    const TSRPtr& goalTsr,
    const TSRPtr& constraintTsr,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  auto collisionConstraint
      = getFullCollisionConstraint(stateSpace, metaSkeleton, collisionFree);

  // Uses CRRT.
  return util::planToTSRwithTrajectoryConstraint(
      stateSpace,
      metaSkeleton,
      bodyNode,
      goalTsr,
      constraintTsr,
      collisionConstraint,
      timelimit,
      mCRRTParameters);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToNamedConfiguration(
    const std::string& name,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  if (mNamedConfigurations.find(name) == mNamedConfigurations.end())
    throw std::runtime_error(name + " does not exist.");

  auto configuration = mNamedConfigurations[name];
  auto goalState = mStateSpace->createState();
  mStateSpace->convertPositionsToState(configuration, goalState);

  return planToConfiguration(
      mStateSpace, mRobot, goalState, collisionFree, timelimit);
}

//=============================================================================
void ConcreteRobot::setCRRTPlannerParameters(
    const util::CRRTPlannerParameters& crrtParameters)
{
  mCRRTParameters = crrtParameters;
}

//==============================================================================
std::unique_ptr<common::RNG> ConcreteRobot::cloneRNG()
{
  return mRng->clone();
}

} // namespace robot
} // namespace aikido