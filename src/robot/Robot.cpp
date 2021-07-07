#include "aikido/robot/Robot.hpp"

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>

namespace aikido {
namespace robot {

//==============================================================================
Robot::Robot(
    const dart::common::Uri& urdf,
    const dart::common::Uri& srdf,
    const std::string name,
    const dart::common::ResourceRetrieverPtr& retriever)
  : mName(name)
  , mCollisionDetector(dart::collision::FCLCollisionDetector::create())
  , mSelfCollisionFilter(
        std::make_shared<dart::collision::BodyNodeCollisionFilter>())
  , mResourceRetriever(retriever)
{
  // Read the URDF.
  dart::utils::DartLoader urdfLoader;
  mMetaSkeleton = urdfLoader.parseSkeleton(urdf, mResourceRetriever);
  if (!mMetaSkeleton)
  {
    throw std::runtime_error("Unable to load the robot from URDF.");
  }

  // Read the SRDF.
  urdf::Model urdfModel;
  std::string urdfAsString = mResourceRetriever->readAll(urdf);
  urdfModel.initString(urdfAsString);

  srdf::Model srdfModel;
  std::string srdfAsString = mResourceRetriever->readAll(srdf);
  srdfModel.initString(urdfModel, srdfAsString);
  auto disabledCollisions = srdfModel.getDisabledCollisionPairs();
  for (auto disabledPair : disabledCollisions)
  {
    auto body0 = mMetaSkeleton->getBodyNode(disabledPair.link1_);
    auto body1 = mMetaSkeleton->getBodyNode(disabledPair.link2_);
    mSelfCollisionFilter->addBodyNodePairToBlackList(body0, body1);
  }
  mStateSpace = std::make_shared<statespace::dart::MetaSkeletonStateSpace>(
      mMetaSkeleton.get());
}

//==============================================================================
Robot::Robot(
    const dart::dynamics::MetaSkeletonPtr& skeleton,
    const std::string name,
    const dart::common::ResourceRetrieverPtr& retriever)
  : mName(name), mResourceRetriever(retriever)
{
  mMetaSkeleton = skeleton;
  mStateSpace = std::make_shared<statespace::dart::MetaSkeletonStateSpace>(
      mMetaSkeleton.get());
  // TODO(avk): Do we want to allow SRDF equivalence after construction?
}

//==============================================================================
std::future<void> Robot::executeTrajectory(
    const trajectory::TrajectoryPtr& trajectory) const
{
  if (!mTrajectoryExecutor)
  {
    // TODO(avk): Try with the parent's trajectory executor.
    throw std::invalid_argument("Executor is null!");
  }
  return mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
void Robot::step(const std::chrono::system_clock::time_point& timepoint)
{
  // TODO(avk): Is the following statement valid? Where is it conveyed to the
  // user and what are its implications and where is the parent robot locked?
  // Assumes that the parent robot is locked
  mTrajectoryExecutor->step(timepoint);
}

// ==============================================================================
constraint::dart::CollisionFreePtr Robot::getSelfCollisionConstraint(
    statespace::dart::MetaSkeletonStateSpacePtr space,
    dart::dynamics::MetaSkeletonPtr skeleton) const
{
  // If this is the root root, return self collision constraint.
  if (mRootRobot)
  {
    return mRootRobot->getSelfCollisionConstraint(space, skeleton);
  }

  // TODO(avk): Do I really need this post SRDF?
  // TODO(avk): Can we move this to the constructor?
  // TODO(avk): Why disable adjacent body check?
  auto rootSkeleton = mMetaSkeleton->getBodyNode(0)->getSkeleton();
  rootSkeleton->enableSelfCollisionCheck();
  rootSkeleton->disableAdjacentBodyCheck();

  // Create collision option with self-collision filter from SRDF.
  auto collisionOption
      = dart::collision::CollisionOption(false, 1, mSelfCollisionFilter);

  // Create the constraint and return.
  auto collisionFreeConstraint
      = std::make_shared<constraint::dart::CollisionFree>(
          space, skeleton, mCollisionDetector, collisionOption);
  collisionFreeConstraint->addSelfCheck(
      mCollisionDetector->createCollisionGroupAsSharedPtr(mMetaSkeleton.get()));
  return collisionFreeConstraint;
}

//=============================================================================
constraint::TestablePtr Robot::getFullCollisionConstraint(
    statespace::dart::MetaSkeletonStateSpacePtr space,
    dart::dynamics::MetaSkeletonPtr skeleton,
    const constraint::dart::CollisionFreePtr& collisionFree) const
{
  // If this is the root robot, return full collision constraint.
  if (mRootRobot)
  {
    return mRootRobot->getFullCollisionConstraint(
        space, skeleton, collisionFree);
  }

  // Get the robot's self collision constraint.
  auto selfCollisionFree = getSelfCollisionConstraint(space, skeleton);
  if (!collisionFree)
  {
    return selfCollisionFree;
  }

  // Make testable constraints for collision check.
  std::vector<constraint::ConstTestablePtr> constraints{
      selfCollisionFree, collisionFree};
  return std::make_shared<constraint::TestableIntersection>(
      mStateSpace, constraints);
}

//=============================================================================
std::shared_ptr<Robot> Robot::registerSubRobot(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::string& name)
{
  if (mSubRobots.find(name) != mSubRobots.end())
  {
    throw std::invalid_argument("Child is already present");
  }

  // Create a child robot.
  auto subRobot
      = std::make_shared<Robot>(metaSkeleton, name, mResourceRetriever);
  const auto& root = mRootRobot ? mRootRobot : this;
  subRobot->setRootRobot(root);
  subRobot->mTrajectoryExecutor = mTrajectoryExecutor;
  mSubRobots[name] = subRobot;
  return subRobot;
}

//=============================================================================
trajectory::TrajectoryPtr Robot::planToConfiguration(
    const planner::PlannerPtr& planner,
    const statespace::StateSpace::State* goalState,
    const constraint::TestablePtr& testableConstraint) const
{
  // Create the problem.
  auto problem = planner::ConfigurationToConfiguration(
      mStateSpace, getCurrentState(), goalState, testableConstraint);

  // Solve the problem with the provided planner.
  return planner->plan(problem);
}

} // namespace robot
} // namespace aikido

// TODO(avk): Why do planner and problem store similar information?
// TODO(avk): Why does planToConfiguration need to take current robot state?
// TODO: Switch to PRIMITIVE once this is fixed in DART.
// mCollisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
