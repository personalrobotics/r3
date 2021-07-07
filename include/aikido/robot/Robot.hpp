#ifndef AIKIDO_ROBOT_ROBOT_HPP_
#define AIKIDO_ROBOT_ROBOT_HPP_

#include <chrono>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/planner/Planner.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"
#include <aikido/io/CatkinResourceRetriever.hpp>

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Robot)

/// Robot interface for defining basic behaviors of a robot.
/// A concrete implementation of robot should support a collection
/// of planning methods.
class Robot
{
public:
  /// Construct a new Robot object.
  /// \param[in] urdf The path to the robot's URDF.
  /// \param[in] srdf The path to the robot's SRDF.
  /// \param[in] name The name of the robot.
  /// \param[in] retriever Retriever to access resources (urdf/srdf).
  Robot(
      const dart::common::Uri& urdf,
      const dart::common::Uri& srdf,
      const std::string name = "robot",
      const dart::common::ResourceRetrieverPtr& retriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>());

  ///
  /// Construct a new Robot object.
  ///
  /// \param[in] skeleton The skeleton defining the robot.
  /// \param[in] name The name of the robot.
  Robot(
      const dart::dynamics::MetaSkeletonPtr& skeleton,
      const std::string name = "robot",
      const dart::common::ResourceRetrieverPtr& retriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>());

  /// Destructor.
  virtual ~Robot() = default;

  ///
  /// Executes given trajectory on the robot.
  ///
  /// \param[in] trajectory The trajectory to execute.
  virtual std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) const;

  ///
  /// Steps the robot through time.
  ///
  /// \param[in] timepoint The point in time to step to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint);

  // TODO(avk): If I use mSpace and mSkeleton, the whole robot moves around
  // during planning. Is it a locking issue?
  constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      statespace::dart::MetaSkeletonStateSpacePtr space,
      dart::dynamics::MetaSkeletonPtr skeleton) const;

  ///
  /// Given a collision constraint, returns it coupled with the self collision
  /// constraint of the entire robot.
  /// TODO(avk): Are the following parameters really necessary?
  /// \param[in] space The statespace the constraint is specified in.
  /// \param[in] skeleton The skeleton the constraint is specified for.
  /// \param[in] collisionFree The collision constraint that is to be tied with
  /// parent's self collision-constraint.
  constraint::TestablePtr getFullCollisionConstraint(
      statespace::dart::MetaSkeletonStateSpacePtr space,
      dart::dynamics::MetaSkeletonPtr skeleton,
      const constraint::dart::CollisionFreePtr& collisionFree) const;

  /// Registers a subset of the joints of the skeleton as a new robot.
  ///
  /// \param[in] metaSkeleton The metaskeleton corresponding to the subrobot.
  /// \param[in] name Name of the subrobot.
  /// \param[in] This needs to have default trajectory executor?
  virtual std::shared_ptr<Robot> registerSubRobot(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::string& name);

  // Sets the root robot.
  void setRootRobot(Robot* root)
  {
    mRootRobot = root;
  }

  // TODO(avk): Is this function necessary?
  // TODO(avk): Will we want the state from the joint state client?
  statespace::StateSpace::State* getCurrentState() const
  {
    return mStateSpace->getScopedStateFromMetaSkeleton(mMetaSkeleton.get());
  }

  trajectory::TrajectoryPtr planToConfiguration(
      const planner::PlannerPtr& planner,
      const statespace::StateSpace::State* goalState,
      const constraint::TestablePtr& testableConstraint) const;

  // TODO(avk): Make private after testing via libherb.
  std::string mName;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  std::shared_ptr<control::TrajectoryExecutor> mTrajectoryExecutor;

  // TODO(avk): Think about when this will get deleted.
  Robot* mRootRobot{nullptr};
  // TODO(avk): Better way to store subrobots instead of names?
  std::unordered_map<std::string, std::shared_ptr<Robot>> mSubRobots;

  // Collision objects.
  dart::collision::CollisionDetectorPtr mCollisionDetector;
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;

  // Resource retriever.
  const dart::common::ResourceRetrieverPtr& mResourceRetriever;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_ROBOT_HPP_
