#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTOROFFSET_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTOROFFSET_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Planning problem to plan to desired end-effector offset while maintaining
/// the current end-effector orientation.
class ConfigurationToEndEffectorOffset : public Problem
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// offest while maintaining the current orientation.
  /// \param[in] startState Start state.
  /// \param[in] direction Unit vector that represents the direction of motion
  /// [unit vector in the world frame].
  /// \param[in] signedDistance Signed distance to move, in meters.
  /// \param[in] interpolator Interpolator used to produce the output
  /// trajectory.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If the size of \c direction is zero.
  ConfigurationToEndEffectorOffset(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      const statespace::dart::MetaSkeletonStateSpace::State* startState,
      const Eigen::Vector3d& direction,
      double signedDistance,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the end-effector BodyNode to be planned to move a desired offest
  /// while maintaining the current orientation.
  ::dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Returns the start state.
  const statespace::dart::MetaSkeletonStateSpace::State* getStartState() const;

  /// Returns the direction of motion specified in the world frame.
  ///
  /// \note The direction is allowed to be zero vector.
  const Eigen::Vector3d& getDirection() const;

  /// Returns the signed distance in meters to move in the specified direction.
  double getDistance() const;

protected:
  /// End-effector body node.
  const ::dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;

  /// Start state.
  statespace::dart::MetaSkeletonStateSpace::ScopedStateConst mStartState;

  /// Direction of motion.
  const Eigen::Vector3d mDirection;

  /// Signed distance to move in the direction specified.
  const double mDistance;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTOROFFSET_HPP_