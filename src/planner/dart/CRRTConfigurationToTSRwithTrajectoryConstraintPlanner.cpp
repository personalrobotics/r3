#include "aikido/planner/dart/CRRTConfigurationToTSRwithTrajectoryConstraintPlanner.hpp"

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "aikido/constraint.hpp"
#include "aikido/constraint/CyclicSampleable.hpp"
#include "aikido/constraint/FiniteSampleable.hpp"
#include "aikido/constraint/NewtonsMethodProjectable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/planner/ompl/Planner.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace dart {

using constraint::CyclicSampleable;
using constraint::NewtonsMethodProjectable;
using constraint::Sampleable;
using constraint::Testable;
using constraint::dart::createProjectableBounds;
using constraint::dart::createSampleableBounds;
using constraint::dart::createTestableBounds;
using constraint::dart::FrameDifferentiable;
using constraint::dart::FrameTestable;
using constraint::dart::InverseKinematicsSampleable;
using constraint::dart::TSR;
using distance::createDistanceMetric;
using planner::ompl::planCRRTConnect;
using statespace::GeodesicInterpolator;
using statespace::dart::MetaSkeletonStateSaver;
using statespace::dart::MetaSkeletonStateSpace;

using ::dart::dynamics::BodyNode;
using ::dart::dynamics::InverseKinematics;

//==============================================================================
CRRTConfigurationToTSRwithTrajectoryConstraintPlanner::
    CRRTConfigurationToTSRwithTrajectoryConstraintPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
        double timelimit,
        robot::util::CRRTPlannerParameters crrtParameters)
  : dart::SingleProblemPlanner<
        CRRTConfigurationToTSRwithTrajectoryConstraintPlanner,
        ConfigurationToTSRwithTrajectoryConstraint>(
        std::move(stateSpace), std::move(metaSkeleton))
  , mCRRTParameters(std::move(crrtParameters))
  , mTimelimit(timelimit)
{
  // Do nothing
}

//==============================================================================
trajectory::TrajectoryPtr
CRRTConfigurationToTSRwithTrajectoryConstraintPlanner::plan(
    const SolvableProblem& problem, Result* /*result*/)
{
  std::size_t projectionMaxIteration = mCRRTParameters.projectionMaxIteration;
  double projectionTolerance = mCRRTParameters.projectionTolerance;

  auto robot = mMetaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());

  // Save the current state of the stateSpace
  auto saver = MetaSkeletonStateSaver(mMetaSkeleton);
  DART_UNUSED(saver);

  // Create seed constraint
  std::shared_ptr<Sampleable> seedConstraint = createSampleableBounds(
      mMetaSkeletonStateSpace, mCRRTParameters.rng->clone());

  // TODO: DART may be updated to check for single skeleton
  if (mMetaSkeleton->getNumDofs() == 0)
    throw std::invalid_argument("MetaSkeleton has 0 degrees of freedom.");

  auto skeleton = mMetaSkeleton->getDof(0)->getSkeleton();
  for (size_t i = 1; i < mMetaSkeleton->getNumDofs(); ++i)
  {
    if (mMetaSkeleton->getDof(i)->getSkeleton() != skeleton)
      throw std::invalid_argument("MetaSkeleton has more than 1 skeleton.");
  }

  auto bodyNode = problem.getEndEffectorBodyNode();
  auto goalTsr = problem.getGoalTSR();
  auto constraintTsr = problem.getConstraintTSR();

  // Create an IK solver with metaSkeleton dofs
  auto ik = InverseKinematics::create(const_cast<BodyNode*>(bodyNode.get()));

  ik->setDofs(mMetaSkeleton->getDofs());

  // create goal sampleable
  auto goalSampleable = std::make_shared<InverseKinematicsSampleable>(
      mMetaSkeletonStateSpace,
      mMetaSkeleton,
      std::make_shared<CyclicSampleable>(std::const_pointer_cast<TSR>(goalTsr)),
      seedConstraint,
      ik,
      mCRRTParameters.maxNumTrials);

  // create goal testable
  auto goalTestable = std::make_shared<FrameTestable>(
      std::const_pointer_cast<MetaSkeletonStateSpace>(mMetaSkeletonStateSpace),
      mMetaSkeleton,
      bodyNode.get(),
      std::const_pointer_cast<TSR>(goalTsr));

  // create constraint sampleable
  auto constraintSampleable = std::make_shared<InverseKinematicsSampleable>(
      mMetaSkeletonStateSpace,
      mMetaSkeleton,
      std::const_pointer_cast<TSR>(constraintTsr),
      seedConstraint,
      ik,
      mCRRTParameters.maxNumTrials);

  // create constraint projectable
  auto frameDiff = std::make_shared<FrameDifferentiable>(
      std::const_pointer_cast<MetaSkeletonStateSpace>(mMetaSkeletonStateSpace),
      mMetaSkeleton,
      bodyNode.get(),
      std::const_pointer_cast<TSR>(constraintTsr));

  std::vector<double> projectionToleranceVec(
      frameDiff->getConstraintDimension(), projectionTolerance);
  auto constraintProjectable = std::make_shared<NewtonsMethodProjectable>(
      frameDiff, projectionToleranceVec, projectionMaxIteration);

  // Current state
  auto startState = mMetaSkeletonStateSpace->getScopedStateFromMetaSkeleton(
      mMetaSkeleton.get());

  // Call planner
  auto traj = planCRRTConnect(
      startState,
      goalTestable,
      goalSampleable,
      constraintProjectable,
      mMetaSkeletonStateSpace,
      std::make_shared<GeodesicInterpolator>(mMetaSkeletonStateSpace),
      createDistanceMetric(mMetaSkeletonStateSpace),
      constraintSampleable,
      std::const_pointer_cast<constraint::Testable>(problem.getConstraint()),
      createTestableBounds(mMetaSkeletonStateSpace),
      createProjectableBounds(mMetaSkeletonStateSpace),
      mTimelimit,
      mCRRTParameters.maxExtensionDistance,
      mCRRTParameters.maxDistanceBtwProjections,
      mCRRTParameters.minStepSize,
      mCRRTParameters.minTreeConnectionDistance);

  return traj;
}

} // namespace dart
} // namespace planner
} // namespace aikido
