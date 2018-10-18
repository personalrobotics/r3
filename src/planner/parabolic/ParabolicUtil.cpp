#include "ParabolicUtil.hpp"

#include <cassert>
#include <set>

#include <dart/common/StlHelpers.hpp>
#include <dart/dart.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/SE2.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>

#include "DynamicPath.h"

using Eigen::Vector2d;
using aikido::statespace::CartesianProduct;
using aikido::statespace::R;
using aikido::statespace::SO2;
using aikido::statespace::SE2;
using aikido::statespace::StateSpace;
using dart::common::make_unique;

using CubicSplineProblem
    = aikido::common::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

namespace aikido {
namespace planner {
namespace parabolic {
namespace detail {

ParabolicRamp::Vector toVector(const Eigen::VectorXd& _x)
{
  ParabolicRamp::Vector output(_x.size());

  for (int i = 0; i < _x.size(); ++i)
    output[i] = _x[i];

  return output;
}

Eigen::VectorXd toEigen(const ParabolicRamp::Vector& _x)
{
  Eigen::VectorXd output(_x.size());

  for (std::size_t i = 0; i < _x.size(); ++i)
    output[i] = _x[i];

  return output;
}

void evaluateAtTime(
    const ParabolicRamp::DynamicPath& _path,
    double _t,
    Eigen::VectorXd& _position,
    Eigen::VectorXd& _velocity)
{
  ParabolicRamp::Vector positionVector;
  _path.Evaluate(_t, positionVector);
  _position = toEigen(positionVector);

  ParabolicRamp::Vector velocityVector;
  _path.Derivative(_t, velocityVector);
  _velocity = toEigen(velocityVector);
}

bool checkStateSpace(const statespace::StateSpace* _stateSpace)
{
  // TODO(JS): Generalize Rn<N> for arbitrary N.
  if (dynamic_cast<const R<0>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<1>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<2>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<3>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<4>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<5>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<6>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<Eigen::Dynamic>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const SO2*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (auto space = dynamic_cast<const CartesianProduct*>(_stateSpace))
  {
    for (std::size_t isubspace = 0; isubspace < space->getNumSubspaces();
         ++isubspace)
    {
      if (!checkStateSpace(space->getSubspace<>(isubspace).get()))
        return false;
    }
    return true;
  }
  else
  {
    return false;
  }
}

Eigen::VectorXd getTangentVector(
    statespace::ConstStateSpacePtr stateSpace,
    const Eigen::VectorXd from,
    const Eigen::VectorXd to)
{
  auto state = stateSpace->createState();

  auto fromInverse = stateSpace->createState();
  stateSpace->expMap(from, state);
  stateSpace->getInverse(state, fromInverse);

  auto toMinusFrom = stateSpace->createState();
  stateSpace->expMap(to, state);
  stateSpace->compose(fromInverse, state, toMinusFrom);

  Eigen::VectorXd tangentVector;
  stateSpace->logMap(toMinusFrom, tangentVector);

  return tangentVector;
}

std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const ParabolicRamp::DynamicPath& _inputPath,
    double _startTime,
    statespace::ConstStateSpacePtr _stateSpace)
{
  const auto dimension = _stateSpace->getDimension();

  // Construct a list of all ramp transition points.
  double t = 0.;
  std::set<double> transitionTimes;
  transitionTimes.insert(t);

  for (const auto& rampNd : _inputPath.ramps)
  {
    for (const auto& ramp1d : rampNd.ramps)
    {
      transitionTimes.insert(t + ramp1d.tswitch1);
      transitionTimes.insert(t + ramp1d.tswitch2);
    }
    t += rampNd.endTime;
    transitionTimes.insert(t);
  }

  // Convert the output to a spline with a knot at each transition time.
  assert(!transitionTimes.empty());
  const auto startIt = std::begin(transitionTimes);
  double timePrev = *startIt;
  transitionTimes.erase(startIt);

  Eigen::VectorXd positionPrev, velocityPrev;
  evaluateAtTime(_inputPath, timePrev, positionPrev, velocityPrev);

  auto _outputTrajectory = make_unique<aikido::trajectory::Spline>(
      _stateSpace, timePrev + _startTime);
  auto segmentStartState = _stateSpace->createState();

  for (const auto timeCurr : transitionTimes)
  {
    Eigen::VectorXd positionCurr, velocityCurr;
    evaluateAtTime(_inputPath, timeCurr, positionCurr, velocityCurr);

    Eigen::VectorXd a(_stateSpace->getDimension());

    auto currentReference = _stateSpace->createState();
    _stateSpace->expMap(positionPrev, currentReference);
    auto referenceInverse = _stateSpace->createState();
    _stateSpace->getInverse(currentReference, referenceInverse);
    _stateSpace->logMap(referenceInverse, a);


    Eigen::Isometry2d transform(Eigen::Isometry2d::Identity());
    transform.linear() = Eigen::Rotation2Dd(a[0]).matrix();
    transform.translation() = a.tail<2>();

    Eigen::Vector3d v1, v2;
    for (int i = 0; i < velocityPrev.size(); ++i)
    {
      v1[i] = velocityPrev[i];
      v2[i] = velocityPrev[i];
    }

    CubicSplineProblem problem(Vector2d(0, timeCurr - timePrev), 4, dimension);
    problem.addConstantConstraint(0, 0, Eigen::VectorXd::Zero(dimension));
    problem.addConstantConstraint(0, 1, transform * v1);
    problem.addConstantConstraint(1, 0, getTangentVector(_stateSpace, positionPrev, positionCurr));
//    problem.addConstantConstraint(1, 0, positionCurr - positionPrev);
    problem.addConstantConstraint(1, 1, transform * v2);
    const auto spline = problem.fit();

    _stateSpace->expMap(positionPrev, segmentStartState);

    // Add the ramp to the output trajectory.
    assert(spline.getCoefficients().size() == 1);
    const auto& coefficients = spline.getCoefficients().front();

    _outputTrajectory->addSegment(
        coefficients, timeCurr - timePrev, segmentStartState);

    timePrev = timeCurr;
    positionPrev = positionCurr;
    velocityPrev = velocityCurr;
  }

  return _outputTrajectory;
}

std::unique_ptr<ParabolicRamp::DynamicPath> convertToDynamicPath(
    const aikido::trajectory::Spline& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    bool _preserveWaypointVelocity)
{
  const auto stateSpace = _inputTrajectory.getStateSpace();
  const auto numWaypoints = _inputTrajectory.getNumWaypoints();

  std::vector<ParabolicRamp::Vector> milestones;
  std::vector<ParabolicRamp::Vector> velocities;
  milestones.reserve(numWaypoints);
  velocities.reserve(numWaypoints);

  Eigen::VectorXd tangentVector, currVec;

  for (std::size_t iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint)
  {
    auto currentState = stateSpace->createState();
    _inputTrajectory.getWaypoint(iwaypoint, currentState);

    stateSpace->logMap(currentState, currVec);
    milestones.emplace_back(toVector(currVec));

    _inputTrajectory.getWaypointDerivative(iwaypoint, 1, tangentVector);
    velocities.emplace_back(toVector(tangentVector));
  }

  auto outputPath = make_unique<ParabolicRamp::DynamicPath>();
  outputPath->Init(toVector(_maxVelocity), toVector(_maxAcceleration));
  if (_preserveWaypointVelocity)
  {
    outputPath->SetMilestones(milestones, velocities);
  }
  else
  {
    outputPath->SetMilestones(milestones);
  }
  if (!outputPath->IsValid())
    throw std::runtime_error("Converted DynamicPath is not valid");
  return outputPath;
}

std::unique_ptr<ParabolicRamp::DynamicPath> convertToDynamicPath(
    const aikido::trajectory::Interpolated& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration)
{
  const auto stateSpace = _inputTrajectory.getStateSpace();
  const auto numWaypoints = _inputTrajectory.getNumWaypoints();

  std::vector<ParabolicRamp::Vector> milestones;
  std::vector<ParabolicRamp::Vector> velocities;
  milestones.reserve(numWaypoints);
  velocities.reserve(numWaypoints);

  Eigen::VectorXd currVec;

  for (std::size_t iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint)
  {
    auto currentState = _inputTrajectory.getWaypoint(iwaypoint);
    stateSpace->logMap(currentState, currVec);
    milestones.emplace_back(toVector(currVec));
  }

  auto outputPath = make_unique<ParabolicRamp::DynamicPath>();
  outputPath->Init(toVector(_maxVelocity), toVector(_maxAcceleration));
  outputPath->SetMilestones(milestones);
  if (!outputPath->IsValid())
    throw std::runtime_error("Converted DynamicPath is not valid");
  return outputPath;
}

} // namespace detail
} // namespace parabolic
} // namespace planner
} // namespace aikido
