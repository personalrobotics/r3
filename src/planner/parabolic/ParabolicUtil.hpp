#ifndef PARABOLIC_UTIL_HPP_
#define PARABOLIC_UTIL_HPP_

#include <memory>
#include <Eigen/Dense>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include "DynamicPath.h"

namespace aikido {
namespace planner {
namespace parabolic {
namespace detail {

/// Convert a Eigen vector to a ParabolicRamp vector
/// \param _x an Eigen vector
/// \return a ParabolicRamp vector
ParabolicRamp::Vector toVector(const Eigen::VectorXd& _x);

/// Convert a ParabolicRamp vector to a Eigen vector
/// \param _x a ParabolicRamp vector
/// \return an Eigen vector
Eigen::VectorXd toEigen(const ParabolicRamp::Vector& _x);

/// Evaluate the position and the velocity of a dynamic path
/// given time t
/// \param _path a dynamic path
/// \param _t time
/// \param[out] position at time \c _t
/// \param[out] velocity at time \c _t
void evaluateAtTime(ParabolicRamp::DynamicPath& _path, double _t,
    Eigen::VectorXd& _position, Eigen::VectorXd& _velocity);

/// Check wether the state space is supported
/// \param _stateSpace the state space to be checked
/// \return whether the state space is supported
bool checkStateSpace(const statespace::StateSpace* _stateSpace);

/// Computes the tangent vector given two vectors
/// \param[in] from The source state
/// \param[in] to The target state
Eigen::VectorXd getTangentVector(statespace::ConstStateSpacePtr stateSpace,
                                 const Eigen::VectorXd from,
                                 const Eigen::VectorXd to);

/// Convert an interpolated trajectory to a spline trajectory
/// \param _inputTrajectory interpolated trajectory
/// \return a spline trajectory
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const aikido::trajectory::Interpolated& _inputTrajectory);

/// Convert a dynamic path to a spline trajectory
/// \param _inputTrajectory dynamic path
/// \param _startTime the start time of the spline trajectory
/// \param _stateSpace the state space this trajectory is defined in
/// \return a spline trajectory
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
        const ParabolicRamp::DynamicPath& _inputPath,
        double _startTime,
        statespace::ConstStateSpacePtr _stateSpace);

/// Convert a spline trajectory to a dynamic path
/// \param _inputTrajectory a spline trajectory
/// \param _maxVelocity maximum velocity in each dimension
/// \param _maxAcceleration maximum acceleration in each dimension
/// \param _preserveWaypointVelocity preserve waypoint velocity in conversion
/// \return a dynamic path
std::unique_ptr<ParabolicRamp::DynamicPath>
    convertToDynamicPath(const aikido::trajectory::Spline& _inputTrajectory,
                         const Eigen::VectorXd& _maxVelocity,
                         const Eigen::VectorXd& _maxAcceleration,
                         bool _preserveWaypointVelocity = true);

/// Convert an interpolated trajectory to a dynamic path
/// \param _inputTrajectory a spline trajectory
/// \param _maxVelocity maximum velocity in each dimension
/// \param _maxAcceleration maximum acceleration in each dimension
/// \return a dynamic path
std::unique_ptr<ParabolicRamp::DynamicPath>
    convertToDynamicPath(const aikido::trajectory::Interpolated& _inputTrajectory,
                         const Eigen::VectorXd& _maxVelocity,
                         const Eigen::VectorXd& _maxAcceleration);

} // namespace detail
} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // PARABOLIC_UTIL_HPP_
