#include "aikido/control/ros/RosTrajectoryExecutor.hpp"

#include <dart/common/Console.hpp>

#include "aikido/control/TrajectoryRunningException.hpp"
#include "aikido/control/ros/Conversions.hpp"
#include "aikido/control/ros/RosTrajectoryExecutionException.hpp"
#include "aikido/control/ros/util.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace control {
namespace ros {
namespace {

using aikido::statespace::dart::MetaSkeletonStateSpace;
using std::chrono::milliseconds;

//==============================================================================
std::string getFollowJointTrajectoryErrorMessage(int32_t errorCode)
{
  using Result = control_msgs::FollowJointTrajectoryResult;

  switch (errorCode)
  {
    case Result::SUCCESSFUL:
      return "successful";

    case Result::INVALID_GOAL:
      return "invalid goal";

    case Result::INVALID_JOINTS:
      return "invalid joints";

    case Result::OLD_HEADER_TIMESTAMP:
      return "old header timestamp";

    case Result::PATH_TOLERANCE_VIOLATED:
      return "path tolerance violated";

    case Result::GOAL_TOLERANCE_VIOLATED:
      return "goal tolerance violated";

    default:
      std::stringstream errorString;
      errorString << "unknown (" << errorCode << ")";
      return errorString.str();
  }
}

} // namespace

//==============================================================================
RosTrajectoryExecutor::RosTrajectoryExecutor(
    ::ros::NodeHandle node,
    const std::string& serverName,
    double waypointTimestep,
    double goalTimeTolerance,
    const std::chrono::milliseconds& connectionTimeout,
    const std::chrono::milliseconds& connectionPollingPeriod)
  : mNode{std::move(node)}
  , mCallbackQueue{}
  , mClient{mNode, serverName, &mCallbackQueue}
  , mWaypointTimestep{waypointTimestep}
  , mGoalTimeTolerance{goalTimeTolerance}
  , mConnectionTimeout{connectionTimeout}
  , mConnectionPollingPeriod{connectionPollingPeriod}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  if (mWaypointTimestep <= 0)
    throw std::invalid_argument("Waypoint timestep must be positive.");

  if (mGoalTimeTolerance <= 0)
    throw std::invalid_argument("Goal time tolerance must be positive.");
}

//==============================================================================
RosTrajectoryExecutor::~RosTrajectoryExecutor()
{
  // Do nothing.
  // TODO: Should we wait for the current trajectory to finish executing?
}

//==============================================================================
std::future<void> RosTrajectoryExecutor::execute(
    const trajectory::ConstTrajectoryPtr& traj)
{
  static const ::ros::Time invalidTime;
  return execute(traj, invalidTime);
}

//==============================================================================
void RosTrajectoryExecutor::validate(const trajectory::Trajectory* traj)
{
  if (!traj)
    throw std::invalid_argument("Trajectory is null.");

  if (mValidatedTrajectories.find(traj) != mValidatedTrajectories.end())
    return;

  const auto space = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
      traj->getStateSpace());

  if (!space)
    throw std::invalid_argument(
        "Trajectory is not in a MetaSkeletonStateSpace.");

  // TODO: No check's happening here. Check if that's correct.
  mValidatedTrajectories.emplace(traj);
}
//==============================================================================
std::future<void> RosTrajectoryExecutor::execute(
    const trajectory::ConstTrajectoryPtr& traj, const ::ros::Time& startTime)
{
  using aikido::control::ros::toRosJointTrajectory;

  validate(traj.get());

  // Setup the goal properties.
  // TODO: Also set goal_tolerance, path_tolerance.
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.header.stamp = startTime;
  goal.goal_time_tolerance = ::ros::Duration(mGoalTimeTolerance);

  // Convert the Aikido trajectory into a ROS JointTrajectory.
  goal.trajectory = toRosJointTrajectory(traj, mWaypointTimestep);

  bool waitForServer = waitForActionServer<
      control_msgs::FollowJointTrajectoryAction,
      std::chrono::milliseconds,
      std::chrono::milliseconds>(
      mClient, mCallbackQueue, mConnectionTimeout, mConnectionPollingPeriod);

  if (!waitForServer)
    throw std::runtime_error("Unable to connect to action server.");

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
      throw TrajectoryRunningException();

    mPromise.reset(new std::promise<void>());
    mInProgress = true;
    mGoalHandle = mClient.sendGoal(
        goal,
        boost::bind(&RosTrajectoryExecutor::transitionCallback, this, _1));

    return mPromise->get_future();
  }
}

//==============================================================================
void RosTrajectoryExecutor::transitionCallback(GoalHandle handle)
{
  // This function assumes that mMutex is locked.

  using actionlib::TerminalState;
  using Result = control_msgs::FollowJointTrajectoryResult;

  if (handle.getCommState() == actionlib::CommState::DONE)
  {
    std::stringstream message;
    bool isSuccessful = true;
    int errorCode = -1;

    // Check the status of the actionlib call. Note that the actionlib call can
    // succeed, even if execution failed.
    const auto terminalState = handle.getTerminalState();
    if (terminalState != TerminalState::SUCCEEDED)
    {
      message << "Action " << terminalState.toString();

      const auto terminalMessage = terminalState.getText();
      if (!terminalMessage.empty())
        message << " (" << terminalMessage << ")";

      // mPromise->set_exception(std::make_exception_ptr(
      //    RosTrajectoryExecutionException(message.str(), terminalState)));

      isSuccessful = false;
    }
    else
    {
      message << "Execution failed.";
    }

    // Check the status of execution. This is only possible if the actionlib
    // call succeeded.
    const auto result = handle.getResult();
    if (result && result->error_code != Result::SUCCESSFUL)
    {
      message << ": "
              << getFollowJointTrajectoryErrorMessage(result->error_code);

      if (!result->error_string.empty())
        message << " (" << result->error_string << ")";

      errorCode = result->error_code;

      isSuccessful = false;
    }

    if (isSuccessful)
    {
      mPromise->set_value();
    }
    else
    {
      mPromise->set_exception(std::make_exception_ptr(
          RosTrajectoryExecutionException(message.str(), errorCode)));
    }

    mInProgress = false;
  }
}

//==============================================================================
void RosTrajectoryExecutor::step(
    const std::chrono::system_clock::time_point& /*timepoint*/)
{
  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning.

  mCallbackQueue.callAvailable();

  if (!::ros::ok() && mInProgress)
  {
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Detected ROS shutdown.")));
    mInProgress = false;
  }
}

//==============================================================================
void RosTrajectoryExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning.

  if (mInProgress)
  {
    mGoalHandle.cancel();
  }
  else
  {
    dtwarn << "[RosTrajectoryExecutor::cancel] Attempting to "
           << "cancel trajectory, but no trajectory in progress.\n";
  }
}

} // namespace ros
} // namespace control
} // namespace aikido
