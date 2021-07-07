#include "aikido/rviz/InteractiveMarkerViewer.hpp"

#include <dart/dart.hpp>

#include "aikido/common/memory.hpp"
#include "aikido/rviz/FrameMarker.hpp"
#include "aikido/rviz/SkeletonMarker.hpp"
#include "aikido/rviz/TrajectoryMarker.hpp"

using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using interactive_markers::InteractiveMarkerServer;

namespace aikido {
namespace rviz {

//==============================================================================
InteractiveMarkerViewer::InteractiveMarkerViewer(
    const std::string& topicNamespace,
    const std::string& frameId,
    aikido::planner::WorldPtr env)
  : mMarkerServer(topicNamespace, "", true)
  , mTrajectoryNameManager("Trajectory Name Manager", "Trajectory")
  , mRunning(false)
  , mUpdating(false)
  , mFrameId(frameId)
  , mWorld(std::move(env))
{
  // Set the pattern for trajectory names.
  mTrajectoryNameManager.setPattern("Frame[%s(%d)]");

  // Sets the viewer to update by default.
  setAutoUpdate(true);
}

//==============================================================================
InteractiveMarkerViewer::~InteractiveMarkerViewer()
{
  mRunning.store(false, std::memory_order_release);
  mThread.join();
}

//==============================================================================
InteractiveMarkerServer& InteractiveMarkerViewer::marker_server()
{
  return mMarkerServer;
}

//==============================================================================
SkeletonMarkerPtr InteractiveMarkerViewer::addSkeletonMarker(
    const SkeletonPtr& skeleton)
{
  std::lock_guard<std::mutex> lock(mMutex);
  const SkeletonMarkerPtr marker = std::make_shared<SkeletonMarker>(
      nullptr, &mMarkerServer, skeleton, mFrameId);
  mSkeletonMarkers.emplace(skeleton, marker);
  return marker;
}

//==============================================================================
void InteractiveMarkerViewer::updateSkeletonMarkers()
{
  // Update SkeletonMarkers from the world.
  if (mWorld)
  {
    for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i)
    {
      const dart::dynamics::SkeletonPtr skeleton = mWorld->getSkeleton(i);
      if (skeleton)
      {
        // Adds skeleton if previously not in the world.
        mSkeletonMarkers.emplace(
            skeleton,
            std::make_shared<SkeletonMarker>(
                nullptr, &mMarkerServer, skeleton, mFrameId));
      }
    }
  }

  // Update existing skeletons, and delete erased skeletons.
  auto it = std::begin(mSkeletonMarkers);
  while (it != std::end(mSkeletonMarkers))
  {
    // If the skeleton is either a nullptr or no longer exists in an associated
    // world, delete.
    if (!it->first || (mWorld && !mWorld->hasSkeleton(it->first)))
    {
      it = mSkeletonMarkers.erase(it);
    }
    else
    {
      // In any other case, since the skeleton exists, update it, increment
      // iterator.
      std::unique_lock<std::mutex> skeleton_lock(
          it->first->getMutex(), std::try_to_lock);
      if (skeleton_lock.owns_lock())
      {
        it->second->update();
      }
      ++it;
    }
  }
}

//==============================================================================
FrameMarkerPtr InteractiveMarkerViewer::addFrameMarker(
    dart::dynamics::Frame* frame, double length, double thickness, double alpha)
{
  std::lock_guard<std::mutex> lock(mMutex);
  const FrameMarkerPtr marker = std::make_shared<FrameMarker>(
      &mMarkerServer, frame, mFrameId, length, thickness, alpha);
  mFrameMarkers.insert(marker);
  return marker;
}

//==============================================================================
void InteractiveMarkerViewer::updateFrameMarkers()
{
  for (const auto& marker : mFrameMarkers)
    marker->update();
}

//==============================================================================
TSRMarkerPtr InteractiveMarkerViewer::addTSRMarker(
    const constraint::dart::TSR& tsr, int nSamples, const std::string& basename)
{
  using dart::dynamics::Frame;
  using dart::dynamics::SimpleFrame;

  auto sampler = tsr.createSampleGenerator();
  auto state = tsr.getSE3()->createState();

  std::string name;
  if (basename.empty())
  {
    std::ostringstream ost;
    ost << &tsr;
    name = ost.str();
  }
  else
  {
    name = basename;
  }

  std::vector<std::unique_ptr<SimpleFrame>> tsrFrames;
  tsrFrames.reserve(nSamples);

  for (int i = 0; i < nSamples && sampler->canSample(); ++i)
  {
    auto sampled = sampler->sample(state);
    assert(sampled);
    DART_UNUSED(sampled);

    std::stringstream ss;
    ss << "TSRMarker[" << name << "].frame[" << i << "]";

    auto tsrFrame = ::aikido::common::make_unique<SimpleFrame>(
        Frame::World(), ss.str(), state.getIsometry());
    addFrameMarker(tsrFrame.get());
    tsrFrames.emplace_back(std::move(tsrFrame));
  }

  auto tsrMarker = std::make_shared<TSRMarker>(std::move(tsrFrames));
  return tsrMarker;
}

//==============================================================================
TrajectoryMarkerPtr InteractiveMarkerViewer::addTrajectoryMarker(
    trajectory::ConstTrajectoryPtr trajectory,
    dart::dynamics::MetaSkeletonPtr skeleton,
    const dart::dynamics::Frame& frame,
    const Eigen::Vector4d& rgba,
    double thickness,
    std::size_t numLineSegments)
{
  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock);
  auto marker = std::make_shared<TrajectoryMarker>(
      &mMarkerServer,
      mFrameId,
      mTrajectoryNameManager.issueNewNameAndAdd("", trajectory),
      std::move(trajectory),
      std::move(skeleton),
      frame,
      rgba,
      thickness,
      numLineSegments);
  mTrajectoryMarkers.insert(marker);

  return marker;
}

//==============================================================================
void InteractiveMarkerViewer::updateTrajectoryMarkers()
{
  for (const auto& marker : mTrajectoryMarkers)
    marker->update();
}

//==============================================================================
void InteractiveMarkerViewer::setAutoUpdate(bool flag)
{
  mUpdating.store(flag, std::memory_order_release);

  const bool wasRunning = mRunning.exchange(flag);
  if (flag && !wasRunning)
    mThread = std::thread(&InteractiveMarkerViewer::autoUpdate, this);
  else if (!flag && wasRunning)
    mThread.join();
}

//==============================================================================
void InteractiveMarkerViewer::autoUpdate()
{
  ros::Rate rate(30);

  while (mRunning.load() && ros::ok())
  {
    update();
    rate.sleep();
  }

  mRunning.store(false);
}

//==============================================================================
void InteractiveMarkerViewer::update()
{
  std::lock_guard<std::mutex> lock(mMutex);

  // Update skeleton markers.
  updateSkeletonMarkers();

  // Update frame markers.
  updateFrameMarkers();

  // Update trajectory markers.
  updateTrajectoryMarkers();

  // Apply changes anytime a marker has been modified.
  mMarkerServer.applyChanges();
}

} // namespace rviz
} // namespace aikido
