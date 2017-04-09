#include <gtest/gtest.h>
#include <aikido/control/ros/Conversions.hpp>
#include <dart/dart.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/statespace/Rn.hpp>
#include "eigen_tests.hpp"

using dart::dynamics::SkeletonPtr;
using dart::dynamics::Skeleton;
using dart::dynamics::BodyNode;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::BallJoint;
using aikido::trajectory::Spline;
using aikido::control::ros::convertTrajectoryToRosTrajectory;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::tests::make_vector;

static const double kTolerance{1e-6};

// TODO: We might want to merge this with test_Conversions.cpp
class ConvertTrajectoryToRosJointTrajectoryTests : public testing::Test
{
protected:
  void SetUp() override
  {
    // Create a single-DOF skeleton.
    auto skeleton = Skeleton::create();
    skeleton->createJointAndBodyNodePair<RevoluteJoint>();
    skeleton->getJoint(0)->setName("Joint1");

    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(skeleton);
    auto startState = mStateSpace->createState();
    mStateSpace->getState(startState);

    // Spline trajectory
    mTrajectory = std::make_shared<Spline>(mStateSpace, 0.);
    Eigen::Matrix<double, 1, 2> coeffs;
    coeffs << 0., 1.;
    mTrajectory->addSegment(coeffs, 0.1, startState);

    // Timestep
    mTimestep = 0.1;

    // indexMap
    mIndexMap.insert(std::make_pair("Joint1", 0));

    // Create a 2-DOF skeleton.
    auto skeleton2 = Skeleton::create();
    RevoluteJoint::Properties jointProperties;
    jointProperties.mName = "Joint1";
    BodyNode::Properties bnProperties;
    bnProperties.mName = "BodyNode1";
    
    auto bn = skeleton2->createJointAndBodyNodePair<
      RevoluteJoint, BodyNode>(
        nullptr, jointProperties, bnProperties).second;
    skeleton2->createJointAndBodyNodePair<RevoluteJoint>(bn);
    skeleton2->getJoint(0)->setName("Joint1");
    skeleton2->getJoint(1)->setName("Joint2");
    skeleton2->getJoint(1)->setPosition(0, 1);

    mStateSpace2DOF = std::make_shared<MetaSkeletonStateSpace>(skeleton2);
    auto startState2DOF = mStateSpace2DOF->createState();
    mStateSpace2DOF->getState(startState2DOF);

    // Spline trajectory
    mTrajectory2DOF = std::make_shared<Spline>(mStateSpace2DOF, 0.);
    Eigen::Matrix2d coeffs2;
    coeffs2 << 3., 1.,
               1., 1.;
    mTrajectory2DOF->addSegment(coeffs2, 0.1, startState2DOF);
    mIndexMap2DOF.insert(std::make_pair("Joint1", 0));
    mIndexMap2DOF.insert(std::make_pair("Joint2", 1));

  }

  std::shared_ptr<Spline> mTrajectory;
  MetaSkeletonStateSpacePtr mStateSpace;

  std::shared_ptr<Spline> mTrajectory2DOF;
  MetaSkeletonStateSpacePtr mStateSpace2DOF;

  double mTimestep;
  std::map<std::string, size_t> mIndexMap;
  std::map<std::string, size_t> mIndexMap2DOF;
};

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, TrajectoryIsNull_Throws)
{
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(nullptr, mIndexMap, mTimestep);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, TimestepIsZero_Throws)
{
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(mTrajectory, mIndexMap, 0.0);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, TimestepIsNegative_Throws)
{
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(mTrajectory, mIndexMap, -0.1);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests,
    NonExistingJointInIndexMap_Throws)
{
  std::map<std::string, size_t> indexMap;
  indexMap.insert(std::make_pair("Joint0", 0));
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(mTrajectory, indexMap, mTimestep);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests,
    IndexMapHasDuplicateElements_Throws)
{
  std::map<std::string, size_t> indexMap;
  indexMap.insert(std::make_pair("Joint1", 0));
  indexMap.insert(std::make_pair("Joint2", 0));

  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(mTrajectory2DOF, indexMap, mTimestep);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, EmptyIndexMap_Throws)
{
  std::map<std::string, size_t> indexMap;
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(mTrajectory, indexMap, mTimestep);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, SkeletonHasUnsupportedJoint_Throws)
{
  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<BallJoint>();
  auto space = std::make_shared<MetaSkeletonStateSpace>(skeleton);

  auto trajectory = std::make_shared<Spline>(space, 0.0);
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(trajectory, mIndexMap, mTimestep);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, TrajectoryHasCorrectWaypoints)
{
  auto rosTrajectory = convertTrajectoryToRosTrajectory(
      mTrajectory, mIndexMap, mTimestep);

  for (auto it = mIndexMap.begin(); it != mIndexMap.end(); ++it)
    EXPECT_EQ(it->first, rosTrajectory.joint_names[it->second]);

  EXPECT_EQ(2, rosTrajectory.points.size());
  for (int i = 0; i < 2; ++i)
  {
    ASSERT_DOUBLE_EQ(mTimestep*i, rosTrajectory.points[i].time_from_start.toSec());

    auto state = mStateSpace->createState();
    Eigen::VectorXd values;

    mTrajectory->evaluate(mTimestep*i, state);
    mStateSpace->convertStateToPositions(state, values);

    EXPECT_EIGEN_EQUAL(values,
        make_vector(rosTrajectory.points[i].positions[0]), kTolerance);
  }
  ASSERT_DOUBLE_EQ(mTimestep*(rosTrajectory.points.size()-1),
      mTrajectory->getEndTime());

  double timestep = 0.01;
  auto rosTrajectory2 = convertTrajectoryToRosTrajectory(
      mTrajectory, mIndexMap, timestep);
  
  EXPECT_EQ(11, rosTrajectory2.points.size());
  for (int i = 0; i < 11; ++i)
  {
    ASSERT_DOUBLE_EQ(timestep*i,
        rosTrajectory2.points[i].time_from_start.toSec());

    auto state = mStateSpace->createState();
    Eigen::VectorXd values;

    mTrajectory->evaluate(timestep*i, state);
    mStateSpace->convertStateToPositions(state, values);
    EXPECT_EIGEN_EQUAL(values,
        make_vector(rosTrajectory2.points[i].positions[0]), kTolerance);

    for (auto it = mIndexMap.begin(); it != mIndexMap.end(); ++it)
      EXPECT_EQ(it->first, rosTrajectory2.joint_names[it->second]);
  }
  ASSERT_DOUBLE_EQ(timestep*(rosTrajectory2.points.size()-1),
      mTrajectory->getEndTime());

}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, DifferentOrdering)
{
  auto rosTrajectory = convertTrajectoryToRosTrajectory(
      mTrajectory2DOF, mIndexMap2DOF, mTimestep);

  for (auto it = mIndexMap.begin(); it != mIndexMap.end(); ++it)
    EXPECT_EQ(it->first, rosTrajectory.joint_names[it->second]);

  EXPECT_EQ(2, rosTrajectory.points.size());
  for (int i = 0; i < 2; ++i)
  {
    ASSERT_DOUBLE_EQ(mTimestep*i,
        rosTrajectory.points[i].time_from_start.toSec());

    auto state = mStateSpace2DOF->createState();
    Eigen::VectorXd values;

    mTrajectory2DOF->evaluate(mTimestep*i, state);
    mStateSpace2DOF->convertStateToPositions(state, values);

    EXPECT_EIGEN_EQUAL(values,
        make_vector(rosTrajectory.points[i].positions[0],
                    rosTrajectory.points[i].positions[1]), kTolerance);
  }

  ASSERT_DOUBLE_EQ(mTimestep*(rosTrajectory.points.size()-1),
      mTrajectory->getEndTime());
}

