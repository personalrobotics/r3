#include <dart/dynamics/dynamics.hpp>
#include <gtest/gtest.h>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>

using aikido::statespace::dart::MetaSkeletonStateSaver;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;

class MetaSkeletonStateSaverTest : public testing::Test
{
public:
  void SetUp()
  {
    mSkeleton = Skeleton::create();
    mSkeleton->createJointAndBodyNodePair<RevoluteJoint>();

    mSkeleton->setPosition(0, 1.);
    mSkeleton->setPositionLowerLimit(0, 0.);
    mSkeleton->setPositionUpperLimit(0, 3.);
  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;
};

TEST_F(MetaSkeletonStateSaverTest, MetaSkeletonStateSpaceReturnsToOriginal)
{
  EXPECT_DOUBLE_EQ(0., mSkeleton->getPositionLowerLimit(0));
  EXPECT_DOUBLE_EQ(1., mSkeleton->getPosition(0));
  EXPECT_DOUBLE_EQ(3., mSkeleton->getPositionUpperLimit(0));

  {
    auto saver = MetaSkeletonStateSaver(mSkeleton);
    DART_UNUSED(saver);

    mSkeleton->setPositionLowerLimit(0, 1.);
    mSkeleton->setPositionUpperLimit(0, 5.);
    mSkeleton->setPosition(0, 4.);

    EXPECT_DOUBLE_EQ(1., mSkeleton->getPositionLowerLimit(0));
    EXPECT_DOUBLE_EQ(4., mSkeleton->getPosition(0));
    EXPECT_DOUBLE_EQ(5., mSkeleton->getPositionUpperLimit(0));
  }

  EXPECT_DOUBLE_EQ(0., mSkeleton->getPositionLowerLimit(0));
  EXPECT_DOUBLE_EQ(1., mSkeleton->getPosition(0));
  EXPECT_DOUBLE_EQ(3., mSkeleton->getPositionUpperLimit(0));
}

TEST_F(MetaSkeletonStateSaverTest, Flags_None)
{
  EXPECT_DOUBLE_EQ(0., mSkeleton->getPositionLowerLimit(0));
  EXPECT_DOUBLE_EQ(1., mSkeleton->getPosition(0));
  EXPECT_DOUBLE_EQ(3., mSkeleton->getPositionUpperLimit(0));

  {
    auto saver = MetaSkeletonStateSaver(
        mSkeleton, MetaSkeletonStateSaver::Options::NONE);
    DART_UNUSED(saver);

    mSkeleton->setPositionLowerLimit(0, 1.);
    mSkeleton->setPositionUpperLimit(0, 5.);
    mSkeleton->setPosition(0, 4.);

    EXPECT_DOUBLE_EQ(1., mSkeleton->getPositionLowerLimit(0));
    EXPECT_DOUBLE_EQ(4., mSkeleton->getPosition(0));
    EXPECT_DOUBLE_EQ(5., mSkeleton->getPositionUpperLimit(0));
  }

  EXPECT_DOUBLE_EQ(1., mSkeleton->getPositionLowerLimit(0));
  EXPECT_DOUBLE_EQ(4., mSkeleton->getPosition(0));
  EXPECT_DOUBLE_EQ(5., mSkeleton->getPositionUpperLimit(0));
}

TEST_F(MetaSkeletonStateSaverTest, Flags_PositionsOnly)
{
  EXPECT_DOUBLE_EQ(0., mSkeleton->getPositionLowerLimit(0));
  EXPECT_DOUBLE_EQ(1., mSkeleton->getPosition(0));
  EXPECT_DOUBLE_EQ(3., mSkeleton->getPositionUpperLimit(0));

  {
    auto saver = MetaSkeletonStateSaver(
        mSkeleton, MetaSkeletonStateSaver::Options::POSITIONS);
    DART_UNUSED(saver);

    mSkeleton->setPositionLowerLimit(0, 1.);
    mSkeleton->setPositionUpperLimit(0, 5.);
    mSkeleton->setPosition(0, 4.);

    EXPECT_DOUBLE_EQ(1., mSkeleton->getPositionLowerLimit(0));
    EXPECT_DOUBLE_EQ(4., mSkeleton->getPosition(0));
    EXPECT_DOUBLE_EQ(5., mSkeleton->getPositionUpperLimit(0));
  }

  EXPECT_DOUBLE_EQ(1., mSkeleton->getPositionLowerLimit(0));
  EXPECT_DOUBLE_EQ(1., mSkeleton->getPosition(0));
  EXPECT_DOUBLE_EQ(5., mSkeleton->getPositionUpperLimit(0));
}

TEST_F(MetaSkeletonStateSaverTest, Flags_PositionLimitsOnly)
{
  EXPECT_DOUBLE_EQ(0., mSkeleton->getPositionLowerLimit(0));
  EXPECT_DOUBLE_EQ(1., mSkeleton->getPosition(0));
  EXPECT_DOUBLE_EQ(3., mSkeleton->getPositionUpperLimit(0));

  {
    auto saver = MetaSkeletonStateSaver(
        mSkeleton, MetaSkeletonStateSaver::Options::POSITION_LIMITS);
    DART_UNUSED(saver);

    mSkeleton->setPositionLowerLimit(0, 1.);
    mSkeleton->setPositionUpperLimit(0, 5.);
    mSkeleton->setPosition(0, 2.);

    EXPECT_DOUBLE_EQ(1., mSkeleton->getPositionLowerLimit(0));
    EXPECT_DOUBLE_EQ(2., mSkeleton->getPosition(0));
    EXPECT_DOUBLE_EQ(5., mSkeleton->getPositionUpperLimit(0));
  }

  EXPECT_DOUBLE_EQ(0., mSkeleton->getPositionLowerLimit(0));
  EXPECT_DOUBLE_EQ(2., mSkeleton->getPosition(0));
  EXPECT_DOUBLE_EQ(3., mSkeleton->getPositionUpperLimit(0));
}
