#include <gtest/gtest.h>

#include <limits>

#include <plan_manage/goal_arrival_detector.h>

namespace ego_planner
{

TEST(GoalArrivalDetector, KeepsTrackingClosestDistanceWhileApproaching)
{
  GoalArrivalDetector detector;
  detector.configure(0.5, 0.2);

  EXPECT_FALSE(detector.update(0.70));
  EXPECT_FALSE(detector.update(0.45));
  EXPECT_FALSE(detector.update(0.30));
  EXPECT_FALSE(detector.update(0.12));
  EXPECT_DOUBLE_EQ(0.12, detector.closestDistance());
  EXPECT_FALSE(detector.reached());
}

TEST(GoalArrivalDetector, ReachesAfterMovingTwentyCentimetersFromClosestDistance)
{
  GoalArrivalDetector detector;
  detector.configure(0.5, 0.2);

  EXPECT_FALSE(detector.update(0.35));
  EXPECT_FALSE(detector.update(0.10));
  EXPECT_FALSE(detector.update(0.29));
  EXPECT_TRUE(detector.update(0.301));
  EXPECT_TRUE(detector.reached());
}

TEST(GoalArrivalDetector, CanReachAfterLeavingArrivalRadius)
{
  GoalArrivalDetector detector;
  detector.configure(0.5, 0.2);

  EXPECT_FALSE(detector.update(0.49));
  EXPECT_TRUE(detector.update(0.70));
}

TEST(GoalArrivalDetector, DoesNotArmBeforeEnteringArrivalRadius)
{
  GoalArrivalDetector detector;
  detector.configure(0.5, 0.2);

  EXPECT_FALSE(detector.update(0.70));
  EXPECT_FALSE(detector.update(0.95));
  EXPECT_FALSE(detector.reached());
}

TEST(GoalArrivalDetector, ResetStartsASeparateGoalHistory)
{
  GoalArrivalDetector detector;
  detector.configure(0.5, 0.2);

  EXPECT_FALSE(detector.update(0.10));
  detector.reset();
  EXPECT_FALSE(detector.update(0.31));
  EXPECT_FALSE(detector.reached());
}

TEST(GoalArrivalDetector, IgnoresInvalidDistance)
{
  GoalArrivalDetector detector;
  detector.configure(0.5, 0.2);

  EXPECT_FALSE(detector.update(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(detector.reached());
}

} // namespace ego_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
