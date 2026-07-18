#include <gtest/gtest.h>

#include "controller.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;

void expectHorizontalAccelerationDirection(double acceleration_x, double yaw)
{
  const Eigen::Vector3d acceleration(acceleration_x, 0.0, kGravity);
  const Eigen::Quaterniond attitude =
      LinearControl::attitudeFromAccelerationAndYaw(acceleration, yaw, kGravity);
  const Eigen::Vector3d body_z_world = attitude * Eigen::Vector3d::UnitZ();

  EXPECT_GT(body_z_world.x() * acceleration_x, 0.0);
  EXPECT_NEAR(body_z_world.y(), 0.0, 1e-12);
}
} // namespace

TEST(ControllerAttitude, PreservesWorldAccelerationAcrossYaw)
{
  expectHorizontalAccelerationDirection(1.0, 0.0);
  expectHorizontalAccelerationDirection(1.0, kPi / 2.0);
  expectHorizontalAccelerationDirection(1.0, kPi);
  expectHorizontalAccelerationDirection(-1.0, kPi);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
