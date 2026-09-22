#include <gtest/gtest.h>
#include <limits>
#include <esvo2_core/core/DepthProblem.h>

using namespace esvo2_core;

// The temporal (left-to-last) depth problem warps with T_last_now_, the pose
// of the current frame in the last frame. setProblem must set it for that
// problem; it used to be set only for the static problem, which never reads it.
TEST(DepthProblemTemporal, SetProblemSetsTheLastToNowTransform)
{
  CameraSystem::Ptr cam = std::make_shared<CameraSystem>(
    std::string(ESVO2_CORE_SOURCE_DIR) + "/calib/evk4_stereo", false);
  core::DepthProblemConfig::Ptr cfg = std::make_shared<core::DepthProblemConfig>(
    9, 5, "Tdist", 2.182, 17.277, 1, 5, 8, 8);
  core::DepthProblem problem(cfg, cam);
  problem.T_last_now_.setConstant(std::numeric_limits<double>::quiet_NaN());

  constStampedTimeSurfaceObs obs(ros::Time(0), container::TimeSurfaceObservation());
  obs.second.tr_ = Transformation(Eigen::Vector3d(0.30, -0.02, 0.05),
                                  Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY())));
  obs.second.tr_last_ = Transformation(Eigen::Vector3d(0.05, 0.01, 0.00),
                                       Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY())));
  Eigen::Vector2d coor(640, 360);
  Eigen::Matrix4d T_world_virtual = obs.second.tr_.getTransformationMatrix();
  problem.setProblem(coor, T_world_virtual, &obs, false);

  const Eigen::Matrix4d expected = obs.second.tr_last_.getTransformationMatrix().inverse() *
                                   obs.second.tr_.getTransformationMatrix();
  EXPECT_TRUE(problem.T_last_now_.isApprox(expected, 1e-12)) << problem.T_last_now_;
}

// A point straight ahead, with the rig translated sideways between the frames,
// must warp to a different pixel in the last frame (by the parallax of that translation).
TEST(DepthProblemTemporal, WarpingUsesTheLastToNowTransform)
{
  CameraSystem::Ptr cam = std::make_shared<CameraSystem>(
    std::string(ESVO2_CORE_SOURCE_DIR) + "/calib/evk4_stereo", false);
  core::DepthProblemConfig::Ptr cfg = std::make_shared<core::DepthProblemConfig>(
    9, 5, "Tdist", 2.182, 17.277, 1, 5, 8, 8);
  core::DepthProblem problem(cfg, cam);

  constStampedTimeSurfaceObs obs(ros::Time(0), container::TimeSurfaceObservation());
  obs.second.tr_ = Transformation(Eigen::Vector3d(0.20, 0, 0), Eigen::Quaterniond::Identity());
  obs.second.tr_last_.setIdentity();
  Eigen::Vector2d coor(640, 360);
  Eigen::Matrix4d T_world_virtual = obs.second.tr_.getTransformationMatrix();
  problem.setProblem(coor, T_world_virtual, &obs, false);

  Eigen::Vector2d x1_s, x2_s;
  const double invDepth = 0.5; // 2 m
  ASSERT_TRUE(problem.warping(coor, invDepth, problem.vT_left_virtual_[0], x1_s, x2_s));
  const double fx = cam->cam_left_ptr_->P_(0, 0);
  // Now = last + 0.2 m along x, so a point at 2 m appears fx*0.2/2 px further right in the last frame.
  EXPECT_NEAR(x2_s(0) - x1_s(0), fx * 0.20 * invDepth, 1e-6);
  EXPECT_NEAR(x2_s(1), x1_s(1), 1e-6);
}
