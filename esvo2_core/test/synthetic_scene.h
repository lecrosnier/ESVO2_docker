#ifndef ESVO2_CORE_TEST_SYNTHETIC_SCENE_H
#define ESVO2_CORE_TEST_SYNTHETIC_SCENE_H

// A textured, rectified stereo pair at the EVK4 rig's resolution, where every
// left pixel reappears 40 px to the left in the right image, plus a grid of
// candidate events. Surfaces are 8-bit and blurred, like the node's.

#include <memory>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <esvo2_core/container/CameraSystem.h>
#include <esvo2_core/container/TimeSurfaceObservation.h>

struct SyntheticScene
{
  static constexpr int kDisparity = 40;
  esvo2_core::container::CameraSystem::Ptr cam;
  std::unique_ptr<esvo2_core::container::constStampedTimeSurfaceObs> obs;
  std::vector<dvs_msgs::Event> events;
  std::vector<dvs_msgs::Event *> ptrs;

  SyntheticScene()
  {
    using namespace esvo2_core::container;
    cam = std::make_shared<CameraSystem>(std::string(ESVO2_CORE_SOURCE_DIR) + "/calib/evk4_stereo", false);
    const int W = cam->cam_left_ptr_->width_, H = cam->cam_left_ptr_->height_;
    cv::Mat noise(H, W + kDisparity, CV_8UC1), tex;
    cv::RNG rng(7);
    rng.fill(noise, cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(noise, tex, cv::Size(5, 5), 0.0);
    const cv::Mat left = tex(cv::Rect(0, 0, W, H)).clone();
    const cv::Mat right = tex(cv::Rect(kDisparity, 0, W, H)).clone(); // right(y,x) = left(y,x+40)

    obs.reset(new constStampedTimeSurfaceObs(ros::Time(0), TimeSurfaceObservation()));
    cv::cv2eigen(left, obs->second.TS_left_);
    cv::cv2eigen(right, obs->second.TS_right_);
    obs->second.tr_.setIdentity();

    for (int y = 60; y < H - 60; y += 23)
      for (int x = 100; x < W - 100; x += 29)
      {
        dvs_msgs::Event e;
        e.x = x;
        e.y = y;
        e.ts = ros::Time(1.0);
        e.polarity = true;
        events.push_back(e);
      }
    for (dvs_msgs::Event &e : events)
      ptrs.push_back(&e);
  }
};

#endif // ESVO2_CORE_TEST_SYNTHETIC_SCENE_H
