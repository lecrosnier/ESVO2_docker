#ifndef ESVO2_CORE_CONTAINER_TIMESURFACEOBSERVATION_H
#define ESVO2_CORE_CONTAINER_TIMESURFACEOBSERVATION_H

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <kindr/minimal/quat-transformation.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <esvo2_core/tools/TicToc.h>
#include <esvo2_core/tools/utils.h>

//#define TIME_SURFACE_OBSERVATION_LOG
namespace esvo2_core
{
using namespace tools;
namespace container
{
struct TimeSurfaceObservation
{
  TimeSurfaceObservation(
    cv_bridge::CvImagePtr &left,
    cv_bridge::CvImagePtr &right,
    Transformation &tr,
    size_t id,
    bool bCalcTsGradient = false)
    : tr_(tr),
      id_(id)
  {
    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);

    if (bCalcTsGradient)
    {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      // cv::Mat cv_dTS_du_left_, cv_dTS_dv_left_;
      cv::Sobel(left->image, cv_dTS_du_left_, CV_64F, 1, 0);
      cv::Sobel(left->image, cv_dTS_dv_left_, CV_64F, 0, 1);
      cv::cv2eigen(cv_dTS_du_left_, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left_, dTS_dv_left_);
#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
    bSubNegaTS_ = false;
  }

  // override version without transformation and with negative TS.
  TimeSurfaceObservation(
    cv_bridge::CvImagePtr &left,
    cv_bridge::CvImagePtr &negative,
    cv_bridge::CvImagePtr &negative_dx,
    cv_bridge::CvImagePtr &negative_dy,
    size_t id,
    bool bCalcTsGradient = false)
    : id_(id)
  {
    cvImagePtr_left_ = left;
    // // used for multi-level pyramid
    // cv_TS_negative_left_ = negative->image;
    // cv_dTS_negative_du_left_ = negative_dx->image;
    // cv_dTS_negative_dv_left_ = negative_dy->image;

    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(negative_dx->image, dTS_negative_du_left_);
    cv::cv2eigen(negative_dy->image, dTS_negative_dv_left_);
    cv::cv2eigen(negative->image, TS_negative_left_);
    
    if (bCalcTsGradient)
    {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      cv::Sobel(left->image, cv_dTS_du_left_, CV_64F, 1, 0);
      cv::Sobel(left->image, cv_dTS_dv_left_, CV_64F, 0, 1);
      cv::cv2eigen(cv_dTS_du_left_, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left_, dTS_dv_left_);
#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
    bSubNegaTS_ = true;
  }

  // override version without initializing the transformation in the constructor.
  TimeSurfaceObservation(
    cv_bridge::CvImagePtr &left,
    cv_bridge::CvImagePtr &right,
    size_t id,
    bool bCalcTsGradient = false)
    : id_(id)
  {
    cvImagePtr_left_ = left;
    cvImagePtr_right_ = right;
    cv::cv2eigen(left->image, TS_left_);
    cv::cv2eigen(right->image, TS_right_);
    
    if (bCalcTsGradient)
    {
#ifdef TIME_SURFACE_OBSERVATION_LOG
      TicToc tt;
      tt.tic();
#endif
      // cv::Mat cv_dTS_du_left_, cv_dTS_dv_left_;
      cv::Mat cv_dTS_du_right, cv_dTS_dv_right;
      cv::Sobel(left->image, cv_dTS_du_left_, CV_64F, 1, 0);
      cv::Sobel(left->image, cv_dTS_dv_left_, CV_64F, 0, 1);

      cv::cv2eigen(cv_dTS_du_left_, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left_, dTS_dv_left_);

#ifdef TIME_SURFACE_OBSERVATION_LOG
      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Sobel computation (" << id_ << ") takes " << tt.toc() << " ms.";
#endif
    }
    bSubNegaTS_ = true;
  }

  TimeSurfaceObservation(
    cv_bridge::CvImagePtr &left,
    cv_bridge::CvImagePtr &right,
    cv_bridge::CvImagePtr &AA_map,
    cv_bridge::CvImagePtr &negative,
    cv_bridge::CvImagePtr &negative_dx,
    cv_bridge::CvImagePtr &negative_dy,
    size_t id,
    bool bCalcTsGradient = false)
    : id_(id)
  {
    // Keep the images and convert on demand: this constructor runs for every
    // time surface the mapping node receives (50 Hz on the EVK4 rig), while
    // mapping uses only the observations dataTransferring() selects (~20 Hz).
    // Converting six 1280x720 frames into double matrices for all of them cost
    // 17-19 ms per frame and put the node behind the stream. ensureEigen()
    // must be called before any of the matrices below is read.
    cvImagePtr_left_ = left;
    cvImagePtr_right_ = right;
    cvImagePtr_AA_map_ = AA_map;
    cvImagePtr_negative_ = negative;
    cvImagePtr_negative_dx_ = negative_dx;
    cvImagePtr_negative_dy_ = negative_dy;
    bCalcTsGradient_ = bCalcTsGradient;
    bEigenReady_ = false;
    bSubNegaTS_ = false;
  }

  // Fill the Eigen views of the images this observation was built from. Cheap
  // and idempotent after the first call; a no-op for observations whose
  // matrices were set directly.
  inline void ensureEigen()
  {
    if (bEigenReady_)
      return;
    bEigenReady_ = true;
    if (cvImagePtr_left_ == nullptr || cvImagePtr_right_ == nullptr)
      return;

    cv::cv2eigen(cvImagePtr_left_->image, TS_left_);
    cv::cv2eigen(cvImagePtr_right_->image, TS_right_);
    if (cvImagePtr_AA_map_ != nullptr)
      cv::cv2eigen(cvImagePtr_AA_map_->image, AA_map_);
    if (cvImagePtr_negative_dx_ != nullptr)
      cv::cv2eigen(cvImagePtr_negative_dx_->image, dTS_negative_du_left_);
    if (cvImagePtr_negative_dy_ != nullptr)
      cv::cv2eigen(cvImagePtr_negative_dy_->image, dTS_negative_dv_left_);
    if (cvImagePtr_negative_ != nullptr)
      cv::cv2eigen(cvImagePtr_negative_->image, TS_negative_left_);

    if (bCalcTsGradient_)
    {
      cv::Sobel(cvImagePtr_left_->image, cv_dTS_du_left_, CV_64F, 1, 0);
      cv::Sobel(cvImagePtr_left_->image, cv_dTS_dv_left_, CV_64F, 0, 1);
      cv::cv2eigen(cv_dTS_du_left_, dTS_du_left_);
      cv::cv2eigen(cv_dTS_dv_left_, dTS_dv_left_);
      cv::Sobel(cvImagePtr_right_->image, cv_dTS_du_right_, CV_64F, 1, 0);
      cv::Sobel(cvImagePtr_right_->image, cv_dTS_dv_right_, CV_64F, 0, 1);
      cv::cv2eigen(cv_dTS_du_right_, dTS_du_right_);
      cv::cv2eigen(cv_dTS_dv_right_, dTS_dv_right_);
    }
  }

  TimeSurfaceObservation()
  {};

  inline bool isEmpty()
  {
    // An observation built from images is not empty even before ensureEigen().
    if(cvImagePtr_left_ != nullptr && cvImagePtr_right_ != nullptr)
      return false;
    if(TS_left_.rows() == 0 || TS_left_.cols() == 0 || TS_right_.rows() == 0 || TS_right_.cols() == 0)
      return true;
    else
      return false;
  }

  inline void setTransformation(Transformation &tr)
  {
    tr_ = tr;
  }

  inline void setOriTransformation(Transformation &tr)
  {
    tr_ori_ = tr;
  }

  inline void GaussianBlurTS(size_t kernelSize)
  {
    cv::Mat mat_left_, mat_right_, mat_last_;
    cv::GaussianBlur(cvImagePtr_left_->image, mat_left_,
                     cv::Size(kernelSize, kernelSize), 0.0);
    cv::GaussianBlur(cvImagePtr_right_->image, mat_right_,
                     cv::Size(kernelSize, kernelSize), 0.0);
    cv::cv2eigen(mat_left_, TS_left_);
    cv::cv2eigen(mat_right_, TS_right_);

    if(TS_last_.rows() != 0 && TS_last_.cols() != 0 && TS_last_.rows() != 0 && TS_last_.cols() != 0)
    {
      cv::GaussianBlur(cvImagePtr_last_->image, mat_last_,
                    cv::Size(kernelSize, kernelSize), 0.0);
      cv::cv2eigen(mat_last_, TS_last_);
    }
  }

  inline void getTimeSurfaceNegative(size_t kernelSize)
  {
    Eigen::MatrixXd ceilMat(TS_left_.rows(), TS_left_.cols());
    ceilMat.setConstant(255.0);
    if (kernelSize > 0)
    {
      cv::Mat mat_left_;
      cv::GaussianBlur(cvImagePtr_left_->image, mat_left_,
                       cv::Size(kernelSize, kernelSize), 0.0);
      cv::cv2eigen(mat_left_, TS_blurred_left_);
      TS_negative_left_ = ceilMat - TS_blurred_left_;
    }
    else
    {
      TS_negative_left_ = ceilMat - TS_left_;
    }
  }

  inline void computeTsNegativeGrad()
  {
    cv::Mat cv_TS_flipped_left;
    cv::eigen2cv(TS_negative_left_, cv_TS_flipped_left);

    cv::Mat cv_dFlippedTS_du_left, cv_dFlippedTS_dv_left;
    cv::Sobel(cv_TS_flipped_left, cv_dFlippedTS_du_left, CV_64F, 1, 0);
    cv::Sobel(cv_TS_flipped_left, cv_dFlippedTS_dv_left, CV_64F, 0, 1);

    cv::cv2eigen(cv_dFlippedTS_du_left, dTS_negative_du_left_);
    cv::cv2eigen(cv_dFlippedTS_dv_left, dTS_negative_dv_left_);
  }

  // Single-precision copies of TS_left_ / TS_right_, read only by the mapping
  // node's float path (MAPPING_FLOAT); tracking never reads them. Refreshed by
  // EventBM::createMatchProblem after its optional blur, so they always hold
  // exactly what the double path reads (the surfaces are 8-bit, so the cast is exact).
  inline void refreshFloatMirrors()
  {
    TS_left_f_ = TS_left_.cast<float>();
    TS_right_f_ = TS_right_.cast<float>();
  }
  Eigen::MatrixXf TS_left_f_, TS_right_f_;

  Eigen::MatrixXd TS_left_, TS_right_, TS_last_, AA_map_, TS_last_du, TS_last_dv;
  Eigen::MatrixXd TS_blurred_left_;
  Eigen::MatrixXd TS_negative_left_;
  cv_bridge::CvImagePtr cvImagePtr_left_, cvImagePtr_right_, cvImagePtr_last_, cvImagePtr_AA_map_;
  cv_bridge::CvImagePtr cvImagePtr_negative_, cvImagePtr_negative_dx_, cvImagePtr_negative_dy_;
  // false only between the six-image constructor and ensureEigen()
  bool bEigenReady_ = true;
  bool bCalcTsGradient_ = false;
  Transformation tr_, tr_last_, tr_ori_;
  Eigen::MatrixXd dTS_du_left_, dTS_dv_left_, dTS_du_right_, dTS_dv_right_;
  cv::Mat cv_dTS_du_left_, cv_dTS_dv_left_, cv_dTS_du_right_, cv_dTS_dv_right_;
  Eigen::MatrixXd dTS_negative_du_left_, dTS_negative_dv_left_;
  cv::Mat cv_TS_negative_left_, cv_dTS_negative_du_left_, cv_dTS_negative_dv_left_;
  size_t id_;
  cv::Mat x_grad_, y_grad_;
  bool bSubNegaTS_;
  EventQueue events_;
};

struct ROSTimeCmp
{
  bool operator()(const ros::Time &a, const ros::Time &b) const
  {
    return a.toNSec() < b.toNSec();
  }
};

using TimeSurfaceHistory = std::map<ros::Time, TimeSurfaceObservation, ROSTimeCmp>;
using StampedTimeSurfaceObs = std::pair<ros::Time, TimeSurfaceObservation>;
using constStampedTimeSurfaceObs = std::pair<const ros::Time, TimeSurfaceObservation>;

inline static TimeSurfaceHistory::iterator TSHistory_lower_bound(TimeSurfaceHistory &ts_history, ros::Time &t)
{
  return std::lower_bound(ts_history.begin(), ts_history.end(), t,
                          [](const std::pair<ros::Time, TimeSurfaceObservation> &tso, const ros::Time &t) {
                            return tso.first.toSec() < t.toSec();
                          });
}

inline static TimeSurfaceHistory::iterator TSHistory_upper_bound(TimeSurfaceHistory &ts_history, ros::Time &t)
{
  return std::upper_bound(ts_history.begin(), ts_history.end(), t,
                          [](const ros::Time &t, const std::pair<ros::Time, TimeSurfaceObservation> &tso) {
                            return t.toSec() < tso.first.toSec();
                          });
}
}
}

#endif //ESVO2_CORE_CONTAINER_TIMESURFACEOBSERVATION_H
