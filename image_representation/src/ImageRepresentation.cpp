#include <image_representation/ImageRepresentation.h>
#include <chrono>
#include <map>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <std_msgs/Float32.h>
#include <glog/logging.h>

#include <cmath>
#include <vector>

// #define ESVIO_REPRESENTATION_LOG

namespace image_representation
{
  // exp() of this is 0 in float: a pixel that has never fired.
  static const float kTsMapEmpty = -200.0f;
  // Time surface lookup table: 256 steps per decay constant, up to 6 of them.
  static const int kLutPerDecay = 256;
  static const int kLutSize = 6 * kLutPerDecay;
  // An event this much older than the previous one means the clock went back.
  static const double kBackwardJumpS = 1.0;

  ImageRepresentation::ImageRepresentation(ros::NodeHandle &nh, ros::NodeHandle nh_private) : nh_(nh)
  {
    // setup subscribers and publishers
    // queue_size 0 is unbounded in roscpp; the backlog reached 11GB when generation held data_mutex_ too long.
    int event_sub_queue_size;
    nh_private.param<int>("event_sub_queue_size", event_sub_queue_size, 10000);
    event_sub_ = nh_.subscribe("events", event_sub_queue_size, &ImageRepresentation::eventsCallback, this);
    image_transport::ImageTransport it_(nh_);
    nh_private.param<bool>("is_left", is_left_, true);    // is left camera
    if (is_left_)   
    {
      image_representation_pub_TS_ = it_.advertise("image_representation_TS_", 5);                   // for block matching
      image_representation_pub_negative_TS_ = it_.advertise("image_representation_negative_TS_", 5); // negative OS-TS for 3D-2D regristration
      image_representation_pub_AA_frequency_ = it_.advertise("image_representation_AA_frequency_", 5);
      image_representation_pub_AA_mat_ = it_.advertise("image_representation_AA_mat_", 5); // for temporal stereo matching
      dx_image_pub_ = it_.advertise("dx_image_pub_", 5);                                   // gradient map for point sampling
      dy_image_pub_ = it_.advertise("dy_image_pub_", 5);
    }
    else
    {
      image_representation_pub_TS_ = it_.advertise("image_representation_TS_", 5);
    }
    nh_private.param<bool>("use_sim_time", bUse_Sim_Time_, true);

    // system variables
    int representation_mode;
    nh_private.param<int>("representation_mode", representation_mode, 0);
    nh_private.param<int>("median_blur_kernel_size", median_blur_kernel_size_, 1);
    nh_private.param<int>("blur_size", blur_size_, 7);
    nh_private.param<int>("max_event_queue_len", max_event_queue_length_, 20);
   
    representation_mode_ = (RepresentationMode)representation_mode;
       
    // rectify variables
    bCamInfoAvailable_ = false;
    bSensorInitialized_ = false;
    sensor_size_ = cv::Size(0, 0);

    // local parameters
    nh_private.param<bool>("use_stereo_cam", bUseStereoCam_, true);
    nh_private.param<double>("decay_ms", decay_ms_, 30);
    decay_sec_ = decay_ms_ / 1000.0;
    nh_private.param<int>("x_patches", x_patches_, 8); // patch of AA
    nh_private.param<int>("y_patches", y_patches_, 6);
    nh_private.param<int>("generation_rate_hz", generation_rate_hz_, 100);
    // The AA map (and the mapping candidates sampled from it) otherwise covers only
    // the events since the previous render, i.e. 1/generation_rate_hz seconds, so a
    // higher rate thins it out. A fixed window keeps it independent of the rate.
    // OpenCV runs its parallel_for over all cores by default. With the mapping,
    // tracking and both representation nodes doing that at once the pools
    // oversubscribe the machine and every full-frame op slows down several
    // fold. 0 keeps OpenCV's default.
    int opencv_threads;
    nh_private.param<int>("opencv_threads", opencv_threads, 0);
    if (opencv_threads > 0)
      cv::setNumThreads(opencv_threads);

    double aa_window_ms;
    nh_private.param<double>("aa_window_ms", aa_window_ms, 0.0);
    aa_window_s_ = aa_window_ms / 1000.0;
    nh_private.param("calibInfoDir", calibInfoDir_, std::string("path is not given"));
    if (!loadCalibInfo(calibInfoDir_, is_left_))
    {
      ROS_ERROR("Load Calib Info Error!!!  Given path is: %s", calibInfoDir_.c_str());
    }

    if(is_left_)
      LOG(INFO) << "\33[32m" << "Left event representation node is up " << "\33[0m";
    else
      LOG(INFO) << "\33[32m" << "Right event representation node is up " << "\33[0m";

    // start generation
    std::thread GenerationThread(&ImageRepresentation::GenerationLoop, this);
    GenerationThread.detach();
  }

  ImageRepresentation::~ImageRepresentation()
  {
    dx_image_pub_.shutdown();
    dy_image_pub_.shutdown();
    image_representation_pub_TS_.shutdown();
    image_representation_pub_negative_TS_.shutdown();
    image_representation_pub_AA_frequency_.shutdown();
    image_representation_pub_AA_mat_.shutdown();
  }

  void ImageRepresentation::init(int width, int height)
  {
    sensor_size_ = cv::Size(width, height);
    bSensorInitialized_ = true;
    ROS_INFO("Sensor size: (%d x %d)", sensor_size_.width, sensor_size_.height);

    representation_TS_ = cv::Mat::zeros(sensor_size_, CV_32F);
    representation_AA_ = cv::Mat::zeros(sensor_size_, CV_8U);

    //Access to Eigen matrix is faster than cv::Mat
    TS_temp_map = Eigen::MatrixXd::Constant(sensor_size_.height, sensor_size_.width, -10);
    ts_map_ = cv::Mat(sensor_size_, CV_32F, cv::Scalar(kTsMapEmpty));
    ts_work_ = cv::Mat(sensor_size_, CV_32F);
    ts_epoch_ = 0.0;
    vEvents_.reserve(5000000);
  }

  void ImageRepresentation::GenerationLoop()
  {
    ros::Rate r(generation_rate_hz_);
    while (ros::ok())
    {
      sync_time_ = ros::Time::now();
      {
        createImageRepresentationAtTime(sync_time_);
      }

      r.sleep();
    }
  }

  void ImageRepresentation::AA_thread(const std::vector<dvs_msgs::Event> &events, double external_t)
  {
    ros::Time external_sync_time(external_t);

    representation_AA_ = cv::Mat::zeros(sensor_size_, CV_8U);   //for temporal stereo matching
    cv::Mat AA_frequency = cv::Mat::zeros(sensor_size_, CV_8U);   //for point sampling

    std::vector<double> last_activity(x_patches_ * y_patches_, 0), event_activity(x_patches_ * y_patches_, 0), beta(x_patches_ * y_patches_, 0);
    std::vector<double> last_event_time(x_patches_ * y_patches_, 0);
    std::vector<bool> flag(x_patches_ * y_patches_, true);
    int flags = 0;
    double conv_thresh_ = 0.95; // convergence threshold
    std::vector<double> final_activity(x_patches_ * y_patches_, 0);
    std::vector<int> num(x_patches_ * y_patches_, 0);

    // std::vector<int> nums_temp(x_patches_ * y_patches_, 0);
    int nums_EQ = 0;
    // calculate the final activity by all events, also can be estimated by eq. 3 in the paper
    for (auto it = events.begin(); it != events.end(); it++)
    {
      dvs_msgs::Event e = *it;
      int y = e.y / (int)ceil((double)sensor_size_.height / (double)y_patches_);
      int x = e.x / (int)ceil((double)sensor_size_.width / (double)x_patches_);
      beta[y * x_patches_ + x] = 1 / (1 + final_activity[y * x_patches_ + x] * abs(e.ts.toSec() - last_event_time[y * x_patches_ + x])); // eq. 2
      if (y * x_patches_ + x >= x_patches_ * y_patches_)
        exit(-1);
      final_activity[y * x_patches_ + x] = beta[y * x_patches_ + x] * final_activity[y * x_patches_ + x] + 1; // eq. 1
      last_event_time[y * x_patches_ + x] = e.ts.toSec();
      // nums_temp[y * x_patches_ + x]++;
    }
    // for(int i = 0; i < x_patches_ * y_patches_; i++)
    // final_activity[i] = std::sqrt(1 / (0.01 / nums_temp[i]));  // eq. 3

    std::fill(beta.begin(), beta.end(), 0);
    std::fill(last_event_time.begin(), last_event_time.end(), 0);
    // Walk this cycle's events newest-first. (An older version started at the sync-time iterator itself, which is end() whenever every buffered event predates the sync time.)
    for (auto rit = events.rbegin(); rit != events.rend(); ++rit) // traverse events in reverse to accumulate the latest events
    {
      dvs_msgs::Event e = *rit;
      int y = e.y / (int)ceil((double)sensor_size_.height / (double)y_patches_);
      int x = e.x / (int)ceil((double)sensor_size_.width / (double)x_patches_);
      if (flag[y * x_patches_ + x] != true)
        continue;
      beta[y * x_patches_ + x] = 1 / (1 + event_activity[y * x_patches_ + x] * abs(e.ts.toSec() - last_event_time[y * x_patches_ + x])); // eq. 2
      event_activity[y * x_patches_ + x] = beta[y * x_patches_ + x] * event_activity[y * x_patches_ + x] + 1;                            // eq. 1
      last_event_time[y * x_patches_ + x] = e.ts.toSec();
      AA_frequency.at<uchar>(e.y, e.x)++;
      num[y * x_patches_ + x]++;
      if (AA_frequency.at<uchar>(e.y, e.x) >= 1)
        representation_AA_.at<uchar>(e.y, e.x) = 255;
      if (num[y * x_patches_ + x] >= 10) // each patch is checked for convergence once every ten events accumulated
      {
        if (last_activity[y * x_patches_ + x] != 0)
        {
          if ((abs(event_activity[y * x_patches_ + x] - final_activity[y * x_patches_ + x])) < conv_thresh_)
          {
            flag[y * x_patches_ + x] = false;
            flags++;
            if (flags == x_patches_ * y_patches_)
              break;
            else
              continue;
          }
        }
        last_activity[y * x_patches_ + x] = event_activity[y * x_patches_ + x];
        num[y * x_patches_ + x] = 0;
      }
    }

    //distortion correction
    cv::remap(representation_AA_, representation_AA_, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);

    cv_bridge::CvImage cv_AA_frequency, cv_AA_mat;
    cv_AA_frequency.encoding = "mono8";
    cv_AA_mat.encoding = "mono8";
    cv_AA_frequency.image = AA_frequency.clone();
    cv_AA_mat.image = representation_AA_.clone();
    cv_AA_frequency.header.stamp = external_sync_time;
    cv_AA_mat.header.stamp = external_sync_time;
    image_representation_pub_AA_frequency_.publish(cv_AA_frequency.toImageMsg());
    image_representation_pub_AA_mat_.publish(cv_AA_mat.toImageMsg());
  }

  // TS_img = 255 * exp((t_pixel - external_t) / decay_sec_), as 8-bit.
  // Single precision throughout, and the pixel times live in a cv::Mat so the
  // fill below is row-major (the Eigen map it replaced was column-major, and
  // eigen2cv copied the whole frame every cycle).
  void ImageRepresentation::renderTimeSurface(double external_t, int distance, cv::Mat &TS_img)
  {
    if (ts_epoch_ == 0.0)
      ts_epoch_ = external_t;
    // Rebase before float resolution at now_rel degrades: at 1e6 the spacing is
    // 0.0625, i.e. 1.25 ms with a 20 ms decay.
    double now_rel = (external_t - ts_epoch_) / decay_sec_;
    if (now_rel > 1e6)
    {
      cv::subtract(ts_map_, cv::Scalar(now_rel), ts_map_);
      cv::max(ts_map_, kTsMapEmpty, ts_map_);
      ts_epoch_ = external_t;
      now_rel = 0.0;
    }

    std::vector<dvs_msgs::Event>::iterator it = vBatch_.begin();
    for (int i = 0; i < distance; i++)
    {
      if (i > distance - 2)
        break;
      const dvs_msgs::Event &e = *(it + i);
      ts_map_.at<float>(e.y, e.x) = static_cast<float>((e.ts.toSec() - ts_epoch_) / decay_sec_);
    }

    // 255 * exp(ts - now) in one pass, from a table indexed by the age in units
    // of kLutPerDecay per decay constant. Beyond kLutSize the value rounds to 0
    // in 8 bits anyway (exp(-6) * 255 < 0.7).
    if (lut_.empty())
    {
      lut_.resize(kLutSize);
      for (int i = 0; i < kLutSize; i++)
        lut_[i] = cv::saturate_cast<uchar>(255.0 * std::exp(-static_cast<double>(i) / kLutPerDecay));
    }
    TS_img.create(ts_map_.size(), CV_8U);
    const float nowf = static_cast<float>(now_rel);
    for (int y = 0; y < ts_map_.rows; y++)
    {
      const float *src = ts_map_.ptr<float>(y);
      uchar *dst = TS_img.ptr<uchar>(y);
      for (int x = 0; x < ts_map_.cols; x++)
      {
        // A pixel newer than the render time cannot occur on a monotonic clock;
        // clamp anyway rather than index the table out of range.
        const int age = std::max(0, static_cast<int>((nowf - src[x]) * kLutPerDecay));
        dst[x] = age >= kLutSize ? 0 : lut_[age];
      }
    }
  }

  void ImageRepresentation::createImageRepresentationAtTime(const ros::Time &external_sync_time)
  {
    if (!bcreat_)
      return;
    else
      bcreat_ = false;
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (!bSensorInitialized_ || !bCamInfoAvailable_)
      return;
    
    //for AA generation
    cv::Mat filiter_image = cv::Mat::zeros(sensor_size_, CV_64F);
    cv::Mat rectangle_image = cv::Mat::zeros(cv::Size(80, 80), CV_8U);
    cv::Mat AA_frequency = cv::Mat::zeros(sensor_size_, CV_8U);

    if (representation_mode_ == Fast)
    {
      if (vEvents_.size() == 0)
        return;
      double external_t = external_sync_time.toSec();
      std::vector<dvs_msgs::Event>::iterator ptr_e = EventVector_lower_bound(vEvents_, external_t);
      int distance = std::distance(vEvents_.begin(), ptr_e);

      // Take this cycle's events out while holding data_mutex_, then build the
      // images without it. eventsCallback() needs the same mutex for every
      // message. Holding it for the whole TS/AA/Sobel computation (25 to 50 ms
      // on the left node, against a 40 ms period) starved the callback: event
      // messages queued up for 100+ ms and the TS rendered black.
      const bool timeJumped = bTimeJumped_;
      bTimeJumped_ = false;
      vBatch_.assign(vEvents_.begin(), ptr_e);
      clearEvents(distance, ptr_e);
      lock.unlock();
      if (timeJumped)
      {
        ts_map_.setTo(cv::Scalar(kTsMapEmpty));
        ts_epoch_ = 0.0;
        vAAWindow_.clear();
      }

      const std::vector<dvs_msgs::Event> *aa_events = &vBatch_;
      if (aa_window_s_ > 0)
      {
        vAAWindow_.insert(vAAWindow_.end(), vBatch_.begin(), vBatch_.end());
        auto keep = std::lower_bound(vAAWindow_.begin(), vAAWindow_.end(), external_t - aa_window_s_,
                                     [](const dvs_msgs::Event &e, double t) { return e.ts.toSec() < t; });
        vAAWindow_.erase(vAAWindow_.begin(), keep);
        aa_events = &vAAWindow_;
      }

      if (is_left_)   // generate AA and TS in parallel, just for left camera
      {
        std::thread thread0(&ImageRepresentation::AA_thread, this, std::cref(*aa_events), external_t);
        cv::Mat TS_img;
        renderTimeSurface(external_t, distance, TS_img);

        //distortion correction
        // Not in place: cv::remap cannot work in place and silently allocates a
        // temporary for the whole frame on every call.
        cv::remap(TS_img, ts_rect_, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);
        cv::swap(TS_img, ts_rect_);

        // generate OS-TS
        cv::Mat TS_img_blur;
        cv::Mat OS_TS = TS_img.clone(); 
        cv::blur(TS_img, TS_img_blur, cv::Size(blur_size_, blur_size_));
        cv::Mat mask = (TS_img == 0);
        TS_img_blur.copyTo(OS_TS, mask);
        cv::medianBlur(TS_img, TS_img, 2 * median_blur_kernel_size_ + 1);

        // generate and publish gradient map in parallel
        if (thread_sobel.joinable())
          thread_sobel.join();
        // 255 - OS_TS, in one pass and without the two temporaries.
        cv::bitwise_not(OS_TS, negative_TS_img);

        cv_bridge::CvImage cv_TS_image, cv_negative_TS_image;

        cv_TS_image.encoding = "mono8";
        cv_negative_TS_image.encoding = "mono8";
        cv_dx_image.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
        cv_dy_image.encoding = sensor_msgs::image_encodings::TYPE_16SC1;

        cv_TS_image.header.stamp = ros::Time(external_t);
        cv_negative_TS_image.header.stamp = ros::Time(external_t);
        cv_dx_image.header.stamp = ros::Time(external_t);
        cv_dy_image.header.stamp = ros::Time(external_t);

        cv_TS_image.image = TS_img.clone();
        cv_negative_TS_image.image = negative_TS_img.clone();

        thread_sobel = std::thread(&ImageRepresentation::sobel, this, external_t);

        cv_TS_image.header.stamp = external_sync_time;
        cv_negative_TS_image.header.stamp = external_sync_time;

        image_representation_pub_TS_.publish(cv_TS_image.toImageMsg());
        image_representation_pub_negative_TS_.publish(cv_negative_TS_image.toImageMsg());
        thread0.join();
      }
      else // generate TS, just for right camera
      {
        cv::Mat TS_img;
        renderTimeSurface(external_t, distance, TS_img);

        cv::remap(TS_img, ts_rect_, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);
        cv::swap(TS_img, ts_rect_);

        cv::medianBlur(TS_img, TS_img, 2 * median_blur_kernel_size_ + 1);

        cv_bridge::CvImage cv_TS_image;
        cv_TS_image.encoding = "mono8";
        cv_TS_image.header.stamp = ros::Time(external_t);
        cv_TS_image.image = TS_img.clone();
        image_representation_pub_TS_.publish(cv_TS_image.toImageMsg());
      }
    }
  }

  void ImageRepresentation::clearEvents(int distance, std::vector<dvs_msgs::Event>::iterator ptr_e)
  {
    if (vEvents_.size() > distance + 2)
      vEvents_.erase(vEvents_.begin(), ptr_e);
    else
      vEvents_.clear();
  }

  void ImageRepresentation::eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
  {
    TicToc t;
    std::lock_guard<std::mutex> lock(data_mutex_);
    double t1 = t.toc();
    if (!bSensorInitialized_)
      init(msg->width, msg->height);
    for (const dvs_msgs::Event &e : msg->events)
    {
      if (e.x >= sensor_size_.width || e.y >= sensor_size_.height)
        continue;
      // Time went backwards (a bag replayed in a loop, a clock step): drop what
      // is buffered rather than insertion-sort every new event past all of it,
      // and let the generation thread restart its per-pixel times.
      const double te = e.ts.toSec();
      if (last_event_t_ > 0.0 && te < last_event_t_ - kBackwardJumpS)
      {
        ROS_WARN("image_representation: event time jumped back %.3f s; restarting", last_event_t_ - te);
        vEvents_.clear();
        bTimeJumped_ = true;
      }
      last_event_t_ = te;
      vEvents_.push_back(e);

      int i = vEvents_.size() - 2;
      while (i >= 0 && vEvents_[i].ts > e.ts)
      {
        vEvents_[i + 1] = vEvents_[i];
        i--;
      }
      vEvents_[i + 1] = e;
    }
    clearEventQueue();
    bcreat_ = true;
  }

  void ImageRepresentation::clearEventQueue()
  {
    static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 5000000;
    if (vEvents_.size() > MAX_EVENT_QUEUE_LENGTH)
    {
      size_t remove_events = vEvents_.size() - MAX_EVENT_QUEUE_LENGTH;
      vEvents_.erase(vEvents_.begin(), vEvents_.begin() + remove_events);
    }
  }

  void ImageRepresentation::sobel(double external_t)
  {
    cv::Sobel(negative_TS_img, cv_dx_image.image, CV_16SC1, 1, 0);
    cv::Sobel(negative_TS_img, cv_dy_image.image, CV_16SC1, 0, 1);
    cv_dx_image.header.stamp = ros::Time(external_t);
    cv_dy_image.header.stamp = ros::Time(external_t);
    dx_image_pub_.publish(cv_dx_image.toImageMsg());
    dy_image_pub_.publish(cv_dy_image.toImageMsg());
  }

  bool ImageRepresentation::loadCalibInfo(const std::string &cameraSystemDir, bool &is_left)
  {
    bCamInfoAvailable_ = false;
    std::string cam_calib_dir;
    if (is_left)
      cam_calib_dir = cameraSystemDir + "/left.yaml";
    else
      cam_calib_dir = cameraSystemDir + "/right.yaml";
    if (!fileExists(cam_calib_dir))
      return bCamInfoAvailable_;
    YAML::Node CamCalibInfo = YAML::LoadFile(cam_calib_dir);

    // load calib (left)
    size_t width = CamCalibInfo["image_width"].as<int>();
    size_t height = CamCalibInfo["image_height"].as<int>();
    std::string cameraNameLeft = CamCalibInfo["camera_name"].as<std::string>();
    std::string distortion_model = CamCalibInfo["distortion_model"].as<std::string>();
    std::vector<double> vD, vK, vRectMat, vP;
    std::vector<double> vT_right_left, vT_b_c;

    vD = CamCalibInfo["distortion_coefficients"]["data"].as<std::vector<double>>();
    vK = CamCalibInfo["camera_matrix"]["data"].as<std::vector<double>>();
    vRectMat = CamCalibInfo["rectification_matrix"]["data"].as<std::vector<double>>();
    vP = CamCalibInfo["projection_matrix"]["data"].as<std::vector<double>>();

    vT_right_left = CamCalibInfo["T_right_left"]["data"].as<std::vector<double>>();
    vT_b_c = CamCalibInfo["T_b_c"]["data"].as<std::vector<double>>();

    cv::Size sensor_size(width, height);
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        camera_matrix_.at<double>(cv::Point(i, j)) = vK[i + j * 3];

    distortion_model_ = distortion_model;
    dist_coeffs_ = cv::Mat(vD.size(), 1, CV_64F);
    for (int i = 0; i < vD.size(); i++)
      dist_coeffs_.at<double>(i) = vD[i];

    if (bUseStereoCam_)
    {
      rectification_matrix_ = cv::Mat(3, 3, CV_64F);
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          rectification_matrix_.at<double>(cv::Point(i, j)) = vRectMat[i + j * 3];

      projection_matrix_ = cv::Mat(3, 4, CV_64F);
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
          projection_matrix_.at<double>(cv::Point(i, j)) = vP[i + j * 4];

      if (distortion_model_ == "equidistant")
      {
        cv::fisheye::initUndistortRectifyMap(camera_matrix_, dist_coeffs_,
                                             rectification_matrix_, projection_matrix_,
                                             sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
        // Fixed point maps: cv::remap is about twice as fast with CV_16SC2 as with CV_32FC1.
        { cv::Mat m1, m2; cv::convertMaps(undistort_map1_, undistort_map2_, m1, m2, CV_16SC2);
          undistort_map1_ = m1; undistort_map2_ = m2; }
        bCamInfoAvailable_ = true;
        ROS_INFO("Camera information is loaded (Distortion model %s).", distortion_model_.c_str());
      }
      else if (distortion_model_ == "plumb_bob")
      {
        cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_,
                                    rectification_matrix_, projection_matrix_,
                                    sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
        // Fixed point maps: cv::remap is about twice as fast with CV_16SC2 as with CV_32FC1.
        { cv::Mat m1, m2; cv::convertMaps(undistort_map1_, undistort_map2_, m1, m2, CV_16SC2);
          undistort_map1_ = m1; undistort_map2_ = m2; }
        bCamInfoAvailable_ = true;
        ROS_INFO("Camera information is loaded (Distortion model %s).", distortion_model_.c_str());
      }
      else
      {
        ROS_ERROR_ONCE("Distortion model %s is not supported.", distortion_model_.c_str());

        return bCamInfoAvailable_;
      }

      /* pre-compute the undistorted-rectified look-up table */
      precomputed_rectified_points_ = Eigen::Matrix2Xd(2, sensor_size.height * sensor_size.width);
      // raw coordinates
      cv::Mat_<cv::Point2f> RawCoordinates(1, sensor_size.height * sensor_size.width);
      for (int y = 0; y < sensor_size.height; y++)
      {
        for (int x = 0; x < sensor_size.width; x++)
        {
          int index = y * sensor_size.width + x;
          RawCoordinates(index) = cv::Point2f((float)x, (float)y);
        }
      }
      // undistorted-rectified coordinates
      cv::Mat_<cv::Point2f> RectCoordinates(1, sensor_size.height * sensor_size.width);
      if (distortion_model_ == "plumb_bob")
      {
        cv::undistortPoints(RawCoordinates, RectCoordinates, camera_matrix_, dist_coeffs_,
                            rectification_matrix_, projection_matrix_);
        ROS_INFO("Undistorted-Rectified Look-Up Table with Distortion model: %s", distortion_model_.c_str());
      }
      else if (distortion_model_ == "equidistant")
      {
        cv::fisheye::undistortPoints(
            RawCoordinates, RectCoordinates, camera_matrix_, dist_coeffs_,
            rectification_matrix_, projection_matrix_);
        ROS_INFO("Undistorted-Rectified Look-Up Table with Distortion model: %s", distortion_model_.c_str());
      }
      else
      {
        ROS_INFO("Unknown distortion model is provided.");
        return bCamInfoAvailable_;
      }
      // load look-up table
      for (size_t i = 0; i < sensor_size.height * sensor_size.width; i++)
      {
        precomputed_rectified_points_.col(i) = Eigen::Matrix<double, 2, 1>(
            RectCoordinates(i).x, RectCoordinates(i).y);
      }
      ROS_INFO("Undistorted-Rectified Look-Up Table has been computed.");
    }
    else
    {
      // TODO: calculate undistortion map
      bCamInfoAvailable_ = true;
    }
    return bCamInfoAvailable_;
  }

  bool ImageRepresentation::fileExists(const std::string &filename)
  {
    std::ifstream file(filename);
    return file.good();
  }

} // namespace image_representation
