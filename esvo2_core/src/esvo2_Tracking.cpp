#include <esvo2_core/esvo2_Tracking.h>
#include <cstring>
#include <iomanip>
#include <esvo2_core/tools/TicToc.h>
#include <esvo2_core/tools/params_helper.h>
#include <minkindr_conversions/kindr_tf.h>
#include <tf/transform_broadcaster.h>
#include <sys/stat.h>

//#define ESVO2_CORE_TRACKING_DEBUG
//#define ESVO2_CORE_TRACKING_DEBUG
namespace esvo2_core
{
esvo2_Tracking::esvo2_Tracking(
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nh_private):
  nh_(nh),
  pnh_(nh_private),
  it_(nh),
  TS_left_sub_(nh_, "time_surface_left", 10),
  TS_right_sub_(nh_, "time_surface_right", 10),
  TS_negative_sub_(nh_, "time_surface_negative", 10),
  TS_dx_sub_(nh_, "time_surface_dx", 10),
  TS_dy_sub_(nh_, "time_surface_dy", 10),
  TS_sync_(ApproximateSyncPolicy(10), TS_left_sub_, TS_right_sub_),
  TS_negaTS_sync_(ApproximateSyncPolicy_negaTS(10), TS_left_sub_, TS_negative_sub_, TS_dx_sub_, TS_dy_sub_),
  calibInfoDir_(tools::param(pnh_, "calibInfoDir", std::string(""))),
  camSysPtr_(new CameraSystem(calibInfoDir_, false)),
  rpConfigPtr_(new RegProblemConfig(
    tools::param(pnh_, "patch_size_X", 25),
    tools::param(pnh_, "patch_size_Y", 25),
    tools::param(pnh_, "kernelSize", 15),
    tools::param(pnh_, "LSnorm", std::string("l2")),
    tools::param(pnh_, "huber_threshold", 10.0),
    tools::param(pnh_, "invDepth_min_range", 0.0),
    tools::param(pnh_, "invDepth_max_range", 0.0),
    tools::param(pnh_, "MIN_NUM_EVENTS", 1000),
    tools::param(pnh_, "MAX_REGISTRATION_POINTS", 500),
    tools::param(pnh_, "BATCH_SIZE", 200),
    tools::param(pnh_, "MAX_ITERATION", 10))),
  rpType_((RegProblemType)((size_t)tools::param(pnh_, "RegProblemType", 0))),
  rpSolver_(camSysPtr_, rpConfigPtr_, rpType_, NUM_THREAD_TRACKING),
  ESVO2_System_Status_("INITIALIZATION"),
  imu_data_(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), g_optimal),
  ets_(IDLE)
{
  // offline data
  dvs_frame_id_        = tools::param(pnh_, "dvs_frame_id", std::string("dvs"));
  world_frame_id_      = tools::param(pnh_, "world_frame_id", std::string("world"));

  /**** online parameters ***/
  tracking_rate_hz_    = tools::param(pnh_, "tracking_rate_hz", 100);
  TS_HISTORY_LENGTH_  = tools::param(pnh_, "TS_HISTORY_LENGTH", 100);
  REF_HISTORY_LENGTH_  = tools::param(pnh_, "REF_HISTORY_LENGTH", 5);
  bSaveTrajectory_     = tools::param(pnh_, "SAVE_TRAJECTORY", false);
  bVisualizeTrajectory_ = tools::param(pnh_, "VISUALIZE_TRAJECTORY", true);
  bUseImu_ = tools::param(pnh_, "USE_IMU", true);
  bImuRotationPrediction_ = tools::param(pnh_, "IMU_ROTATION_PREDICTION", false);
  // With USE_IMU, upstream adds the moving average of the last five registered
  // displacements to every frame's translation prior, unbounded. One bad
  // registration step therefore becomes velocity, the next frame starts far
  // off, lands wrong again, and the error compounds: tracking reached 1e5-1e11 m
  // on MVSEC indoor_flying1 and VECtor desk-normal, and the mapping node crashed
  // in every such run seen. Without the term both track normally (ATE 0.090 m
  // and 0.20-0.24 m). Set true to restore the upstream behaviour.
  bImuConstantVelocityPrior_ = tools::param(pnh_, "IMU_CONSTANT_VELOCITY_PRIOR", false);
  imuTimeOffset_ = tools::param(pnh_, "IMU_TIME_OFFSET", 0.0);
  // tools::param cannot print a vector, so GYRO_BIAS is read directly.
  std::vector<double> vGyroBias;
  if (pnh_.getParam("GYRO_BIAS", vGyroBias) && vGyroBias.size() == 3)
  {
    gyroBias_ = Eigen::Vector3d(vGyroBias[0], vGyroBias[1], vGyroBias[2]);
    bGyroBiasKnown_ = true;
  }
  else
  {
    if (pnh_.hasParam("GYRO_BIAS"))
      LOG(WARNING) << "GYRO_BIAS must be a list of 3 numbers (rad/s, IMU frame); estimating the bias instead";
    gyroBiasWindow_ = tools::param(pnh_, "GYRO_BIAS_WINDOW", 2.0);
    gyroBiasMaxStd_ = tools::param(pnh_, "GYRO_STILL_MAX_STD", 0.0035);
    if (gyroBiasWindow_ <= 0.0 || gyroBiasMaxStd_ <= 0.0)
    {
      LOG(WARNING) << "GYRO_BIAS_WINDOW and GYRO_STILL_MAX_STD must be > 0 (got " << gyroBiasWindow_ << ", "
                   << gyroBiasMaxStd_ << "); using defaults (2.0, 0.0035)";
      gyroBiasWindow_ = 2.0;
      gyroBiasMaxStd_ = 0.0035;
    }
    gyroBiasEstimator_.reset(new tools::GyroBiasEstimator(gyroBiasWindow_, gyroBiasMaxStd_));
    bGyroBiasRefresh_ = tools::param(pnh_, "GYRO_BIAS_REFRESH", false);
  }
  bImuRotationLock_ = tools::param(pnh_, "IMU_ROTATION_LOCK", false);
  if (bImuRotationLock_ && (!bImuRotationPrediction_ || bUseImu_))
  {
    LOG(WARNING) << "IMU_ROTATION_LOCK needs IMU_ROTATION_PREDICTION: True and USE_IMU: False; lock disabled";
    bImuRotationLock_ = false;
  }
  if (bImuRotationLock_ && rpType_ != REG_ANALYTICAL)
  {
    LOG(WARNING) << "IMU_ROTATION_LOCK only works with RegProblemType: 1 (analytical); lock disabled";
    bImuRotationLock_ = false;
  }
  if (bImuRotationLock_)
    LOG(INFO) << "IMU rotation lock enabled: rotation from the gyro, translation-only registration";
  lastPredLog_ = ros::WallTime::now();
  // Hold at rest. A still event camera sees almost nothing, so the map empties and upstream
  // resets the whole system at every stop. Events decide first: a frame the events can
  // register is tracked whatever the IMU says. Only a frame they cannot register (map too
  // sparse, or J^T J below STILL_MIN_INFO) asks the IMU, and only an IMU that is fresh and
  // quiet (tools::StillnessDetector) turns the reset into a hold. A missing, stale or frozen
  // IMU is never "still", so losing the IMU falls back to upstream behaviour.
  bStillnessHold_ = tools::param(pnh_, "STILLNESS_HOLD", false);
  stillMinInfo_ = tools::param(pnh_, "STILL_MIN_INFO", 0.0);
  stillMinSupport_ = tools::param(pnh_, "STILL_MIN_SUPPORT", 0.0);
  holdMaxS_ = tools::param(pnh_, "HOLD_MAX_S", 120.0);
  mapGraceS_ = tools::param(pnh_, "HOLD_MAP_GRACE_S", 1.0);
  coastMaxS_ = tools::param(pnh_, "HOLD_COAST_S", 0.5);
  stillDet_ = tools::StillnessDetector(tools::param(pnh_, "STILL_WINDOW", 0.5), gyroBiasMaxStd_,
                                       tools::param(pnh_, "ACC_STILL_MAX_STD", 0.03));
  const std::string motionLogPath = tools::param(pnh_, "MOTION_LOG", std::string());
  if (!motionLogPath.empty())
  {
    motionLog_.open(motionLogPath);
    motionLog_ << "t,action,ref_pts,imu,gyro_std,acc_std,imu_age,fresh,support,info,locked,x,y,z\n";
  }
  if (bStillnessHold_)
    LOG(INFO) << "Stillness hold enabled: STILL_MIN_INFO " << stillMinInfo_ << ", STILL_MIN_SUPPORT "
              << stillMinSupport_ << ", HOLD_MAX_S " << holdMaxS_
              << ", GYRO_BIAS_REFRESH " << bGyroBiasRefresh_;
  resultPath_             = tools::param(pnh_, "PATH_TO_SAVE_TRAJECTORY", std::string());
  nh_.setParam("/ESVO2_SYSTEM_STATUS", ESVO2_System_Status_);

  // get extrinsic parameters
  R_b_c_ = camSysPtr_->cam_left_ptr_->T_b_c_.block<3, 3>(0, 0);
  
  imu_data_.initialization(ba_, bg_);
  initVsFlag = false;

  TS_negaTS_sync_.registerCallback(boost::bind(&esvo2_Tracking::timeSurface_NegaTS_Callback, this, _1, _2, _3, _4));

  tf_ = std::make_shared<tf::Transformer>(true, ros::Duration(100.0));
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/esvo2_tracking/pose_pub", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/esvo2_tracking/trajectory", 1);
  map_sub_ = nh_.subscribe("pointcloud", 0, &esvo2_Tracking::refMapCallback, this);// local map in the ref view.
  stampedPose_sub_ = nh_.subscribe("stamped_pose", 0, &esvo2_Tracking::stampedPoseCallback, this);// for accessing the pose of the ref view.
  imu_sub_ = nh_.subscribe("/imu/data", 0, &esvo2_Tracking::refImuCallback, this);// local map in the ref view.
  if (bImuRotationPrediction_ || bStillnessHold_ || motionLog_.is_open())
    imu_prediction_sub_ = nh_.subscribe("imu_prediction", 2000, &esvo2_Tracking::imuPredictionCallback, this);
  if (bImuRotationPrediction_)
  {
    if (bUseImu_)
      LOG(WARNING) << "IMU_ROTATION_PREDICTION is ignored while USE_IMU is true";
    LOG(INFO) << "IMU rotation prediction enabled, IMU_TIME_OFFSET = " << imuTimeOffset_ << " s";
    if (bGyroBiasKnown_)
      LOG(INFO) << "Gyro bias from GYRO_BIAS: " << gyroBias_.transpose() << " rad/s";
    else
      LOG(INFO) << "Gyro bias: estimating from the first " << gyroBiasEstimator_->window()
                << " s still window (hold the rig still); prediction is uncorrected until then";
  }
  V_ba_bg_sub_ = nh_.subscribe("/esvo2_mapping/V_ba_bg", 0, &esvo2_Tracking::VBaBgCallback, this);
  /*** For Visualization and Test ***/
  reprojMap_pub_left_ = it_.advertise("Reproj_Map_Left", 1);
  rpSolver_.setRegPublisher(&reprojMap_pub_left_);

  // rename the old trajectory file
  renameOldTraj();

  /*** Tracker ***/
  T_world_cur_ = Eigen::Matrix<double,4,4>::Identity();
  T_world_hold_ = T_world_cur_;
  t_world_cur_ = last_t_world_cur_ = last_t_ = Eigen::Vector3d::Zero();
  std::thread TrackingThread(&esvo2_Tracking::TrackingLoop, this);
  TrackingThread.detach();
}

esvo2_Tracking::~esvo2_Tracking()
{
  if(!resultPath_.empty())
  {
    LOG(INFO) << "pose size: " << lPose_.size();
    string path = std::string(resultPath_ + "result.txt");
    saveTrajectory(path);
  }
  pose_pub_.shutdown();
}

void esvo2_Tracking::TrackingLoop()
{
  ros::Rate r(tracking_rate_hz_);
  while(ros::ok())
  {
    // Keep Idling
    if(refPCMap_.size() < 1 || TS_history_.size() < 1)
    {
      r.sleep();
      continue;
    }

    // Reset
    nh_.getParam("/ESVO2_SYSTEM_STATUS", ESVO2_System_Status_);
    if(ESVO2_System_Status_ == "INITIALIZATION" && ets_ == WORKING)// This is true when the system is reset from dynamic reconfigure
    {
      reset();
      r.sleep();
      continue;
    }
    if(ESVO2_System_Status_ == "TERMINATE")
    {
      LOG(INFO) << "The tracking node is terminated manually...";
      break;
    }
    TicToc tt;
    double curData_time;
    // Data Transfer (If mapping node had published refPC.)
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if(lastMapSeen_ < refPCMap_.rbegin()->first)// new reference map arrived
        refDataTransferring();
      if(cur_.t_.toSec() < TS_history_.rbegin()->first.toSec())// new observation arrived
      {
        if(ref_.t_.toSec() >= TS_history_.rbegin()->first.toSec())
        {
          LOG(INFO) << "The time_surface observation should be obtained after the reference frame";
          exit(-1);
        }
        if(!curDataTransferring())
        {
          continue;
        }
      }
      else
      {
        continue;
      }
    }
    curData_time = tt.toc();

    // create new regProblem
    double t_resetRegProblem, t_solve, t_pub_result;
    const double tCur = cur_.t_.toSec();
    const bool wasWorking = ets_ == WORKING;
    const bool canRegister = ref_.vPointXYZPtr_.size() >= rpConfigPtr_->BATCH_SIZE_;
    tools::Motion imuMotion = tools::Motion::UNKNOWN;
    double gyroStd = -1.0, accStd = -1.0, imuAge = 1e9;
    if(bStillnessHold_ || motionLog_.is_open())
    {
      std::lock_guard<std::mutex> lock(gyro_mutex_);
      const double tImu = tCur - imuTimeOffset_; // camera time = IMU time + IMU_TIME_OFFSET
      imuMotion = stillDet_.classify(tImu);
      gyroStd = stillDet_.gyroStd();
      accStd = stillDet_.accStd();
      imuAge = stillDet_.age(tImu);
    }
    if(bStillnessHold_)
    {
      if(imuAge > 0.1 && !bImuLostWarned_)
      {
        LOG(WARNING) << "Stillness hold: " << (bImuSeen_ ? "IMU silent" : "no IMU yet")
                     << " on imu_prediction; holding disabled until it streams";
        bImuLostWarned_ = true;
      }
      else if(imuAge <= 0.1 && bImuLostWarned_)
      {
        LOG(INFO) << "Stillness hold: IMU streaming again";
        bImuLostWarned_ = false;
      }
    }
    // Events first. A registration counts only when the events support it: the smallest
    // eigenvalue of J^T J/N reaches STILL_MIN_INFO and, if the IMU says still, the time
    // surface shows structure (STILL_MIN_SUPPORT); a still camera matches the map against an
    // empty time surface and drifts. A frame the events cannot support is never published;
    // the last pose is republished instead:
    //  - IMU still: hold.
    //  - IMU moving, time surface quiet: the events see no motion relative to the scene (the
    //    cart being handled, or starting to roll before the scene shows anything), and they
    //    outrank the IMU: coast on the last pose.
    //  - IMU moving, time surface active but the registration unusable: coast for at most
    //    HOLD_COAST_S, then upstream behaviour.
    // Registration is absolute against the map, so a skipped frame loses nothing. Holds and
    // coasts end after HOLD_MAX_S. Without a usable IMU, or before tracking has started:
    // upstream behaviour, publish the registration if there is one, else reset.
    double info = -1.0, fresh = -1.0, support = -1.0;
    if((bStillnessHold_ && wasWorking) || motionLog_.is_open())
      if(cur_.pTsObs_ && cur_.pTsObs_->cvImagePtr_left_)
        eventActivity(cur_.pTsObs_->cvImagePtr_left_->image, fresh, support);
    const bool eventsQuiet = support >= 0.0 && support < stillMinSupport_;
    const bool holdTooLong = (bHolding_ && tCur - holdStartT_ > holdMaxS_) ||
                             (coastStartT_ >= 0.0 && tCur - coastStartT_ > holdMaxS_);
    if(holdTooLong && !bHoldTooLongWarned_)
    {
      LOG(WARNING) << "Stillness hold: held for more than HOLD_MAX_S = " << holdMaxS_ << " s; no longer holding";
      bHoldTooLongWarned_ = true;
    }
    const bool canHold = bStillnessHold_ && wasWorking && imuMotion == tools::Motion::STILL && !holdTooLong;
    const bool canCoast = bStillnessHold_ && wasWorking && imuMotion == tools::Motion::MOVING && !holdTooLong &&
                          (eventsQuiet || coastStartT_ < 0.0 || tCur - coastStartT_ < coastMaxS_);

    bool registered = false;
    if(canRegister && rpSolver_.resetRegProblem(&ref_, &cur_))
    {
      if(ets_ == IDLE)
        ets_ = WORKING;
      if(ESVO2_System_Status_ != "WORKING")
      {
        nh_.setParam("/ESVO2_SYSTEM_STATUS", "WORKING");
        LOG(INFO) << "ESVO2_SYSTEM_STATUS: WORKING";
      }

      rpSolver_.setFixRotation(bLockThisFrame_);
      // TicToc t_coarse;
      if(rpType_ == REG_NUMERICAL)
        rpSolver_.solve_numerical();
      if(rpType_ == REG_ANALYTICAL)
        rpSolver_.solve_analytical();
      if(bStillnessHold_ || motionLog_.is_open())
        info = rpSolver_.informationMinEig(bLockThisFrame_);
      registered = true;
    }
    // Leaving a hold or a coast takes positive evidence from the events: a structured time
    // surface, not only J^T J crossing its threshold, which isolated frames of a still camera
    // do by chance (jumps of up to 0.5 m on hallway4_lateral). While tracking, a quiet surface
    // alone does not stop it: 7-10% of moving frames have one, and they register well.
    const bool resuming = bHolding_ || coastStartT_ >= 0.0;
    const bool observable = registered &&
        (!bStillnessHold_ || (info >= stillMinInfo_ &&
                              ((imuMotion != tools::Motion::STILL && !resuming) || support >= stillMinSupport_)));

    const char *action;
    if(observable || (registered && !canHold && !canCoast))
    {
      action = "TRACK";
      T_world_cur_ = cur_.tr_.getTransformationMatrix();
      t_world_cur_ = T_world_cur_.block(0, 3, 3, 1);
      T_world_hold_ = T_world_cur_;
      if(bHolding_)
      {
        LOG(INFO) << "Stillness hold: released after " << tCur - holdStartT_ << " s (" << nHold_ << " frames held, "
                  << nCoast_ << " coasted)";
        bHolding_ = false;
        holdEndT_ = tCur;
      }
      coastStartT_ = -1.0;
      publishPose(cur_.t_, cur_.tr_);
      if(bVisualizeTrajectory_)
        publishPath(cur_.t_, cur_.tr_);

      // save result and gt if available.
      if(bSaveTrajectory_)
      {
        // save results to listPose and listPoseGt
        lTimestamp_.push_back(std::to_string(cur_.t_.toSec()));
        lPose_.push_back(cur_.tr_.getTransformationMatrix());
      }
    }
    else if(canHold || canCoast)
    {
      // Republish the last tracked pose at this frame's time: mapping needs a pose for every
      // time surface (it resets itself after 0.5 s without one).
      if(canHold)
      {
        action = "HOLD";
        coastStartT_ = -1.0;
        if(!bHolding_)
        {
          bHolding_ = true;
          bHoldTooLongWarned_ = false;
          holdStartT_ = tCur;
          nHold_ = nCoast_ = 0;
          LOG(INFO) << "Stillness hold: started (" << (canRegister ? "events uninformative" : "map too sparse")
                    << ", gyro std " << gyroStd << ", acc std " << accStd << ")";
        }
        nHold_++;
      }
      else
      {
        action = "COAST";
        if(coastStartT_ < 0.0)
        {
          coastStartT_ = tCur;
          if(!bHolding_)
            bHoldTooLongWarned_ = false;
        }
        nCoast_++;
      }
      T_world_cur_ = T_world_hold_;
      t_world_cur_ = T_world_cur_.block(0, 3, 3, 1);
      cur_.tr_ = Transformation(T_world_cur_);
      publishPose(cur_.t_, cur_.tr_);
      if(bVisualizeTrajectory_)
        publishPath(cur_.t_, cur_.tr_);
      if(bSaveTrajectory_)
      {
        lTimestamp_.push_back(std::to_string(cur_.t_.toSec()));
        lPose_.push_back(cur_.tr_.getTransformationMatrix());
      }
    }
    else
    {
      action = "RESET";
      if(!canRegister)
        rpSolver_.resetRegProblem(&ref_, &cur_); // fails; logs the upstream re-initialisation message
      nh_.setParam("/ESVO2_SYSTEM_STATUS", "INITIALIZATION");
      ets_ = IDLE;
      coastStartT_ = -1.0;
      if(bHolding_)
      {
        bHolding_ = false;
        holdEndT_ = tCur;
      }
    }

    bHoldFrame_ = std::strcmp(action, "HOLD") == 0;

    if(motionLog_.is_open())
    {
      const Eigen::Vector3d p = cur_.tr_.getPosition();
      motionLog_ << std::fixed << std::setprecision(4) << tCur << ',' << action << ','
                 << ref_.vPointXYZPtr_.size() << ',' << tools::motionName(imuMotion) << ','
                 << std::setprecision(6) << gyroStd << ',' << accStd << ',' << std::min(imuAge, 99.0) << ','
                 << fresh << ',' << support << ',' << std::scientific << info << std::fixed << ','
                 << bLockThisFrame_ << ',' << p.x() << ',' << p.y() << ',' << p.z() << std::endl;
    }
    std::ofstream f;

#ifdef  ESVO2_CORE_TRACKING_LOG
    double t_overall_count = 0;
    t_overall_count = t_resetRegProblem + t_solve + t_pub_result;
    LOG(INFO) << "\n";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "--------------------Tracking Computation Cost---------------";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "ResetRegProblem: " << t_resetRegProblem << " ms, (" << t_resetRegProblem / t_overall_count * 100 << "%).";
    LOG(INFO) << "Registration: " << t_solve << " ms, (" << t_solve / t_overall_count * 100 << "%).";
    LOG(INFO) << "pub result: " << t_pub_result << " ms, (" << t_pub_result / t_overall_count * 100 << "%).";
    LOG(INFO) << "Total Computation (" << rpSolver_.lmStatics_.nPoints_ << "): " << t_overall_count << " ms.";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "------------------------------------------------------------";
#endif
    r.sleep();
  }// while
}

bool
esvo2_Tracking::refDataTransferring()
{
  lastMapSeen_ = refPCMap_.rbegin()->first;
  // Keep a usable map rather than swap in a near-empty one: while holding (the rig has not
  // moved, so the old map is still valid), and for HOLD_MAP_GRACE_S after the first sparse map
  // or the end of a hold (the rig is slowing down, or mapping is refilling). A rig that keeps
  // moving on sparse maps past the grace adopts them, and resets as upstream does.
  const size_t batch = rpConfigPtr_->BATCH_SIZE_;
  if(refPCMap_.rbegin()->second->size() >= batch)
    sparseSinceT_ = -1.0;
  else if(bStillnessHold_ && ets_ == WORKING && refCloud_ && refCloud_->size() >= batch)
  {
    if(sparseSinceT_ < 0.0)
      sparseSinceT_ = cur_.t_.toSec();
    if(bHolding_ || cur_.t_.toSec() - std::max(sparseSinceT_, holdEndT_) < mapGraceS_)
      return true;
  }

  // load reference info
  ref_.t_ = ros::Time(refPCMap_.rbegin()->first.toSec());
  ros::Time t = ros::Time(refPCMap_.rbegin()->first.toSec()-0.001);
  nh_.getParam("/ESVO2_SYSTEM_STATUS", ESVO2_System_Status_);

  if(ESVO2_System_Status_ == "INITIALIZATION" && ets_ == IDLE)
    ref_.tr_.setIdentity();
  if(ESVO2_System_Status_ == "WORKING" || (ESVO2_System_Status_ == "INITIALIZATION" && ets_ == WORKING))
  {
    if(!getPoseAt(t, ref_.tr_, dvs_frame_id_))
    {
      LOG(INFO) << "ESVO2_System_Status_: " << ESVO2_System_Status_ << ", ref_.t_: " << ref_.t_.toNSec();
      LOG(INFO) << "Logic error ! There must be a pose for the given timestamp, because mapping has been finished.";
      // exit(-1);
      return false;
    }
  }

  // get the point cloud
  refCloud_ = refPCMap_.rbegin()->second;
  size_t numPoint = refPCMap_.rbegin()->second->size();
  ref_.vPointXYZPtr_.clear();
  ref_.vPointXYZPtr_.reserve(numPoint);
  auto PointXYZ_begin_it = refPCMap_.rbegin()->second->begin();
  auto PointXYZ_end_it   = refPCMap_.rbegin()->second->end();
  while(PointXYZ_begin_it != PointXYZ_end_it)
  {
    ref_.vPointXYZPtr_.push_back(PointXYZ_begin_it.base());// Copy the pointer of the pointXYZ
    PointXYZ_begin_it++;
  }
  return true;
}


bool
esvo2_Tracking::curDataTransferring()
{
  // load current observation
  auto ev_last_it = EventBuffer_lower_bound(events_left_, cur_.t_);
  auto TS_it = TS_history_.rbegin();

  // TS_history may not be updated before the tracking loop excutes the data transfering
  if(cur_.t_ == TS_it->first)
    return false;
  const double t_prev_frame = cur_.t_.toSec();
  bLockThisFrame_ = false;
  cur_.t_ = TS_it->first;
  cur_.pTsObs_ = &TS_it->second;

  nh_.getParam("/ESVO2_SYSTEM_STATUS", ESVO2_System_Status_);
  if(ESVO2_System_Status_ == "INITIALIZATION" && ets_ == IDLE)
  {
    cur_.tr_ = ref_.tr_;
  }
  if(ESVO2_System_Status_ == "WORKING" || (ESVO2_System_Status_ == "INITIALIZATION" && ets_ == WORKING))
  {
    curImuTransferring();
    // if use imu, the pose of the current frame is updated in the imu preintegration.
    if(bUseImu_)
    {
      Eigen::Matrix3d R_w_c = T_world_cur_.block(0, 0, 3, 3);

      Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond q = q1.slerp(1, Imu_q);

      if(t_world_cur_ != Eigen::Vector3d::Zero() && last_t_world_cur_ != Eigen::Vector3d::Zero())
      {
        last_t_ = t_world_cur_ - last_t_world_cur_;
        last_t_world_cur_ = T_world_cur_.block(0, 3, 3, 1);
        qprevious_ts_.push_back(last_t_);
        int average_window_size = 5;
        if(qprevious_ts_.size() > average_window_size)
        {
          qprevious_ts_.erase(qprevious_ts_.begin(), qprevious_ts_.begin() + qprevious_ts_.size() - average_window_size);
        }
        if(qprevious_ts_.size() > 3)
        {
          last_t_.setZero();
          for(Eigen::Vector3d t: qprevious_ts_)
          {
            last_t_ += t;
          }
          last_t_ = last_t_ / qprevious_ts_.size();
        }
        
        // If the predicted position change is significantly different from the previous displacement due to potentially unstable velocity estimates, 
        // use the previous displacement as the initial value for the next optimization.
        if (!bImuConstantVelocityPrior_)
          T_world_cur_.block(0, 3, 3, 1) += R_b_c_.transpose() * Imu_t;
        else if(initVsFlag && (imu_data_.t_v_last_mapping.second * imu_data_.sum_dt - last_t_).norm()/last_t_.norm() < 0.1)
          T_world_cur_.block(0, 3, 3, 1) += R_b_c_.transpose() * Imu_t + (imu_data_.t_v_last_mapping.second * imu_data_.sum_dt); 
        else
          T_world_cur_.block(0, 3, 3, 1) += R_b_c_.transpose() * Imu_t + last_t_;
      }
      else
      {
        last_t_world_cur_ = T_world_cur_.block(0, 3, 3, 1);
      }

      T_world_cur_.block(0, 0, 3, 3) =  R_w_c * R_b_c_* q.toRotationMatrix() * R_b_c_.inverse();
    } 
    if(bImuRotationPrediction_ && !bUseImu_ && ESVO2_System_Status_ == "WORKING") // no prediction during INITIALIZATION
    {
      bool biasCorrected = false;
      predictRotationWithGyro(t_prev_frame, cur_.t_.toSec(), biasCorrected, bLockThisFrame_);
    }
    Eigen::Matrix3d R_w_c = T_world_cur_.block(0, 0, 3, 3);
    T_world_cur_.block(0, 0, 3, 3) = fixRotationMatrix(R_w_c);
    cur_.tr_ = Transformation(T_world_cur_);
  }
  return true;
}

bool esvo2_Tracking::curImuTransferring()
{
  auto TS_it = TS_history_.rbegin();
  double cur_TS_time = TS_it->first.toSec();
  double last_TS_time = (++TS_it)->first.toSec();

  if(bUseImu_)
  {
    imu_mutex_.lock();
    if(initVsFlag && (refPCMap_.rbegin()->first.toSec() - imu_data_.t_v_last_mapping.first > 0.001))
        imu_data_.update_v(refPCMap_.rbegin()->first.toSec(), last_TS_time);
    imu_data_.getPose(last_TS_time, cur_TS_time, true, ref_.t_.toSec());
    if(t_world_cur_ != Eigen::Vector3d::Zero() && last_t_world_cur_ != Eigen::Vector3d::Zero())
    {
      Eigen::Matrix3d R_w_c = T_world_cur_.block(0, 0, 3, 3);
      imu_data_.t_v_last_mapping.second += R_b_c_ * R_w_c.transpose() * imu_data_.delta_v;
    }
    imu_mutex_.unlock();
  }
  
  Eigen::Vector3d delta_p = imu_data_.delta_p;
  Eigen::Quaterniond delta_q = imu_data_.delta_q;

  Imu_q = imu_data_.delta_q ;
  Imu_t = imu_data_.delta_p;
  return true;
}


void esvo2_Tracking::reset()
{
  // clear all maintained data
  ets_ = IDLE;
  bHolding_ = false;
  TS_id_ = 0;
  TS_history_.clear();
  refPCMap_.clear();
  events_left_.clear();
}


/********************** Callback functions *****************************/
void esvo2_Tracking::refImuCallback(const sensor_msgs::ImuPtr& msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  Eigen::Vector3d acc, gyr;
  if(imu_data_.dt_buf.size() == 0){
  
    acc[0] = msg->linear_acceleration.x;
    acc[1] = msg->linear_acceleration.y;
    acc[2] = msg->linear_acceleration.z;

    gyr[0] = msg->angular_velocity.x;
    gyr[1] = msg->angular_velocity.y;
    gyr[2] = msg->angular_velocity.z;
    if(imu_data_.last_time == 0)
    {
      imu_data_.begin_time = msg->header.stamp.toSec();
      imu_data_.push_back(0.001, acc, gyr);
      imu_data_.last_time = imu_data_.begin_time;
    }
    else
    {
      double dt = msg->header.stamp.toSec() - imu_data_.last_time;
      if(dt < 0)
        return;
      imu_data_.begin_time = msg->header.stamp.toSec();
      imu_data_.push_back(dt, acc, gyr);
    }
  }
  else{
    double time = msg->header.stamp.toSec();
    double dt = time - imu_data_.last_time;
    if(dt < 0)
      return;
    acc[0] = msg->linear_acceleration.x;
    acc[1] = msg->linear_acceleration.y;
    acc[2] = msg->linear_acceleration.z;

    gyr[0] = msg->angular_velocity.x;
    gyr[1] = msg->angular_velocity.y;
    gyr[2] = msg->angular_velocity.z;
    imu_data_.push_back(dt, acc, gyr);
    imu_data_.last_time = time;
  }
}

void esvo2_Tracking::VBaBgCallback(const events_repacking_tool::V_ba_bg& msg)
{
  Eigen::Vector3d g_temp, ba_temp, bg_temp, V_temp;
  double t_temp = msg.head[0];
  for(int i = 0; i < 3; i++)
  {
    g_temp(i) = msg.g[i];
    ba_temp(i) = msg.ba[i];
    bg_temp(i) = msg.bg[i];
    V_temp(i) = msg.Vs[i];
  }
  imu_mutex_.lock();
  imu_data_.G = g_temp;
  imu_data_.linearized_ba = ba_temp;
  imu_data_.linearized_bg = bg_temp;
  imu_data_.t_v_last_mapping = std::make_pair(t_temp, V_temp);
  imu_mutex_.unlock();
  initVsFlag = true;
}

void esvo2_Tracking::refMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*msg, pcl_pc);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr PC_ptr(new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::fromPCLPointCloud2(pcl_pc, *PC_ptr);
  refPCMap_.emplace(msg->header.stamp, PC_ptr);
  while(refPCMap_.size() > REF_HISTORY_LENGTH_)
  {
    auto it = refPCMap_.begin();
    refPCMap_.erase(it);
  }
}

void esvo2_Tracking::eventsCallback(
  const dvs_msgs::EventArray::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // add new ones and remove old ones
  for(const dvs_msgs::Event& e : msg->events)
  {
    events_left_.push_back(e);
    int i = events_left_.size() - 2;
    while(i >= 0 && events_left_[i].ts > e.ts) // we may have to sort the queue, just in case the raw event messages do not come in a chronological order.
    {
      events_left_[i+1] = events_left_[i];
      i--;
    }
    events_left_[i+1] = e;
  }
  clearEventQueue();
}

void esvo2_Tracking::clearEventQueue()
{
  static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 5000000;
  if (events_left_.size() > MAX_EVENT_QUEUE_LENGTH)
  {
    size_t remove_events = events_left_.size() - MAX_EVENT_QUEUE_LENGTH;
    events_left_.erase(events_left_.begin(), events_left_.begin() + remove_events);
  }
}

void
esvo2_Tracking::timeSurface_NegaTS_Callback(
  const sensor_msgs::ImageConstPtr &time_surface_left,
  const sensor_msgs::ImageConstPtr &time_surface_negative,
  const sensor_msgs::ImageConstPtr &time_surface_dx,
  const sensor_msgs::ImageConstPtr &time_surface_dy)
{
  
  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_negative, cv_ptr_dx, cv_ptr_dy;
  try
  {
    cv_ptr_left  = cv_bridge::toCvCopy(time_surface_left,  sensor_msgs::image_encodings::MONO8);
    cv_ptr_negative = cv_bridge::toCvCopy(time_surface_negative, sensor_msgs::image_encodings::MONO8);
    cv_ptr_dx = cv_bridge::toCvCopy(time_surface_dx, sensor_msgs::image_encodings::TYPE_16SC1);
    cv_ptr_dy = cv_bridge::toCvCopy(time_surface_dy, sensor_msgs::image_encodings::TYPE_16SC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  // push back the most current TS.
  ros::Time t_new_ts = time_surface_left->header.stamp;
  TS_history_.emplace(t_new_ts, TimeSurfaceObservation(cv_ptr_left, cv_ptr_negative, cv_ptr_dx, cv_ptr_dy, TS_id_, false));
  TS_id_++;

  // keep TS_history_'s size constant
  while(TS_history_.size() > TS_HISTORY_LENGTH_)
  {
    auto it = TS_history_.begin();
    TS_history_.erase(it);
  }
}

void esvo2_Tracking::stampedPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // add pose to tf
  tf::Transform tf(
    tf::Quaternion(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w),
    tf::Vector3(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z));
  tf::StampedTransform st(tf, msg->header.stamp, msg->header.frame_id, dvs_frame_id_.c_str());
  tf_->setTransform(st);
  // broadcast the tf such that the nav_path messages can find the valid fixed frame "map".
  static tf::TransformBroadcaster br;
  br.sendTransform(st);
}

bool
esvo2_Tracking::getPoseAt(
  const ros::Time &t, esvo2_core::Transformation &Tr, const std::string &source_frame)
{
  std::string* err_msg = new std::string();
  if(!tf_->canTransform(world_frame_id_, source_frame, t, err_msg))
  {
    LOG(WARNING) << t.toNSec() << " : " << *err_msg;
    delete err_msg;
    return false;
  }
  else
  {
    tf::StampedTransform st;
    tf_->lookupTransform(world_frame_id_, source_frame, t, st);
    tf::transformTFToKindr(st, &Tr);
    return true;
  }
}

/************ publish results *******************/
void esvo2_Tracking::publishPose(const ros::Time &t, Transformation &tr)
{
  geometry_msgs::PoseStampedPtr ps_ptr(new geometry_msgs::PoseStamped());

  ps_ptr->header.stamp = t;
  ps_ptr->header.frame_id = world_frame_id_;
  ps_ptr->pose.position.x = tr.getPosition()(0);
  ps_ptr->pose.position.y = tr.getPosition()(1);
  ps_ptr->pose.position.z = tr.getPosition()(2);
  ps_ptr->pose.orientation.x = tr.getRotation().x();
  ps_ptr->pose.orientation.y = tr.getRotation().y();
  ps_ptr->pose.orientation.z = tr.getRotation().z();
  ps_ptr->pose.orientation.w = tr.getRotation().w();
  pose_pub_.publish(ps_ptr);
  if(!resultPath_.empty())
  {
    std::ofstream f;
    f.open(resultPath_ + "stamped_traj_estimate_ours.txt", std::ofstream::app);
    f << std::fixed;
    Eigen::Matrix3d Rwc_result;
    Eigen::Vector3d twc_result;
    f.setf(std::ios::fixed, std::ios::floatfield);
    f.precision(9);
    f << t << " ";
    f.precision(5);
    f << ps_ptr->pose.position.x << " "
    << ps_ptr->pose.position.y << " "
    << ps_ptr->pose.position.z << " "
    << ps_ptr->pose.orientation.x << " "
    << ps_ptr->pose.orientation.y << " "
    << ps_ptr->pose.orientation.z << " "
    << ps_ptr->pose.orientation.w << endl;
    f.close();
  }
}

void esvo2_Tracking::publishPath(const ros::Time& t, Transformation& tr)
{
  geometry_msgs::PoseStampedPtr ps_ptr(new geometry_msgs::PoseStamped());
  
  ps_ptr->header.stamp = t;
  ps_ptr->header.frame_id = world_frame_id_;
  ps_ptr->pose.position.x = tr.getPosition()(0);
  ps_ptr->pose.position.y = tr.getPosition()(1);
  ps_ptr->pose.position.z = tr.getPosition()(2);
  ps_ptr->pose.orientation.x = tr.getRotation().x();
  ps_ptr->pose.orientation.y = tr.getRotation().y();
  ps_ptr->pose.orientation.z = tr.getRotation().z();
  ps_ptr->pose.orientation.w = tr.getRotation().w();
  path_.header.stamp = t;
  path_.header.frame_id = world_frame_id_;
  path_.poses.push_back(*ps_ptr);
  path_pub_.publish(path_);
}

void
esvo2_Tracking::saveTrajectory(std::string &resultDir)
{
  LOG(INFO) << "Saving trajectory to " << resultPath_ + "stamped_traj_estimate.txt" << " ......";

  std::ofstream f;
  f.open(resultPath_ + "stamped_traj_estimate.txt", std::ofstream::app);
  if(!f.is_open())
  {
    LOG(INFO) << "File at " << resultPath_ + "stamped_traj_estimate.txt" << " is not opened, save trajectory failed.";
    exit(-1);
  }
  f << std::fixed;

  std::list<Eigen::Matrix<double,4,4>,
    Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > >::iterator result_it_begin = lPose_.begin();
  std::list<Eigen::Matrix<double,4,4>,
    Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > >::iterator result_it_end = lPose_.end();
  std::list<std::string>::iterator  ts_it_begin = lTimestamp_.begin();

  for(;result_it_begin != result_it_end; result_it_begin++, ts_it_begin++)
  {
    Eigen::Matrix3d Rwc_result;
    Eigen::Vector3d twc_result;
    Rwc_result = (*result_it_begin).block<3,3>(0,0);
    twc_result = (*result_it_begin).block<3,1>(0,3);
    Eigen::Quaterniond q(Rwc_result);
    f.setf(std::ios::fixed, std::ios::floatfield);
    f.precision(9);
    f << *ts_it_begin << " ";
    f.precision(5);
    f << twc_result.transpose().x() << " "
    << twc_result.transpose().y() << " "
    << twc_result.transpose().z() << " "
    << q.x() << " "
    << q.y() << " "
    << q.z() << " "
    << q.w() << endl;
  }
  f.close();
  LOG(INFO) << "Saving trajectory to " << resultPath_ + "stamped_traj_estimate.txt" << ". Done !!!!!!.";
}

void esvo2_Tracking::renameOldTraj()
{
  string ori_name = resultPath_ + "stamped_traj_estimate_ours.txt";
  string new_name = resultPath_ + "traj_ours_old.txt";
  if(std::rename(ori_name.c_str(), new_name.c_str()) == 0) 
  {
    LOG(INFO) << "\33[32m" << "File renamed successfully." << "\33[0m";
  } 
  else 
  {
    LOG(INFO) << "\33[33m" << "Failed to rename the file." << "\33[0m";
  }
}

void esvo2_Tracking::groundTruthCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  std::ofstream  f;
  f.open("/home/njk/output/ESVO2/stamped_groundtruth.txt", std::ofstream::app);
  f << std::fixed;
  f.setf(std::ios::fixed, std::ios::floatfield);
  f.precision(9);
  f << msg->header.stamp.toSec() << " ";
  f.precision(5);
  f << msg->pose.position.x << " "
  << msg->pose.position.y << " "
  << msg->pose.position.z << " "
  << msg->pose.orientation.x << " "
  << msg->pose.orientation.y << " "
  << msg->pose.orientation.z << " "
  << msg->pose.orientation.w << endl;
  f.close();
}

void esvo2_Tracking::eventActivity(const cv::Mat &ts, double &freshFrac, double &support)
{
  freshFrac = support = -1.0;
  if (ts.empty() || ts.type() != CV_8UC1)
    return;
  // The time surface is 255 * exp(-age / decay): above 255 / e, the pixel fired within one decay.
  constexpr int kCell = 8;
  cv::Mat fresh;
  cv::threshold(ts, fresh, 94, 1.0, cv::THRESH_BINARY);
  fresh.convertTo(fresh, CV_32F);
  cv::Mat cells;
  cv::resize(fresh(cv::Rect(0, 0, ts.cols / kCell * kCell, ts.rows / kCell * kCell)), cells,
             cv::Size(ts.cols / kCell, ts.rows / kCell), 0, 0, cv::INTER_AREA);
  double total = 0.0, supported = 0.0;
  for (int r = 0; r < cells.rows; r++)
    for (int c = 0; c < cells.cols; c++)
    {
      const double n = std::round(cells.at<float>(r, c) * kCell * kCell);
      total += n;
      if (n >= 4)
        supported += n;
    }
  freshFrac = total / static_cast<double>(ts.rows * ts.cols);
  support = total > 0 ? supported / total : 0.0;
}

Eigen::Matrix3d esvo2_Tracking::fixRotationMatrix(const Eigen::Matrix3d& R) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    return U * V.transpose();
}

void esvo2_Tracking::imuPredictionCallback(const sensor_msgs::ImuConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(gyro_mutex_);
  const double t = msg->header.stamp.toSec();
  stillDet_.add({t, Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                 Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)});
  bImuSeen_ = true;
  if (!gyroBuf_.empty() && t < gyroBuf_.back().t - 2.0)
  {
    // Stamp jumped backwards further than the buffer span (clock step, sim time reset, bag
    // loop, relay re-anchor): the ordinary rejection below would otherwise stall prediction
    // forever, since gyroBuf_.back().t would never advance again. Recover by resetting.
    if (!bGyroJumpWarned_)
    {
      LOG(WARNING) << "IMU rotation prediction: gyro stamp jumped backwards from "
                   << gyroBuf_.back().t << " to " << t << " s; resetting gyro buffer";
      bGyroJumpWarned_ = true;
    }
    gyroBuf_.clear();
    // The estimator's in-progress window may straddle the jump (mixing pre- and post-jump
    // samples into one mean); re-create it so the next window starts clean at the next sample.
    if (gyroBiasEstimator_ && !bGyroBiasKnown_)
      gyroBiasEstimator_.reset(new tools::GyroBiasEstimator(gyroBiasWindow_, gyroBiasMaxStd_));
  }
  else if (!gyroBuf_.empty() && t <= gyroBuf_.back().t)
  {
    return; // ordinary out-of-order sample: drop it
  }
  else
  {
    bGyroJumpWarned_ = false; // sequence is healthy again
  }
  gyroBuf_.push_back({t, Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z)});
  if (gyroBiasEstimator_ && !bGyroBiasKnown_ && gyroBiasEstimator_->add(gyroBuf_.back()))
  {
    gyroBias_ = gyroBiasEstimator_->bias();
    bGyroBiasKnown_ = true;
    LOG(INFO) << "Gyro bias estimated from " << gyroBiasEstimator_->window() << " s still window: "
              << gyroBias_.transpose() << " rad/s (std " << gyroBiasEstimator_->biasStd().transpose() << ")";
  }
  // Refresh the bias from the still windows of each hold (GYRO_BIAS_REFRESH).
  if (bGyroBiasRefresh_ && bGyroBiasKnown_)
  {
    if (!bHoldFrame_)
    {
      if (bGyroBiasRefreshFed_)
      {
        gyroBiasRefresh_.reset();
        bGyroBiasRefreshFed_ = false;
      }
    }
    else
    {
      if (!gyroBiasRefresh_)
        gyroBiasRefresh_.reset(new tools::GyroBiasEstimator(gyroBiasWindow_, gyroBiasMaxStd_));
      bGyroBiasRefreshFed_ = true;
      if (gyroBiasRefresh_->add(gyroBuf_.back()))
      {
        const Eigen::Vector3d b = gyroBiasRefresh_->bias();
        LOG(INFO) << "Gyro bias refreshed during a hold: " << b.transpose() << " rad/s (change "
                  << (b - gyroBias_).norm() << " rad/s)";
        gyroBias_ = b;
        nGyroBiasRefresh_++;
        gyroBiasRefresh_.reset(); // the next window starts afresh
        bGyroBiasRefreshFed_ = false;
      }
    }
  }
  while (!gyroBuf_.empty() && gyroBuf_.front().t < t - 2.0)
    gyroBuf_.pop_front();
}

// Rotates T_world_cur_ (the previous frame's pose) by the gyro-integrated
// camera rotation between the two frames. Translation is left unchanged.
bool esvo2_Tracking::predictRotationWithGyro(double t_prev_frame, double t_cur_frame, bool &biasCorrected, bool &locked)
{
  std::vector<tools::GyroSample> samples;
  Eigen::Vector3d bias;
  {
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    samples.assign(gyroBuf_.begin(), gyroBuf_.end());
    biasCorrected = bGyroBiasKnown_;
    bias = gyroBias_;
  }
  // Subtracted here rather than on arrival so samples buffered before the estimate completed are corrected too.
  if (biasCorrected)
    tools::subtractBias(samples, bias);
  Eigen::Matrix3d R_imu;
  const bool applied = tools::gyroDeltaRotation(samples, t_prev_frame, t_cur_frame, imuTimeOffset_, R_imu);
  if (applied)
  {
    Eigen::Matrix3d R_c = tools::imuToCameraRotation(R_imu, R_b_c_);
    T_world_cur_.block<3, 3>(0, 0) = T_world_cur_.block<3, 3>(0, 0) * R_c;
    nPredOk_++;
    predAngleSumDeg_ += Eigen::AngleAxisd(R_c).angle() * 180.0 / M_PI;
  }
  else
  {
    nPredSkip_++;
  }
  locked = bImuRotationLock_ && applied && biasCorrected;
  if (locked)
    nLocked_++;
  if ((ros::WallTime::now() - lastPredLog_).toSec() >= 5.0)
  {
    LOG(INFO) << "IMU rotation prediction: " << nPredOk_ << " predicted, " << nPredSkip_ << " skipped, mean "
              << (nPredOk_ ? predAngleSumDeg_ / nPredOk_ : 0.0) << " deg/frame";
    const size_t nTotal = nPredOk_ + nPredSkip_;
    if (nPredSkip_ > 0 && 10 * nPredSkip_ >= nTotal)
      LOG(WARNING) << "IMU rotation prediction skipped " << nPredSkip_ << " of " << nTotal << " frames ("
                   << (nTotal ? 100.0 * nPredSkip_ / nTotal : 0.0)
                   << "%) in the last 5 s — check /imu/data_synced";
    if (bImuRotationLock_)
    {
      LOG(INFO) << "IMU rotation lock: " << nLocked_ << " of " << nTotal << " frames locked";
      if (nTotal > 0 && 5 * nLocked_ < 4 * nTotal)
        LOG(WARNING) << "IMU rotation lock applied to only " << nLocked_ << " of " << nTotal
                     << " frames in the last 5 s (bias not known yet, or gyro gaps)";
    }
    nLocked_ = 0;
    nPredOk_ = nPredSkip_ = 0;
    predAngleSumDeg_ = 0.0;
    lastPredLog_ = ros::WallTime::now();
  }
  return applied;
}

}// namespace esvo2_core
