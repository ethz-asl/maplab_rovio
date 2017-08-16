#include "rovio/rovio-interface.h"

namespace rovio {

RovioInterface::RovioInterface(std::shared_ptr<mtFilter> mpFilter)
    : mpFilter_(mpFilter), transformFeatureOutputCT_(&mpFilter->multiCamera_),
      landmarkOutputImuCT_(&mpFilter->multiCamera_),
      cameraOutputCov_((int)(mtOutput::D_), (int)(mtOutput::D_)),
      featureOutputCov_((int)(FeatureOutput::D_), (int)(FeatureOutput::D_)),
      landmarkOutputCov_(3, 3),
      featureOutputReadableCov_((int)(FeatureOutputReadable::D_),
                                (int)(FeatureOutputReadable::D_)) {
  // TODO(mfehr): IMPLEMENT

  mpImgUpdate_ = &std::get<0>(mpFilter_->mUpdates_);
  mpPoseUpdate_ = &std::get<1>(mpFilter_->mUpdates_);
}

void RovioInterface::requestReset() {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  if (!init_state_.isInitialized()) {
    std::cout << "Reinitialization already triggered. Ignoring request...";
    return;
  }

  init_state_.state_ = FilterInitializationState::State::WaitForInitUsingAccel;
}

void RovioInterface::requestResetToPose(const V3D &WrWM, const QPD &qMW) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  if (!init_state_.isInitialized()) {
    std::cout << "Reinitialization already triggered. Ignoring request...";
    return;
  }

  init_state_.WrWM_ = WrWM;
  init_state_.qMW_ = qMW;
  init_state_.state_ =
      FilterInitializationState::State::WaitForInitExternalPose;
}

bool RovioInterface::processVelocityUpdate(const Eigen::Vector3d &AvM,
                                           const double time_s) {
  CHECK_NOTNULL(state);
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  if (init_state_.isInitialized()) {
    // Set velocity update.
    velocityUpdateMeas_.vel() = AvM;
    mpFilter_->template addUpdateMeas<2>(velocityUpdateMeas_, time_s);                                     );

    // Notify filter.
    return updateFilter();
  }
  return false;
}

bool RovioInterface::processImuUpdate(const Eigen::Vector3d &acc,
                                      const Eigen::Vector3d &gyr,
                                      const double time_s) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  predictionMeas_.template get<mtPredictionMeas::_acc>() = acc;
  predictionMeas_.template get<mtPredictionMeas::_gyr>() = gyr;

  if (init_state_.isInitialized()) {
    mpFilter_->addPredictionMeas(predictionMeas_, time_s);

    // Notify filter.
    return updateFilter();
  }

  switch (init_state_.state_) {
  case FilterInitializationState::State::WaitForInitExternalPose:
    std::cout << "-- Filter: Initializing using external pose ..." << std::endl;
    mpFilter_->resetWithPose(init_state_.WrWM_, init_state_.qMW_, time_s);
    break;
  case FilterInitializationState::State::WaitForInitUsingAccel:
    std::cout << "-- Filter: Initializing using accel. measurement ..."
              << std::endl;
    mpFilter_->resetWithAccelerometer(
        predictionMeas_.template get<mtPredictionMeas::_acc>(), time_s);
    break;
  default:
    std::cout << "Unhandeld initialization type." << std::endl;
    // TODO(mfehr): Check what this actually does and what consequences it has
    // for the ROS node.
    abort();
    break;
  }

  std::cout << std::setprecision(12);
  std::cout << "-- Filter: Initialized at t = " << time_s << std::endl;
  init_state_.state_ = FilterInitializationState::State::Initialized;

  return false;
}

bool RovioInterface::processImageUpdate(const int camID, const cv::Mat &cv_img,
                                        const double time_s) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);
  if (init_state_.isInitialized() && !cv_img.empty()) {
    if (time_s != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_) {
      for (int i = 0; i < mtState::nCam_; i++) {
        if (imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i]) {
          std::cout
              << "    \033[31mFailed Synchronization of Camera Frames, t = "
              << time_s << "\033[0m" << std::endl;
        }
      }
      imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(time_s);
    }
    imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage(
        cv_img, true);
    imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

    if (imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()) {
      mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_, time_s);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(time_s);

      // Notify filter.
      return updateFilter();
    }
  }
  return false;
}

bool RovioInterface::processGroundTruthUpdate(const Eigen::Vector3d &JrJV,
                                              const QPD &qJV,
                                              const double time_s) {
  if (init_state_.isInitialized()) {
    poseUpdateMeas_.pos() = JrJV;
    poseUpdateMeas_.att() = qJV.inverted();
    mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,
                                         time_s + mpPoseUpdate_->timeOffset_);

    // Notify filter.
    return updateFilter();
  }
  return false;
}

bool RovioInterface::processGroundTruthOdometryUpdate(
    const Eigen::Vector3d &JrJV, const QPD &qJV, const double time_s) {
  if (init_state_.isInitialized()) {
    poseUpdateMeas_.pos() = JrJV;
    poseUpdateMeas_.att() = qJV.inverted();
    poseUpdateMeas_.measuredCov() = measuredCov;
    mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,
                                         time_s + mpPoseUpdate_->timeOffset_);
    // Notify filter.
    return updateFilter();
  }
  return false;
}

void RovioInterface::registerStateUpdateCallback(
    FilterUpdateStateCallback callback, const bool get_feature_update,
    const bool get_patch_update) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  FilterUpdateStateCallbackSettings settings;
  settings.get_feature_update = get_feature_update;
  settings.get_patch_update = get_patch_update;

  filter_update_state_callbacks_.emplace_back(callback, settings);

  std::cout << "Registered filter state update callback.";
}

bool RovioInterface::getState(const bool get_feature_update,
                              const bool get_patch_update,
                              FilterUpdateState *filter_update) {
  CHECK_NOTNULL(filter_update);
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  // Check if filter is initialized.
  filter_update->isInitialized = init_state_.isInitialized();
  if (filter_update->isInitialized) {
    return false;
  }

  // Get time stamp of update.
  filter_update->timeAfterUpdate = mpFilter_->safe_.t_;
  filter_update->mpImgUpdate = mpImgUpdate_;
  filter_update->mpPoseUpdate = mpPoseUpdate_;

  // Get filter state.
  mtFilterState &filterState = mpFilter_->safe_;
  mtState &state = mpFilter_->safe_.state_;

  // Get camera extrinsics.
  state.updateMultiCameraExtrinsics(&mpFilter_->multiCamera_);
  for (unsigned int i = 0u; i < nCam; ++i) {
    filter_update->BrBC[i] = mpFilter_->multiCamera_.BrBC_[i];
    filter_update->qCB[i] = mpFilter_->multiCamera_.qCB_[i];
  }

  // Filter covariance.
  filter_update->filterCovariance = mpFilter_->safe_.cov_;

  if (mpPoseUpdate_->inertialPoseIndex_ >= 0) {
    filter_update->hasInertialPose = true;
    filter_update->IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
    filter_update->qWI = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
  }

  filter_update->MrMC = state.MrMC;
  filter_update->qCM = state.qCM;

  // IMU state and IMU covariance.
  imuOutputCT_.transformState(state, filter_update->imuOutput);
  imuOutputCT_.transformCovMat(state, filter_update->filterCovariance,
                               filter_update->imuOutputCov);

  // IMU biases.
  filter_update->gyb = state.gyb();
  filter_update->acb = state.acb();

  if (get_feature_update) {
    filter_update->feature_state.reset(new FeatureUpdateState<mState::nMax_>());
    FeatureUpdateState &feature_state = *filter_update->feature_state;

    feature_state.hasFeatureUpdate = true;

    FeatureDistance distance;
    double d, d_minus, d_plus;
    const double stretchFactor = 3.0;
    for (unsigned int i = 0u; i < mtState::nMax_; ++i) {
      const bool featureIsValid = filterState.fsm_.isValid_[i];
      feature_state.isFeatureValid[i] = featureIsValid;

      if (!featureIsValid) {
        continue;
      }

      // Get 3D feature coordinates.
      int camID = filterState.fsm_.features_[i].mpCoordinates_->camID_;
      distance = state.dep(i);
      d = distance.getDistance();
      const double sigma =
          sqrt(cov(mtState::template getId<mtState::_fea>(i) + 2,
                   mtState::template getId<mtState::_fea>(i) + 2));
      distance.p_ -= stretchFactor * sigma;
      d_minus = distance.getDistance();
      if (d_minus > 1000)
        d_minus = 1000;
      if (d_minus < 0)
        d_minus = 0;
      distance.p_ += 2 * stretchFactor * sigma;
      d_plus = distance.getDistance();
      if (d_plus > 1000)
        d_plus = 1000;
      if (d_plus < 0)
        d_plus = 0;
      Eigen::Vector3d bearingVector =
          filterState.state_.CfP(i).get_nor().getVec();

      // Get human readable output
      transformFeatureOutputCT_.setFeatureID(i);
      transformFeatureOutputCT_.setOutputCameraID(
          filterState.fsm_.features_[i].mpCoordinates_->camID_);
      transformFeatureOutputCT_.transformState(state, featureOutput_);
      transformFeatureOutputCT_.transformCovMat(state, cov, featureOutputCov_);
      featureOutputReadableCT_.transformState(featureOutput_,
                                              featureOutputReadable_);
      featureOutputReadableCT_.transformCovMat(
          featureOutput_, featureOutputCov_, featureOutputReadableCov_);

      // Get landmark output
      landmarkOutputImuCT_.setFeatureID(i);
      landmarkOutputImuCT_.transformState(state, landmarkOutput_);
      landmarkOutputImuCT_.transformCovMat(state, cov, landmarkOutputCov_);

      feature_state.status_vec[i] =
          filterState.fsm_.features_[i].mpStatistics_->status_[0];
      feature_state.MrMP_vec[i] =
          landmarkOutput_.get<LandmarkOutput::_lmk>().template cast<float>();
      feature_state.CrCPp_vec[i] = bearingVector * d_plus;
      feature_state.CrCPm_vec[i] = bearingVector * d_minus;
      feature_state.bearings[i] =
          featureOutputReadable_.bea().template cast<float>();
      feature_state.cov_MrMP_vec[i] = landmarkOutputCov_.cast<float>();
      feature_state.distances[i] =
          static_cast<float>(featureOutputReadable_.dis());
      feature_state.distances_cov[i] =
          static_cast<float>(featureOutputReadableCov_(3, 3));
    }
  }

  if (get_patch_update) {
    filter_update->patch_state.reset(new PatchUpdateState<mState::nMax_>());
    PatchUpdateState &patch_state = *filter_update->patch_state;

    for (unsigned int i = 0u; i < mtState::nMax_; ++i) {
      const bool featureIsValid = filterState.fsm_.isValid_[i];
      featureIsValid;

      if (!featureIsValid) {
        continue;
      }

      patch_state.isFeatureValid[i] = true;
      patch_state.patchId[i] = filterState.fsm_.features_[i].idx_;
      patch_state.patches[i] =
          filterState.fsm_.features_[i].mpMultilevelPatch_->patches_;
      patch_state.isPatchValid[i] =
          filterState.fsm_.features_[i].mpMultilevelPatch_->isValidPatch_;
    }
  }

  return true;
}

bool RovioInterface::updateFilter() {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  // Statistics values that persist over all updates.
  static double timing_T = 0;
  static int timing_C = 0;

  if (!init_state_.isInitialized()) {
    return false;
  }

  // Execute the filter update.
  const double t1 = (double)cv::getTickCount();
  const double oldSafeTime = mpFilter_->safe_.t_;
  const int c1 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
  double lastImageTime;
  if (std::get<0>(mpFilter_->updateTimelineTuple_).getLastTime(lastImageTime)) {
    mpFilter_->updateSafe(&lastImageTime);
  }
  const double t2 = (double)cv::getTickCount();

  const int c2 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
  const double filterUpdateTimeMs = (t2 - t1) / cv::getTickFrequency() * 1000;
  const size_t numberImagesProcessed = c1 - c2;

  // Update statistics.
  timing_T += filterUpdateTimeMs;
  timing_C += numberImagesProcessed;

  VLOG(5) << " == Filter Update: " << filterUpdateTimeMs
          << " ms for processing " << numberImagesProcessed
          << " images, average: " << timing_T / timing_C;

  // If there is no change, return false.
  if (mpFilter_->safe_.t_ <= oldSafeTime) {
    return false;
  }

  visualizeUpdate();

  // Notify all filter state update callbacks.
  for (FilterUpdateStateCallbackHandle &callback_handle :
       filter_update_state_callbacks_) {
    const FilterUpdateStateCallbackSettings &settings = callback_handle.second;

    FilterUpdateState<mtState::nCam_, mtState::nMax_, mtState::patchSize_,
                      mtState::nLevels_> state;
    getState(settings.get_feature_update, settings.get_patch_update, &state);

    // Execute callback.
    callback_handle.first(state);
  }
  return true;
}

void RovioInterface::visualizeUpdate() const {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  for (int i = 0; i < mtState::nCam_; i++) {
    if (!mpFilter_->safe_.img_[i].empty() &&
        mpImgUpdate_->doFrameVisualisation_) {
      cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
      cv::waitKey(3);
    }
  }
  if (!mpFilter_->safe_.patchDrawing_.empty() &&
      mpImgUpdate_->visualizePatches_) {
    cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
    cv::waitKey(3);
  }

  if (mpImgUpdate_->verbose_) {
    if (mpPoseUpdate_->inertialPoseIndex_ >= 0) {
      std::cout << "Transformation between inertial frames, IrIW, qWI: "
                << std::endl;
      std::cout << "  "
                << state.poseLin(mpPoseUpdate_->inertialPoseIndex_).transpose()
                << std::endl;
      std::cout << "  " << state.poseRot(mpPoseUpdate_->inertialPoseIndex_)
                << std::endl;
    }
    if (mpPoseUpdate_->bodyPoseIndex_ >= 0) {
      std::cout << "Transformation between body frames, MrMV, qVM: "
                << std::endl;
      std::cout << "  "
                << state.poseLin(mpPoseUpdate_->bodyPoseIndex_).transpose()
                << std::endl;
      std::cout << "  " << state.poseRot(mpPoseUpdate_->bodyPoseIndex_)
                << std::endl;
    }
  }
}

RovioInterface::makeTest() {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  mtFilterState *mpTestFilterState = new mtFilterState();
  *mpTestFilterState = mpFilter_->init_;
  mpTestFilterState->setCamera(&mpFilter_->multiCamera_);
  mtState &testState = mpTestFilterState->state_;
  unsigned int s = 2;
  testState.setRandom(s);
  predictionMeas_.setRandom(s);
  imgUpdateMeas_.setRandom(s);

  LWF::NormalVectorElement tempNor;
  for (int i = 0; i < mtState::nMax_; i++) {
    testState.CfP(i).camID_ = 0;
    tempNor.setRandom(s);
    if (tempNor.getVec()(2) < 0) {
      tempNor.boxPlus(Eigen::Vector2d(3.14, 0), tempNor);
    }
    testState.CfP(i).set_nor(tempNor);
    testState.CfP(i).trackWarping_ = false;
    tempNor.setRandom(s);
    if (tempNor.getVec()(2) < 0) {
      tempNor.boxPlus(Eigen::Vector2d(3.14, 0), tempNor);
    }
    testState.aux().feaCoorMeas_[i].set_nor(tempNor, true);
    testState.aux().feaCoorMeas_[i].mpCamera_ =
        &mpFilter_->multiCamera_.cameras_[0];
    testState.aux().feaCoorMeas_[i].camID_ = 0;
  }
  testState.CfP(0).camID_ = mtState::nCam_ - 1;
  mpTestFilterState->fsm_.setAllCameraPointers();

  // Prediction
  std::cout << "Testing Prediction" << std::endl;
  mpFilter_->mPrediction_.testPredictionJacs(testState, predictionMeas_, 1e-8,
                                             1e-6, 0.1);

  // Update
  if (!mpImgUpdate_->useDirectMethod_) {
    std::cout << "Testing Update (can sometimes exhibit large absolut errors "
                 "due to the float precision)"
              << std::endl;
    for (int i = 0; i < (std::min((int)mtState::nMax_, 2)); i++) {
      testState.aux().activeFeature_ = i;
      testState.aux().activeCameraCounter_ = 0;
      mpImgUpdate_->testUpdateJacs(testState, imgUpdateMeas_, 1e-4, 1e-5);
      testState.aux().activeCameraCounter_ = mtState::nCam_ - 1;
      mpImgUpdate_->testUpdateJacs(testState, imgUpdateMeas_, 1e-4, 1e-5);
    }
  }

  // Testing CameraOutputCF and CameraOutputCF
  std::cout << "Testing cameraOutputCF" << std::endl;
  cameraOutputCT_.testTransformJac(testState, 1e-8, 1e-6);
  std::cout << "Testing imuOutputCF" << std::endl;
  imuOutputCT_.testTransformJac(testState, 1e-8, 1e-6);
  std::cout << "Testing attitudeToYprCF" << std::endl;
  rovio::AttitudeToYprCT attitudeToYprCF;
  attitudeToYprCF.testTransformJac(1e-8, 1e-6);

  // Testing TransformFeatureOutputCT
  std::cout << "Testing transformFeatureOutputCT" << std::endl;
  transformFeatureOutputCT_.setFeatureID(0);
  if (mtState::nCam_ > 1) {
    transformFeatureOutputCT_.setOutputCameraID(1);
    transformFeatureOutputCT_.testTransformJac(testState, 1e-8, 1e-5);
  }
  transformFeatureOutputCT_.setOutputCameraID(0);
  transformFeatureOutputCT_.testTransformJac(testState, 1e-8, 1e-5);

  // Testing LandmarkOutputImuCT
  std::cout << "Testing LandmarkOutputImuCT" << std::endl;
  landmarkOutputImuCT_.setFeatureID(0);
  landmarkOutputImuCT_.testTransformJac(testState, 1e-8, 1e-5);

  // Getting featureOutput for next tests
  transformFeatureOutputCT_.transformState(testState, featureOutput_);
  if (!featureOutput_.c().isInFront()) {
    featureOutput_.c().set_nor(
        featureOutput_.c().get_nor().rotated(QPD(0.0, 1.0, 0.0, 0.0)), false);
  }

  // Testing FeatureOutputReadableCT
  std::cout << "Testing FeatureOutputReadableCT" << std::endl;
  featureOutputReadableCT_.testTransformJac(featureOutput_, 1e-8, 1e-5);

  // Testing pixelOutputCT
  rovio::PixelOutputCT pixelOutputCT;
  std::cout << "Testing pixelOutputCT (can sometimes exhibit large absolut "
               "errors due to the float precision)"
            << std::endl;
  pixelOutputCT.testTransformJac(
      featureOutput_, 1e-4,
      1.0); // Reduces accuracy due to float and strong camera distortion

  // Testing ZeroVelocityUpdate_
  std::cout << "Testing zero velocity update" << std::endl;
  mpImgUpdate_->zeroVelocityUpdate_.testJacs();

  // Testing PoseUpdate
  if (!mpPoseUpdate_->noFeedbackToRovio_) {
    std::cout << "Testing pose update" << std::endl;
    mpPoseUpdate_->testUpdateJacs(1e-8, 1e-5);
  }

  delete mpTestFilterState;
}

} // namespace rovio
