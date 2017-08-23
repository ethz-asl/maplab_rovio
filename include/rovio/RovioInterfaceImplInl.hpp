/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_ROVIO_INTERFACE_IMPL_INL_HPP_
#define ROVIO_ROVIO_INTERFACE_IMPL_INL_HPP_

#include <functional>
#include <memory>
#include <queue>

#include <glog/logging.h>

#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/Memory.hpp"
#include "rovio/RovioFilter.hpp"

namespace rovio {

template <typename FILTER>
RovioInterfaceImpl<FILTER>::RovioInterfaceImpl(
    typename std::shared_ptr<mtFilter> mpFilter)
    : mpFilter_(mpFilter), transformFeatureOutputCT_(&mpFilter->multiCamera_),
      landmarkOutputImuCT_(&mpFilter->multiCamera_),
      cameraOutputCov_((int)(mtOutput::D_), (int)(mtOutput::D_)),
      featureOutputCov_((int)(FeatureOutput::D_), (int)(FeatureOutput::D_)),
      landmarkOutputCov_(3, 3),
      featureOutputReadableCov_((int)(FeatureOutputReadable::D_),
                                (int)(FeatureOutputReadable::D_)) {
  mpImgUpdate_ = CHECK_NOTNULL(&std::get<0>(mpFilter_->mUpdates_));
  mpPoseUpdate_ = CHECK_NOTNULL(&std::get<1>(mpFilter_->mUpdates_));
}

template <typename FILTER>
RovioInterfaceImpl<FILTER>::RovioInterfaceImpl(
    const std::string &filter_config_file)
    : RovioInterfaceImpl(aligned_shared<mtFilter>()) {
  CHECK(mpFilter_);
  CHECK(!filter_config_file.empty());

  mpFilter_->readFromInfo(filter_config_file);
  mpFilter_->refreshProperties();
}

template <typename FILTER>
RovioInterfaceImpl<FILTER>::RovioInterfaceImpl(
    const std::string &filter_config_file,
    const std::vector<std::string>& camera_calibration_files)
    : RovioInterfaceImpl(aligned_shared<mtFilter>()) {
  CHECK(mpFilter_);
  CHECK(!filter_config_file.empty());
  CHECK_EQ(camera_calibration_files.size(),
           RovioStateImpl<FILTER>::kNumCameras);

  // Load filter configuratino from file.
  mpFilter_->readFromInfo(filter_config_file);

  for (int camID = 0u; camID < camera_calibration_files.size(); ++camID) {
    const std::string &camera_calibration_file =
        camera_calibration_files[camID];
    if (!camera_calibration_file.empty()) {
      mpFilter_->cameraCalibrationFile_[camID] = camera_calibration_file;
    } else {
      // Use the default camera calibration paths specified in the filter config
      // file.
      // TODO(mfehr): Do we need to check if either one of them was successful?
      LOG(FATAL);
    }
  }

  mpFilter_->refreshProperties();
}

template <typename FILTER>
RovioInterfaceImpl<FILTER>::RovioInterfaceImpl(
    const FilterConfiguration &filter_config)
    : RovioInterfaceImpl(aligned_shared<mtFilter>()) {
  CHECK(mpFilter_);

  typedef boost::property_tree::ptree PropertyTree;

  // Load filter configuration from property tree.
  mpFilter_->readFromPropertyTree(static_cast<PropertyTree>(filter_config));

  // Apply changes.
  mpFilter_->refreshProperties();
}

template <typename FILTER>
RovioInterfaceImpl<FILTER>::RovioInterfaceImpl(
    const FilterConfiguration &filter_config,
    const CameraCalibrationVector& camera_calibrations)
    : RovioInterfaceImpl(filter_config) {
  CHECK(mpFilter_);
  CHECK_EQ(camera_calibrations.size(), RovioStateImpl<FILTER>::kNumCameras);

  // Override camera calibrations.
  mpFilter_->setCameraCalibrations(camera_calibrations);
}

template <typename FILTER>
void RovioInterfaceImpl<FILTER>::setEnableFeatureUpdateOutput(
    const bool enable_feature_update_output) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);
  enable_feature_update_output_ = enable_feature_update_output;
}
template <typename FILTER>
void RovioInterfaceImpl<FILTER>::setEnablePatchUpdateOutput(
    const bool enable_patch_update_output) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);
  enable_patch_update_output_ = enable_patch_update_output;
}

template <typename FILTER> void RovioInterfaceImpl<FILTER>::requestReset() {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  if (!init_state_.isInitialized()) {
    std::cout << "Reinitialization already triggered. Ignoring request...";
    return;
  }

  init_state_.state_ = FilterInitializationState::State::WaitForInitUsingAccel;
}

template <typename FILTER>
void RovioInterfaceImpl<FILTER>::requestResetToPose(const V3D &WrWM,
                                                    const QPD &qMW) {
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

template <typename FILTER>
void RovioInterfaceImpl<FILTER>::resetToLastSafePose() {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  requestReset();
  mpFilter_->init_.state_.WrWM() = mpFilter_->safe_.state_.WrWM();
  mpFilter_->init_.state_.qWM() = mpFilter_->safe_.state_.qWM();
}

template <typename FILTER>
double RovioInterfaceImpl<FILTER>::getLastSafeTime() {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);
  return mpFilter_->safe_.t_;
}

template <typename FILTER>
bool RovioInterfaceImpl<FILTER>::processVelocityUpdate(
    const Eigen::Vector3d &AvM, const double time_s) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  if (!init_state_.isInitialized()) {
    return false;
  }

  // Set velocity update.
  velocityUpdateMeas_.vel() = AvM;
  const bool measurement_accepted =
      this->mpFilter_->template addUpdateMeas<2>(
          this->velocityUpdateMeas_, time_s);
  updateFilter();
  return measurement_accepted;
}

template <typename FILTER>
bool RovioInterfaceImpl<FILTER>::processImuUpdate(
    const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr,
    const double time_s, bool update_filter) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  predictionMeas_.template get<mtPredictionMeas::_acc>() = acc;
  predictionMeas_.template get<mtPredictionMeas::_gyr>() = gyr;

  if (!init_state_.isInitialized()) {
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
    return true;
  }
  const bool measurement_accepted =
      mpFilter_->addPredictionMeas(predictionMeas_, time_s);

  if (update_filter) {
    updateFilter();
  }
  return measurement_accepted;
}

template <typename FILTER>
bool RovioInterfaceImpl<FILTER>::processImageUpdate(const int camID,
                                                    const cv::Mat &cv_img,
                                                    const double time_s) {
  CHECK_LT(camID, RovioStateImpl<FILTER>::kNumCameras)
      << "Invalid camID " << camID;

  std::lock_guard<std::recursive_mutex> lock(m_filter_);
  if (!init_state_.isInitialized() || cv_img.empty()) {
    return false;
  }

  double msgTime = time_s;
  if (msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_) {
    for (int i = 0; i < RovioStateImpl<FILTER>::kNumCameras; i++) {
      if (imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i]) {
        std::cout
            << "    \033[31mFailed Synchronization of Camera Frames, t = "
            << msgTime << "\033[0m" << std::endl;
      }
    }
    imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
  }
  imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage(
      cv_img, true);
  imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

  bool measurement_accepted = false;
  if (imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()) {
    measurement_accepted =
        mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_, msgTime);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
    updateFilter();
  }
  return measurement_accepted;
}

template <typename FILTER>
bool RovioInterfaceImpl<FILTER>::processGroundTruthUpdate(
    const Eigen::Vector3d &JrJV, const QPD &qJV, const double time_s) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  if (!init_state_.isInitialized()) {
    return false;
  }

  poseUpdateMeas_.pos() = JrJV;
  poseUpdateMeas_.att() = qJV.inverted();
  const bool measurement_accepted =
      mpFilter_->template addUpdateMeas<1>(
          poseUpdateMeas_, time_s + mpPoseUpdate_->timeOffset_);
  updateFilter();
  return measurement_accepted;
}

template <typename FILTER>
bool RovioInterfaceImpl<FILTER>::processGroundTruthOdometryUpdate(
    const Eigen::Vector3d &JrJV, const QPD &qJV,
    const Eigen::Matrix<double, 6, 6> &measuredCov, const double time_s) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  if (!init_state_.isInitialized()) {
    return false;
  }

  poseUpdateMeas_.pos() = JrJV;
  poseUpdateMeas_.att() = qJV.inverted();
  poseUpdateMeas_.measuredCov() = measuredCov;
  const bool measurement_accepted =
      mpFilter_->template addUpdateMeas<1>(
          poseUpdateMeas_, time_s + mpPoseUpdate_->timeOffset_);
  updateFilter();
  return measurement_accepted;
}

template <typename FILTER>
void RovioInterfaceImpl<FILTER>::registerStateUpdateCallback(
    RovioStateCallback callback) {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  filter_update_state_callbacks_.push_back(callback);

  std::cout << "Registered filter state update callback." << std::endl;
}

template <typename FILTER>
bool RovioInterfaceImpl<FILTER>::getState(RovioState *filter_update) {
  CHECK_NOTNULL(filter_update);
  return getState(enable_feature_update_output_, enable_patch_update_output_,
                  filter_update);
}

template <typename FILTER>
bool RovioInterfaceImpl<FILTER>::getState(const bool get_feature_update,
                                          const bool get_patch_update,
                                          RovioState *base_filter_update) {
  CHECK_NOTNULL(base_filter_update);

  RovioStateImpl<FILTER> *filter_update =
      static_cast<RovioStateImpl<FILTER> *>(base_filter_update);

  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  // Check if filter is initialized.
  filter_update->isInitialized = init_state_.isInitialized();

  if (!filter_update->isInitialized) {
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
  for (unsigned int i = 0u; i < mtState::nCam_; ++i) {
    filter_update->MrMC[i] = state.MrMC(i);
    filter_update->qCM[i] = state.qCM(i);
  }

  // Filter covariance.
  filter_update->filterCovariance = mpFilter_->safe_.cov_;

  if (mpPoseUpdate_->inertialPoseIndex_ >= 0) {
    filter_update->hasInertialPose = true;
    filter_update->IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
    filter_update->qWI = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
  }

  // IMU state and IMU covariance.
  StandardOutput imuOutput;
  imuOutputCT_.transformState(state, imuOutput);

  // IMU frame:
  // Transformation between world frame (W) and the IMU frame (B).
  filter_update->WrWB = imuOutput.WrWB();
  filter_update->qBW = imuOutput.qBW();
  // Velocities of IMU frame (B).
  filter_update->BvB = imuOutput.BvB();
  filter_update->BwWB = imuOutput.BwWB();

  imuOutputCT_.transformCovMat(state, filter_update->filterCovariance,
                               filter_update->imuCovariance);
  CHECK_GT(filter_update->imuCovariance.cols(), 0);
  CHECK_GT(filter_update->imuCovariance.rows(), 0);

  // IMU biases.
  filter_update->gyb = state.gyb();
  filter_update->acb = state.acb();

  if (get_feature_update) {
    RovioFeatureStateImpl<FILTER> *feature_state_ptr =
        new RovioFeatureStateImpl<FILTER>();
    RovioFeatureStateImpl<FILTER> &feature_state = *feature_state_ptr;

    filter_update->feature_state.reset(feature_state_ptr);
    filter_update->hasFeatureUpdate = true;

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
      const double sigma = sqrt(filter_update->filterCovariance(
          mtState::template getId<mtState::_fea>(i) + 2,
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
      transformFeatureOutputCT_.transformCovMat(
          state, filter_update->filterCovariance, featureOutputCov_);
      featureOutputReadableCT_.transformState(featureOutput_,
                                              featureOutputReadable_);
      featureOutputReadableCT_.transformCovMat(
          featureOutput_, featureOutputCov_, featureOutputReadableCov_);

      // Get landmark output
      landmarkOutputImuCT_.setFeatureID(i);
      landmarkOutputImuCT_.transformState(state, landmarkOutput_);
      landmarkOutputImuCT_.transformCovMat(
          state, filter_update->filterCovariance, landmarkOutputCov_);

      feature_state.status_vec[i] =
          filterState.fsm_.features_[i].mpStatistics_->status_[0];
      feature_state.featureIndices[i] = filterState.fsm_.features_[i].idx_;
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
    RovioPatchStateImpl<FILTER> *patch_state_ptr =
        new RovioPatchStateImpl<FILTER>();
    RovioPatchStateImpl<FILTER> &patch_state = *patch_state_ptr;

    filter_update->patch_state.reset(patch_state_ptr);
    filter_update->hasPatchUpdate = true;

    for (unsigned int i = 0u; i < RovioStateImpl<FILTER>::kMaxNumFeatures;
         ++i) {
      const bool featureIsValid = filterState.fsm_.isValid_[i];
      if (!featureIsValid) {
        continue;
      }

      patch_state.isFeatureValid[i] = true;
      patch_state.patchIndices[i] = filterState.fsm_.features_[i].idx_;

      for (unsigned int j = 0u; j < RovioStateImpl<FILTER>::kNumPatchLevels;
           ++j) {
        patch_state.patches[i][j] =
            filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[j];
        patch_state.isPatchValid[i][j] =
            filterState.fsm_.features_[i].mpMultilevelPatch_->isValidPatch_[j];
      }
    }
  }

  return true;
}

template <typename FILTER> bool RovioInterfaceImpl<FILTER>::updateFilter() {
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

  // TODO(mfehr): this was here before, remove or use VLOG.
  constexpr bool outputTiming = false;
  if (outputTiming) {
    std::cout << " == Filter Update: " << filterUpdateTimeMs
              << " ms for processing " << numberImagesProcessed
              << " images, average: " << timing_T / timing_C << std::endl;
  }

  // If there is no change, return false.
  if (mpFilter_->safe_.t_ <= oldSafeTime) {
    return false;
  }

  visualizeUpdate();

  // Notify all filter state update callbacks.
  std::unique_ptr<RovioState> state(new RovioStateImpl<FILTER>());
  const bool hasNewState = getState(state.get());
  if (hasNewState) {
    notifyAllStateUpdateCallbacks(*state);
  }
  return hasNewState;
}

template <typename FILTER>
void RovioInterfaceImpl<FILTER>::notifyAllStateUpdateCallbacks(
    const RovioState &state) const {
  for (const RovioStateCallback &callback : filter_update_state_callbacks_) {
    callback(state);
  }
}

template <typename FILTER> void RovioInterfaceImpl<FILTER>::visualizeUpdate() {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);

  for (int i = 0; i < mtState::nCam_; i++) {
    if (!mpFilter_->safe_.img_[i].empty() &&
        mpImgUpdate_->doFrameVisualisation_) {
      cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
      cv::waitKey(1);
    }
  }
  if (!mpFilter_->safe_.patchDrawing_.empty() &&
      mpImgUpdate_->visualizePatches_) {
    cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
    cv::waitKey(1);
  }

  mtState &state = mpFilter_->safe_.state_;

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

template <typename FILTER> bool RovioInterfaceImpl<FILTER>::isInitialized() const {
  std::lock_guard<std::recursive_mutex> lock(m_filter_);
  return init_state_.state_ == FilterInitializationState::State::Initialized;
}

template <typename FILTER> void RovioInterfaceImpl<FILTER>::makeTest() {
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
  std::cout << "Finished tests" << std::endl;
}

} // namespace rovio

#endif // ROVIO_ROVIO_INTERFACE_IMPL_INL_HPP_
