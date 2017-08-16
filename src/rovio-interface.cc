#include "rovio/rovio-interface.h"

namespace rovio {

RovioInterface::RovioInterface() {
  // TODO(mfehr): IMPLEMENT

  mpImgUpdate_ = &std::get<0>(mpFilter_->mUpdates_);
  mpPoseUpdate_ = &std::get<1>(mpFilter_->mUpdates_);
}

bool RovioInterface::updateFilter(FilterUpdateState *filter_update) {
  CHECK_NOTNULL(filter_update);

  // Statistics values that persist over all updates.
  static double timing_T = 0;
  static int timing_C = 0;

  // TODO(mfehr): Get filter update

  filter_update->isInitialized = init_state_.isInitialized();

  if (!filter_update->isInitialized) {
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

  // Set filter update.
  filter_update->timeAfterUpdate = mpFilter_->safe_.t_;
  filter_update->filterUpdateTimeMs = filterUpdateTimeMs;
  filter_update->filterUpdateTimeTotalMs = timing_T;
  filter_update->numberImagesProcessed = numberImagesProcessed;
  filter_update->numberImagesProcessedTotal = timing_C;
  filter_update->mpImgUpdate = mpImgUpdate_;
  filter_update->mpPoseUpdate = mpPoseUpdate_;

  // Publish only if something changed.
  if (filter_update->timeAfterUpdate <= oldSafeTime) {
    return false;
  }

  // TODO(mfehr): Check if we can move this to the end of the function or into a
  // separate one Visualize filter update.
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

  // Obtain the save filter state.
  mtFilterState &filterState = mpFilter_->safe_;
  mtState &state = mpFilter_->safe_.state_;
  state.updateMultiCameraExtrinsics(&mpFilter_->multiCamera_);

  // Set filter update.
  //   MXD &cov = mpFilter_->safe_.cov_;
  filter_update->covarianceFilter = mpFilter_->safe_.cov_;
  imuOutputCT_.transformState(state, filter_update->imuOutput);

  // TODO(mfehr): Check if we can move this to the end of the function or into a
  // separate one Cout verbose for pose measurements.
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

  if (mpPoseUpdate_->inertialPoseIndex_ >= 0) {
    filter_update->IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
    filter_update->qWI = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
  }

  filter_update->MrMC = state.MrMC;
  filter_update->qCM = state.qCM;

  // TODO(mfehr): Make sure that I got this right, both pose and odometry output
  // topic use the same covariance, but it was computed twice but only if there
  // was a subscriber or the publishing was forced.
  imuOutputCT_.transformCovMat(state, filter_update->covarianceFilter,
                               filter_update->imuOutputCov);

  // TODO(mfehr): Make sure the covariance we publish for the extrinsics is also
  // stored in the covariance were we get the odometry and pose covariance from.

  // TODO(mfehr): move this to the other IMU related results and make sure we
  // get this covariance stuff right.
  filter_update->gyb = state.gyb();
  filter_update->acb = state.acb();

  // PointCloud message.
  if (pubPcl_.getNumSubscribers() > 0 || pubMarkers_.getNumSubscribers() > 0 ||
      forcePclPublishing_ || forceMarkersPublishing_) {
    pclMsg_.header.seq = msgSeq_;
    pclMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
    markerMsg_.header.seq = msgSeq_;
    markerMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
    markerMsg_.points.clear();
    float badPoint = std::numeric_limits<float>::quiet_NaN(); // Invalid point.
    int offset = 0;

    FeatureDistance distance;
    double d, d_minus, d_plus;
    const double stretchFactor = 3;
    for (unsigned int i = 0; i < mtState::nMax_;
         i++, offset += pclMsg_.point_step) {
      if (filterState.fsm_.isValid_[i]) {
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
        const Eigen::Vector3d CrCPm = bearingVector * d_minus;
        const Eigen::Vector3d CrCPp = bearingVector * d_plus;
        const Eigen::Vector3f MrMPm =
            V3D(mpFilter_->multiCamera_.BrBC_[camID] +
                mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPm))
                .cast<float>();
        const Eigen::Vector3f MrMPp =
            V3D(mpFilter_->multiCamera_.BrBC_[camID] +
                mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPp))
                .cast<float>();

        // Get human readable output
        transformFeatureOutputCT_.setFeatureID(i);
        transformFeatureOutputCT_.setOutputCameraID(
            filterState.fsm_.features_[i].mpCoordinates_->camID_);
        transformFeatureOutputCT_.transformState(state, featureOutput_);
        transformFeatureOutputCT_.transformCovMat(state, cov,
                                                  featureOutputCov_);
        featureOutputReadableCT_.transformState(featureOutput_,
                                                featureOutputReadable_);
        featureOutputReadableCT_.transformCovMat(
            featureOutput_, featureOutputCov_, featureOutputReadableCov_);

        // Get landmark output
        landmarkOutputImuCT_.setFeatureID(i);
        landmarkOutputImuCT_.transformState(state, landmarkOutput_);
        landmarkOutputImuCT_.transformCovMat(state, cov, landmarkOutputCov_);
        const Eigen::Vector3f MrMP =
            landmarkOutput_.get<LandmarkOutput::_lmk>().template cast<float>();

        // Write feature id, camera id, and rgb
        uint8_t gray = 255;
        uint32_t rgb = (gray << 16) | (gray << 8) | gray;
        uint32_t status =
            filterState.fsm_.features_[i].mpStatistics_->status_[0];
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset],
               &filterState.fsm_.features_[i].idx_, sizeof(int)); // id
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[1].offset], &camID,
               sizeof(int)); // cam id
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[2].offset], &rgb,
               sizeof(uint32_t)); // rgb
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[3].offset], &status,
               sizeof(int)); // status

        // Write coordinates to pcl message.
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[4].offset], &MrMP[0],
               sizeof(float)); // x
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[5].offset], &MrMP[1],
               sizeof(float)); // y
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[6].offset], &MrMP[2],
               sizeof(float)); // z

        // Add feature bearing vector and distance
        const Eigen::Vector3f bearing =
            featureOutputReadable_.bea().template cast<float>();
        const float distance = static_cast<float>(featureOutputReadable_.dis());
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[7].offset], &bearing[0],
               sizeof(float)); // x
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[8].offset], &bearing[1],
               sizeof(float)); // y
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[9].offset], &bearing[2],
               sizeof(float)); // z
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[10].offset], &distance,
               sizeof(float)); // d

        // Add the corresponding covariance (upper triangular)
        Eigen::Matrix3f cov_MrMP = landmarkOutputCov_.cast<float>();
        int mCounter = 11;
        for (int row = 0; row < 3; row++) {
          for (int col = row; col < 3; col++) {
            memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset],
                   &cov_MrMP(row, col), sizeof(float));
            mCounter++;
          }
        }

        // Add distance uncertainty
        const float distance_cov =
            static_cast<float>(featureOutputReadableCov_(3, 3));
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset],
               &distance_cov, sizeof(float));

        // Line markers (Uncertainty rays).
        geometry_msgs::Point point_near_msg;
        geometry_msgs::Point point_far_msg;
        point_near_msg.x = float(CrCPp[0]);
        point_near_msg.y = float(CrCPp[1]);
        point_near_msg.z = float(CrCPp[2]);
        point_far_msg.x = float(CrCPm[0]);
        point_far_msg.y = float(CrCPm[1]);
        point_far_msg.z = float(CrCPm[2]);
        markerMsg_.points.push_back(point_near_msg);
        markerMsg_.points.push_back(point_far_msg);
      } else {
        // If current feature is not valid copy NaN
        int id = -1;
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &id,
               sizeof(int)); // id
        for (int j = 1; j < pclMsg_.fields.size(); j++) {
          memcpy(&pclMsg_.data[offset + pclMsg_.fields[j].offset], &badPoint,
                 sizeof(float));
        }
      }
    }
    pubPcl_.publish(pclMsg_);
    pubMarkers_.publish(markerMsg_);
  }
  if (pubPatch_.getNumSubscribers() > 0 || forcePatchPublishing_) {
    patchMsg_.header.seq = msgSeq_;
    patchMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
    int offset = 0;
    for (unsigned int i = 0; i < mtState::nMax_;
         i++, offset += patchMsg_.point_step) {
      if (filterState.fsm_.isValid_[i]) {
        memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset],
               &filterState.fsm_.features_[i].idx_, sizeof(int)); // id
        // Add patch data
        for (int l = 0; l < mtState::nLevels_; l++) {
          for (int y = 0; y < mtState::patchSize_; y++) {
            for (int x = 0; x < mtState::patchSize_; x++) {
              memcpy(&patchMsg_
                          .data[offset + patchMsg_.fields[1].offset +
                                (l * mtState::patchSize_ * mtState::patchSize_ +
                                 y * mtState::patchSize_ + x) *
                                    4],
                     &filterState.fsm_.features_[i]
                          .mpMultilevelPatch_->patches_[l]
                          .patch_[y * mtState::patchSize_ + x],
                     sizeof(float)); // Patch
              memcpy(&patchMsg_
                          .data[offset + patchMsg_.fields[2].offset +
                                (l * mtState::patchSize_ * mtState::patchSize_ +
                                 y * mtState::patchSize_ + x) *
                                    4],
                     &filterState.fsm_.features_[i]
                          .mpMultilevelPatch_->patches_[l]
                          .dx_[y * mtState::patchSize_ + x],
                     sizeof(float)); // dx
              memcpy(&patchMsg_
                          .data[offset + patchMsg_.fields[3].offset +
                                (l * mtState::patchSize_ * mtState::patchSize_ +
                                 y * mtState::patchSize_ + x) *
                                    4],
                     &filterState.fsm_.features_[i]
                          .mpMultilevelPatch_->patches_[l]
                          .dy_[y * mtState::patchSize_ + x],
                     sizeof(float)); // dy
              memcpy(&patchMsg_
                          .data[offset + patchMsg_.fields[4].offset +
                                (l * mtState::patchSize_ * mtState::patchSize_ +
                                 y * mtState::patchSize_ + x) *
                                    4],
                     &filterState.mlpErrorLog_[i]
                          .patches_[l]
                          .patch_[y * mtState::patchSize_ + x],
                     sizeof(float)); // error
            }
          }
        }
      } else {
        // If current feature is not valid copy NaN
        int id = -1;
        memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset], &id,
               sizeof(int)); // id
      }
    }

    pubPatch_.publish(patchMsg_);
  }
  gotFirstMessages_ = true;
}

RovioInterface::makeTest() {
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
