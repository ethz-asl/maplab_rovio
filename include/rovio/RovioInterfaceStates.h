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

#ifndef ROVIO_ROVIO_INTERFACE_STATE_H_
#define ROVIO_ROVIO_INTERFACE_STATE_H_

#include "rovio/RovioFilter.hpp"

namespace rovio {

template <typename FILTER> struct RovioPatchState {
  static constexpr int kMaxNumFeatures = FILTER::mtFilterState::mtState::nMax_;
  static constexpr int kNumPatchLevels =
      FILTER::mtFilterState::mtState::nLevels_;
  static constexpr int kPatchSize = FILTER::mtFilterState::mtState::patchSize_;

  bool isFeatureValid[kMaxNumFeatures];
  int patchIndices[kMaxNumFeatures];
  bool isPatchValid[kMaxNumFeatures][kNumPatchLevels];
  Patch<kPatchSize> patches[kMaxNumFeatures][kNumPatchLevels];
};

template <typename FILTER> struct RovioFeatureState {

  static constexpr int kMaxNumFeatures = FILTER::mtFilterState::mtState::nMax_;

  // Since we use a fixed number of features, the (in)valid ones are determined with this array.
  bool isFeatureValid[kMaxNumFeatures];

  int featureObserverCamIDs[kMaxNumFeatures];
  int featureIndices[kMaxNumFeatures];

  // Start and end points of the feature uncertainty visualization (a line along
  // the bearing vector centered around the current estimate of the feature).
  Eigen::Vector3d CrCPm_vec[kMaxNumFeatures];
  Eigen::Vector3d CrCPp_vec[kMaxNumFeatures];

  // Feature bearing vector.
  Eigen::Vector3f bearings[kMaxNumFeatures];

  // Feature position and covariance.
  Eigen::Vector3f MrMP_vec[kMaxNumFeatures];
  Eigen::Matrix3f cov_MrMP_vec[kMaxNumFeatures];

  // Feature distance along the bearing vector with covariance.
  float distances[kMaxNumFeatures];
  float distances_cov[kMaxNumFeatures];

  // Feature status.
  uint32_t status_vec[kMaxNumFeatures];
};

template <typename FILTER> struct RovioState {

  static constexpr int kNumCameras = FILTER::mtFilterState::mtState::nCam_;
  static constexpr int kMaxNumFeatures = FILTER::mtFilterState::mtState::nMax_;
  static constexpr int kPatchSize = FILTER::mtFilterState::mtState::patchSize_;
  static constexpr int kNumPatchLevels =
      FILTER::mtFilterState::mtState::nLevels_;
  static constexpr int kNumPoses = FILTER::mtFilterState::mtState::nPose_;
  static constexpr int kPatchArea = kPatchSize * kPatchSize;
  static constexpr int kPatchAreaTimesLevels = kPatchArea * kNumPatchLevels;

  // If the filter isn't initialized, the state variables do not contain any
  // meaningful data.
  bool isInitialized = false;

  // Time stamp of this state.
  double timeAfterUpdate;

  // Overall filter state covariance.
  Eigen::MatrixXd filterCovariance;

  // Camera extrinsics:
  // Transformation between IMU frame (M) and camera frame (C).
  kindr::RotationQuaternionPD qCM[kNumCameras];
  Eigen::Vector3d MrMC[kNumCameras];

  // IMU frame:
  // Transformation between world frame (W) and the IMU frame (B).
  Eigen::Vector3d WrWB;
  kindr::RotationQuaternionPD qBW;
  // Velocities of IMU frame (B).
  Eigen::Vector3d BvB;
  Eigen::Vector3d BwWB;
  // IMU frame covariance
  Eigen::MatrixXd imuCovariance;
  // IMU biases.
  Eigen::Vector3d gyb;
  Eigen::Vector3d acb;

  // TODO(mfehr): We should try to get rid of those...
  // These objects contain configuration variables that are needed to determine
  // if a certain topic should be published or not.
  typedef typename std::tuple_element<0, typename FILTER::mtUpdates>::type
      mtImgUpdate;
  mtImgUpdate *mpImgUpdate;
  typedef typename std::tuple_element<1, typename FILTER::mtUpdates>::type
      mtPoseUpdate;
  mtPoseUpdate *mpPoseUpdate;

  // Optional: Localization transform:
  // Transformation between world frame (= global base frame, W) and inertial frame
  // (= mission/odometry base frame, I).
  bool hasInertialPose = false;
  Eigen::Vector3d IrIW;
  kindr::RotationQuaternionPD qWI;

  // Optional: Feature state.
  bool hasFeatureUpdate = false;
  std::unique_ptr<RovioFeatureState<FILTER>> feature_state;

  // Optional: Path state.
  bool hasPatchUpdate = false;
  std::unique_ptr<RovioPatchState<FILTER>> patch_state;
};

} // namespace rovio

#endif // ROVIO_ROVIO_INTERFACE_STATE_H_
