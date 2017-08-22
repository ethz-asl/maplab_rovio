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

#ifndef ROVIO_ROVIO_INTERFACE_STATE_IMPL_H_
#define ROVIO_ROVIO_INTERFACE_STATE_IMPL_H_

#include "rovio/RovioFilter.hpp"
#include "rovio/RovioInterfaceStates.hpp"

namespace rovio {

template <typename FILTER> struct RovioPatchStateImpl : public RovioPatchState {
  static constexpr int kMaxNumFeatures = FILTER::mtFilterState::mtState::nMax_;
  static constexpr int kNumPatchLevels =
      FILTER::mtFilterState::mtState::nLevels_;
  static constexpr int kPatchSize = FILTER::mtFilterState::mtState::patchSize_;
  static constexpr int kPatchArea = kPatchSize * kPatchSize;

  virtual ~RovioPatchStateImpl() {}

  bool get_isFeatureValid(const int feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return isFeatureValid[feature_idx];
  }
  int get_PatchIndex(const int feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return patchIndices[feature_idx];
  }
  bool get_isPatchValid(const int feature_idx, const int patch_level) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    DCHECK_LT(patch_level, kNumPatchLevels);
    return isPatchValid[feature_idx][patch_level];
  }

  float get_PatchPixel(const int feature_idx, const int patch_level,
                       const int linear_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    DCHECK_LT(patch_level, kNumPatchLevels);
    DCHECK_LT(linear_idx, kPatchArea);
    return patches[feature_idx][patch_level].patch_[linear_idx];
  }
  float get_PatchDx(const int feature_idx, const int patch_level,
                    const int linear_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    DCHECK_LT(patch_level, kNumPatchLevels);
    DCHECK_LT(linear_idx, kPatchArea);
    return patches[feature_idx][patch_level].dx_[linear_idx];
  }
  float get_PatchDy(const int feature_idx, const int patch_level,
                    const int linear_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    DCHECK_LT(patch_level, kNumPatchLevels);
    DCHECK_LT(linear_idx, kPatchArea);
    return patches[feature_idx][patch_level].dy_[linear_idx];
  }

  bool isFeatureValid[kMaxNumFeatures];
  int patchIndices[kMaxNumFeatures];
  bool isPatchValid[kMaxNumFeatures][kNumPatchLevels];
  Patch<kPatchSize> patches[kMaxNumFeatures][kNumPatchLevels];
};

template <typename FILTER>
struct RovioFeatureStateImpl : public RovioFeatureState {
  virtual ~RovioFeatureStateImpl() {}
  static constexpr int kMaxNumFeatures = FILTER::mtFilterState::mtState::nMax_;

  bool get_isFeatureValid(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return isFeatureValid[feature_idx];
  }

  int get_FeatureObservrCamID(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return featureObserverCamIDs[feature_idx];
  }
  int get_FeatureIndex(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return featureIndices[feature_idx];
  }

  const Eigen::Vector3d &get_CrCPm(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return CrCPm_vec[feature_idx];
  }
  const Eigen::Vector3d &get_CrCPp(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return CrCPp_vec[feature_idx];
  }

  const Eigen::Vector3f &get_bearings(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return bearings[feature_idx];
  }

  const Eigen::Vector3f &get_MrMP(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return MrMP_vec[feature_idx];
  }
  const Eigen::Matrix3f &get_cov_MrMP(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return cov_MrMP_vec[feature_idx];
  }

  float get_Distance(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return distances[feature_idx];
  }
  float get_DistanceCov(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return distances_cov[feature_idx];
  }

  uint32_t get_Status(const size_t feature_idx) const {
    DCHECK_LT(feature_idx, kMaxNumFeatures);
    return status_vec[feature_idx];
  }

  // Since we use a fixed number of features, the (in)valid ones are determined
  // with this array.
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

template <typename FILTER> struct RovioStateImpl : public RovioState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kNumCameras = FILTER::mtFilterState::mtState::nCam_;
  static constexpr int kMaxNumFeatures = FILTER::mtFilterState::mtState::nMax_;
  static constexpr int kPatchSize = FILTER::mtFilterState::mtState::patchSize_;
  static constexpr int kNumPatchLevels =
      FILTER::mtFilterState::mtState::nLevels_;
  static constexpr int kNumPoses = FILTER::mtFilterState::mtState::nPose_;
  static constexpr int kPatchArea = kPatchSize * kPatchSize;
  static constexpr int kPatchAreaTimesLevels = kPatchArea * kNumPatchLevels;

  bool getIsInitialized() const { return isInitialized; }
  double getTimestamp() const { return timeAfterUpdate; }
  const Eigen::MatrixXd &getFilterCovariance() const {
    return filterCovariance;
  }
  const kindr::RotationQuaternionPD &get_qCM(size_t camera_index) const {
    DCHECK_LT(camera_index, kNumCameras);
    return qCM[camera_index];
  }
  const Eigen::Vector3d &get_MrMC(size_t camera_index) const {
    DCHECK_LT(camera_index, kNumCameras);
    return MrMC[camera_index];
  }
  const Eigen::Vector3d &get_WrWB() const { return WrWB; }
  const kindr::RotationQuaternionPD &get_qBW() const { return qBW; }
  const Eigen::Vector3d &get_BvB() const { return BvB; }
  const Eigen::Vector3d &get_BwWB() const { return BwWB; }
  const Eigen::MatrixXd &getImuCovariance() const { return imuCovariance; }
  const Eigen::Vector3d &getGyb() const { return gyb; }
  const Eigen::Vector3d &getAcb() const { return acb; }
  bool getHasInertialPose() const { return hasInertialPose; }
  const Eigen::Vector3d &get_IrIW() const { return IrIW; }
  const kindr::RotationQuaternionPD &get_qWI() const { return qWI; }

  bool hasFeatureState() const { return hasFeatureUpdate; }
  const RovioFeatureState &getFeatureState() const {
    DCHECK(feature_state);
    return *feature_state;
  }

  bool hasPatchState() const { return hasPatchUpdate; }
  const RovioPatchState &getPatchState() const {
    DCHECK(patch_state);
    return *patch_state;
  }

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
  // Transformation between world frame (W) and the IMU frame / body frame (B).
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
  // Transformation between world frame (= global base frame (W)) and inertial
  // frame (= mission/odometry base frame (I)).
  bool hasInertialPose = false;
  Eigen::Vector3d IrIW;
  kindr::RotationQuaternionPD qWI;

  // Optional: Feature state.
  bool hasFeatureUpdate = false;
  std::unique_ptr<RovioFeatureState> feature_state;

  // Optional: Path state.
  bool hasPatchUpdate = false;
  std::unique_ptr<RovioPatchState> patch_state;
};

} // namespace rovio

#endif // ROVIO_ROVIO_INTERFACE_STATE_IMPL_H_
