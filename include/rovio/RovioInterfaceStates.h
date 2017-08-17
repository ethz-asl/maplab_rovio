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

#include <functional>
#include <memory>
#include <mutex>
#include <queue>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/RovioFilter.hpp"
#include "rovio/SrvResetToPose.h"

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

  bool isFeatureValid[kMaxNumFeatures];

  int featureObserverCamIDs[kMaxNumFeatures];
  int featureIndices[kMaxNumFeatures];

  Eigen::Vector3d CrCPm_vec[kMaxNumFeatures];
  Eigen::Vector3d CrCPp_vec[kMaxNumFeatures];

  Eigen::Vector3f bearings[kMaxNumFeatures];

  Eigen::Vector3f MrMP_vec[kMaxNumFeatures];
  Eigen::Matrix3f cov_MrMP_vec[kMaxNumFeatures];

  float distances[kMaxNumFeatures];
  float distances_cov[kMaxNumFeatures];

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

  double timeAfterUpdate;

  // Inertial pose.
  bool hasInertialPose = false;
  Eigen::Vector3d IrIW;
  kindr::RotationQuaternionPD qWI;

  kindr::RotationQuaternionPD qCM[kNumCameras];
  Eigen::Vector3d MrMC[kNumCameras];

  // Camera extrinsics.
  Eigen::Vector3d BrBC[kNumCameras];
  kindr::RotationQuaternionPD qCB[kNumCameras];

  Eigen::MatrixXd filterCovariance;

  // IMU state and convariance.
  StandardOutput imuOutput;
  Eigen::MatrixXd imuOutputCov;

  Eigen::Vector3d gyb;
  Eigen::Vector3d acb;

  typedef typename std::tuple_element<0, typename FILTER::mtUpdates>::type
      mtImgUpdate;
  mtImgUpdate *mpImgUpdate;

  typedef typename std::tuple_element<1, typename FILTER::mtUpdates>::type
      mtPoseUpdate;
  mtPoseUpdate *mpPoseUpdate;

  // Feature state.
  bool hasFeatureUpdate = false;
  std::unique_ptr<RovioFeatureState<FILTER>> feature_state;

  // Path state.
  bool hasPatchUpdate = false;
  std::unique_ptr<RovioPatchState<FILTER>> patch_state;
};

} // namespace rovio

#endif // ROVIO_ROVIO_INTERFACE_STATE_H_
