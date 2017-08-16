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

#ifndef ROVIO_ROVIO_INTERFACE_H_
#define ROVIO_ROVIO_INTERFACE_H_

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

typedef FILTER mtFilter;
typedef typename mtFilter::mtFilterState mtFilterState;
typedef typename mtFilterState::mtState mtState;
typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
typedef typename std::tuple_element<0, typename mtFilter::mtUpdates>::type
    mtImgUpdate;
typedef typename mtImgUpdate::mtMeas mtImgMeas;
typedef typename std::tuple_element<1, typename mtFilter::mtUpdates>::type
    mtPoseUpdate;
typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
typedef typename std::tuple_element<2, typename mtFilter::mtUpdates>::type
    mtVelocityUpdate;
typedef typename mtVelocityUpdate::mtMeas mtVelocityMeas;

struct FilterInitializationState;

class RovioInterface {
public:
  RovioInterface();

  template <size_t nCam> struct FilterUpdateState {
    Eigen::Vector3d IrIW;
    QPD qWI;

    QPD qCM[nCam];
    V3D MrMC[nCam];

    double timeAfterUpdate;

    bool isInitialized;

    double filterUpdateTimeMs;
    double filterUpdateTimeTotalMs;
    size_t numberImagesProcessed;
    size_t numberImagesProcessedTotal;

    MXD covarianceFilter;
    MXD imuOutputCov;

    MXD covariance; // TODO
    MXD covariance; // TODO

    mtOutput imuOutput;

    mtImgUpdate *mpImgUpdate;
    mtPoseUpdate *mpPoseUpdate;
  };

  bool updateFilter(FilterUpdateState *filter_update);

  void getUpdateSettings()

      bool reset();

  bool resetToPose(const V3D &WrWM, const QPD &qMW);

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The orientaetion is initialized using an accel. measurement.
   */
  void requestReset() {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.state_ =
        FilterInitializationState::State::WaitForInitUsingAccel;
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The pose is initialized to the passed pose.
   *  @param WrWM - Position Vector, pointing from the World-Frame to the
   * IMU-Frame, expressed in World-Coordinates.
   *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World
   * Coordinates->IMU Coordinates)
   */
  void requestResetToPose(const V3D &WrWM, const QPD &qMW) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.WrWM_ = WrWM;
    init_state_.qMW_ = qMW;
    init_state_.state_ =
        FilterInitializationState::State::WaitForInitExternalPose;
  }

  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  void makeTest();

private:
  // Rovio filter variables.
  std::shared_ptr<mtFilter> mpFilter_;
  mtPredictionMeas predictionMeas_;
  mtImgMeas imgUpdateMeas_;
  mtPoseMeas poseUpdateMeas_;
  mtVelocityMeas velocityUpdateMeas_;
  FilterInitializationState init_state_;
  mtImgUpdate *mpImgUpdate_;
  mtPoseUpdate *mpPoseUpdate_;

  // Rovio outputs and coordinate transformations
  typedef StandardOutput mtOutput;
  mtOutput cameraOutput_;
  MXD cameraOutputCov_;
  CameraOutputCT<mtState> cameraOutputCT_;
  ImuOutputCT<mtState> imuOutputCT_;
  rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  rovio::LandmarkOutputImuCT<mtState> landmarkOutputImuCT_;
  rovio::FeatureOutput featureOutput_;
  rovio::LandmarkOutput landmarkOutput_;
  MXD featureOutputCov_;
  MXD landmarkOutputCov_;
  rovio::FeatureOutputReadableCT featureOutputReadableCT_;
  rovio::FeatureOutputReadable featureOutputReadable_;
  MXD featureOutputReadableCov_;

  std::mutex m_filter_;
};

struct FilterInitializationState {
  FilterInitializationState()
      : WrWM_(V3D::Zero()), state_(State::WaitForInitUsingAccel) {}

  enum class State {
    // Initialize the filter using accelerometer measurement on the next
    // opportunity.
    WaitForInitUsingAccel,
    // Initialize the filter using an external pose on the next opportunity.
    WaitForInitExternalPose,
    // The filter is initialized.
    Initialized
  } state_;

  // Buffer to hold the initial pose that should be set during initialization
  // with the state WaitForInitExternalPose.
  V3D WrWM_;
  QPD qMW_;

  explicit operator bool() const { return isInitialized(); }

  bool isInitialized() const { return (state_ == State::Initialized); }
};

} // namespace rovio

#endif // ROVIO_ROVIO_INTERFACE_H_
