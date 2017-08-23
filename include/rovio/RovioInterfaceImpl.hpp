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

#ifndef ROVIO_ROVIO_INTERFACE_IMPL_H_
#define ROVIO_ROVIO_INTERFACE_IMPL_H_

#include <functional>
#include <memory>
#include <mutex>
#include <queue>

#include <glog/logging.h>

#include "rovio/CameraCalibration.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/FilterConfiguration.hpp"
#include "rovio/Memory.hpp"
#include "rovio/RovioFilter.hpp"
#include "RovioInterfaceStatesImpl.hpp"

#include "rovio/RovioInterface.hpp"

namespace rovio {

struct FilterInitializationState;

template <typename FILTER>
class RovioInterfaceImpl : public RovioInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RovioInterfaceImpl(typename std::shared_ptr<FILTER> mpFilter);

  explicit RovioInterfaceImpl(const std::string &filter_config_file);
  RovioInterfaceImpl(
      const std::string &filter_config_file,
      const std::vector<std::string>& camera_calibration_files);

  explicit RovioInterfaceImpl(const FilterConfiguration &filter_config);
  RovioInterfaceImpl(
      const FilterConfiguration &filter_config,
      const CameraCalibrationVector& camera_calibrations);

  virtual ~RovioInterfaceImpl() {}

  typedef FILTER mtFilter;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;

  /** \brief Outputting the feature and patch update state involves allocating
  *          some large arrays and could result in some overhead. Therefore the
  *          state will not be retrieved by default.
  */
  bool getState(const bool get_feature_update, const bool get_patch_update,
                RovioState* filter_update);
  bool getState(RovioState* filter_update);

  /** \brief Returns the time step of the last/latest safe filter state..
  */
  double getLastSafeTime();

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The orientaetion is initialized using an accel. measurement.
   */
  void requestReset();

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The pose is initialized to the passed pose.
   *  @param WrWM - Position Vector, pointing from the World-Frame to the
   * IMU-Frame, expressed in World-Coordinates.
   *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World
   * Coordinates->IMU Coordinates)
   */
  void requestResetToPose(const V3D &WrWM, const QPD &qMW);

  void resetToLastSafePose();

  bool processVelocityUpdate(const Eigen::Vector3d &AvM, const double time_s);

  bool processImuUpdate(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr,
                        const double time_s, bool update_filter);

  bool processImageUpdate(const int camID, const cv::Mat &cv_img,
                          const double time_s);

  bool processGroundTruthUpdate(const Eigen::Vector3d &JrJV, const QPD &qJV,
                                const double time_s);

  bool processGroundTruthOdometryUpdate(
      const Eigen::Vector3d &JrJV, const QPD &qJV,
      const Eigen::Matrix<double, 6, 6> &measuredCov, const double time_s);

  /** \brief Register multiple callbacks that are invoked once the filter
   *         concludes a successful update.
   */
  typedef std::function<void(const RovioState&)> RovioStateCallback;
  void registerStateUpdateCallback(RovioStateCallback callback);

  /** \brief Enable and disable feature and patch update output. If disabled,
   *         the RovioStateImpl<FILTER> returned by the callback does not contain
   * any state information of the features/patches.
   */
  void setEnableFeatureUpdateOutput(const bool enable_feature_update);
  void setEnablePatchUpdateOutput(const bool get_patch_update);

  bool isInitialized() const;

  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  void makeTest();

private:
  /** \brief Trigger a filter update. Will return true if an update happened.
  */
  bool updateFilter();

  void notifyAllStateUpdateCallbacks(const RovioState& state) const;

  /** \brief Print update to std::cout and visualize images using opencv. The
   *  visualization is configured and enabled/disabled based on mpImgUpdate.
  */
  void visualizeUpdate();

  std::vector<RovioStateCallback> filter_update_state_callbacks_;

  typedef StandardOutput mtOutput;

  // TODO(mfehr): Ownership of the filter does not need to be shared, consider
  // changing to unique ptr or raw ptr if someone outside the interface can own
  // the filter.
  std::shared_ptr<mtFilter> mpFilter_;

  typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;

  typedef typename std::tuple_element<0, typename mtFilter::mtUpdates>::type
      mtImgUpdate;
  mtImgUpdate *mpImgUpdate_;

  typedef typename std::tuple_element<1, typename mtFilter::mtUpdates>::type
      mtPoseUpdate;
  mtPoseUpdate *mpPoseUpdate_;

  typedef typename mtImgUpdate::mtMeas mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
  mtPoseMeas poseUpdateMeas_;

  typedef typename std::tuple_element<2, typename mtFilter::mtUpdates>::type
      mtVelocityUpdate;
  typedef typename mtVelocityUpdate::mtMeas mtVelocityMeas;
  mtVelocityMeas velocityUpdateMeas_;

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

  FilterInitializationState init_state_;

  // Rovio outputs and coordinate transformations
  Eigen::MatrixXd cameraOutputCov_;

  CameraOutputCT<mtState> cameraOutputCT_;
  ImuOutputCT<mtState> imuOutputCT_;
  rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  rovio::LandmarkOutputImuCT<mtState> landmarkOutputImuCT_;

  // Features.
  rovio::FeatureOutput featureOutput_;
  rovio::FeatureOutputReadable featureOutputReadable_;
  rovio::FeatureOutputReadableCT featureOutputReadableCT_;
  Eigen::MatrixXd featureOutputCov_;
  Eigen::MatrixXd featureOutputReadableCov_;

  // Landmarks.
  rovio::LandmarkOutput landmarkOutput_;
  Eigen::MatrixXd landmarkOutputCov_;

  // State output config.
  bool enable_feature_update_output_;
  bool enable_patch_update_output_;

  mutable std::recursive_mutex m_filter_;
};

} // namespace rovio

#endif // ROVIO_ROVIO_INTERFACE_IMPL_H_

#include "RovioInterfaceImplInl.hpp"
