/*
* Copyright (c) 2017, Autonomous Systems Lab
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

#ifndef ROVIO_ROVIOINTERFACE_HPP_
#define ROVIO_ROVIOINTERFACE_HPP_

#include <Eigen/Core>
#include <rovio/RovioInterfaceStatesImpl.hpp>

namespace rovio {

class RovioInterface {
 public:
  virtual ~RovioInterface() = 0;

  virtual bool getState(const bool get_feature_update, const bool get_patch_update,
                RovioState *filter_update) = 0;
  virtual bool getState(RovioState *filter_update) = 0;

  /** \brief Register multiple callbacks that are invoked once the filter
   *         concludes a successful update.
   */
  typedef std::function<void(const RovioState&)> RovioStateCallback;
  virtual void registerStateUpdateCallback(RovioStateCallback callback) = 0;

  /** \brief Returns the time step of the last/latest safe filter state..
  */
  virtual double getLastSafeTime() = 0;

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The orientaetion is initialized using an accel. measurement.
   */
  virtual void requestReset() = 0;

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The pose is initialized to the passed pose.
   *  @param WrWM - Position Vector, pointing from the World-Frame to the
   * IMU-Frame, expressed in World-Coordinates.
   *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World
   * Coordinates->IMU Coordinates)
   */
  virtual void requestResetToPose(const V3D &WrWM, const QPD &qMW) = 0;

  virtual void resetToLastSafePose() = 0;

  virtual bool processVelocityUpdate(const Eigen::Vector3d &AvM, const double time_s) = 0;

  // If update_filter is false; the measurement will just be queued but the
  // actual prediction is only applied before the next update is processed.
  virtual bool processImuUpdate(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr,
                                const double time_s, bool update_filter) = 0;

  virtual bool processImageUpdate(const int camID, const cv::Mat &cv_img,
                                  const double time_s) = 0;

  virtual bool processGroundTruthUpdate(const Eigen::Vector3d &JrJV, const QPD &qJV,
                                        const double time_s) = 0;

  virtual bool processGroundTruthOdometryUpdate(
      const Eigen::Vector3d &JrJV, const QPD &qJV,
      const Eigen::Matrix<double, 6, 6> &measuredCov, const double time_s) = 0;

  /** \brief Enable and disable feature and patch update output. If disabled,
   *         the RovioState<FILTER> returned by the callback does not contain
   * any state information of the features/patches.
   */
  virtual void setEnableFeatureUpdateOutput(const bool enable_feature_update) = 0;
  virtual void setEnablePatchUpdateOutput(const bool get_patch_update) = 0;

  virtual bool isInitialized() const = 0;
  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  virtual void makeTest() = 0;
};

} // namespace rovio

#endif  // ROVIO_ROVIOINTERFACE_HPP_
