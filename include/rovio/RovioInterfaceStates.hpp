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

#include <glog/logging.h>

#include "rovio/RovioFilter.hpp"

namespace rovio {

struct RovioPatchState {
  virtual ~RovioPatchState() = 0;
  virtual bool get_isFeatureValid(const int feature_idx) const = 0;
  virtual int get_PatchIndex(const int feature_idx) const = 0;
  virtual bool get_isPatchValid(const int feature_idx,
                                const int patch_level) const = 0;

  virtual float get_PatchPixel(const int feature_idx, const int patch_level,
                               const int linear_idx) const = 0;
  virtual float get_PatchDx(const int feature_idx, const int patch_level,
                            const int linear_idx) const = 0;
  virtual float get_PatchDy(const int feature_idx, const int patch_level,
                            const int linear_idx) const = 0;
};

struct RovioFeatureState {
  virtual ~RovioFeatureState() = 0;
  virtual bool get_isFeatureValid(const size_t feature_idx) const = 0;

  virtual int get_FeatureObservrCamID(const size_t feature_idx) const = 0;
  virtual int get_FeatureIndex(const size_t feature_idx) const = 0;

  virtual const Eigen::Vector3d &get_CrCPm(const size_t feature_idx) const = 0;
  virtual const Eigen::Vector3d &get_CrCPp(const size_t feature_idx) const = 0;

  virtual const Eigen::Vector3f &
  get_bearings(const size_t feature_idx) const = 0;

  virtual const Eigen::Vector3f &get_MrMP(const size_t feature_idx) const = 0;
  virtual const Eigen::Matrix3f &
  get_cov_MrMP(const size_t feature_idx) const = 0;

  virtual float get_Distance(const size_t feature_idx) const = 0;
  virtual float get_DistanceCov(const size_t feature_idx) const = 0;

  virtual uint32_t get_Status(const size_t feature_idx) const = 0;
};

struct RovioState {
  virtual ~RovioState() = 0;

  virtual bool getIsInitialized() const = 0;

  virtual double getTimestamp() const = 0;

  virtual const Eigen::MatrixXd &getFilterCovariance() const = 0;

  virtual const Eigen::Vector3d &get_MrMC(size_t camera_index) const = 0;
  virtual const kindr::RotationQuaternionPD &
  get_qCM(size_t camera_index) const = 0;

  virtual const Eigen::Vector3d &get_WrWB() const = 0;
  virtual const kindr::RotationQuaternionPD &get_qBW() const = 0;

  virtual const Eigen::Vector3d &get_BvB() const = 0;
  virtual const Eigen::Vector3d &get_BwWB() const = 0;

  virtual const Eigen::MatrixXd &getImuCovariance() const = 0;

  virtual const Eigen::Vector3d &getGyb() const = 0;
  virtual const Eigen::Vector3d &getAcb() const = 0;

  virtual bool getHasInertialPose() const = 0;
  virtual const Eigen::Vector3d &get_IrIW() const = 0;
  virtual const kindr::RotationQuaternionPD &get_qWI() const = 0;

  virtual bool hasFeatureState() const = 0;
  virtual const RovioFeatureState &getFeatureState() const = 0;

  virtual bool hasPatchState() const = 0;
  virtual const RovioPatchState &getPatchState() const = 0;

  // Optional: Feature state.
  bool hasFeatureUpdate = false;
  std::unique_ptr<RovioFeatureState> feature_state;

  // Optional: Path state.
  bool hasPatchUpdate = false;
  std::unique_ptr<RovioPatchState> patch_state;
};

} // namespace rovio

#endif // ROVIO_ROVIO_INTERFACE_STATE_H_
