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
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_LOCALIZATION_LANDMARK_UPDATE_HPP_
#define ROVIO_LOCALIZATION_LANDMARK_UPDATE_HPP_

#include <Eigen/Core>
#include <glog/logging.h>
#include <lightweight_filtering/common.hpp>
#include <lightweight_filtering/Update.hpp>
#include <lightweight_filtering/State.hpp>

#include "rovio/MultiCamera.hpp"

namespace rovio {

class LocalizationLandmarkInnovation : public LWF::State<LWF::VectorElement<2>> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::State<LWF::VectorElement<2>> Base;
  using Base::E_;
  static constexpr unsigned int _pix = 0;

  LocalizationLandmarkInnovation() {
    static_assert(_pix+1==E_,"Error with indices");
    this->template getName<_pix>() = "pix";
  }
  virtual ~LocalizationLandmarkInnovation() {}
  inline Eigen::Vector2d& pix() {
    return this->template get<_pix>();
  }
};

class LocalizationLandmarkMeasurement :
    public LWF::State<LWF::VectorElement<2>> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr unsigned int _pix = 0;

  LocalizationLandmarkMeasurement() :
    G_landmark_(Eigen::Vector3d::Zero()),
    camera_index_(0u) {
    static_assert(_pix+1==E_,"Error with indices");
  }
  virtual ~LocalizationLandmarkMeasurement() {}

  inline Eigen::Vector2d& keypoint(){
    return this->template get<_pix>();
  }
  inline const Eigen::Vector2d& keypoint() const {
    return this->template get<_pix>();
  }
  inline V3D& G_landmark(){
    return G_landmark_;
  }
  inline const V3D& G_landmark() const {
    return G_landmark_;
  }
  inline size_t& camera_index() {
    return camera_index_;
  }
  inline size_t camera_index() const {
    return camera_index_;
  }
 private:
  Eigen::Vector3d G_landmark_;
  size_t camera_index_;
};

class LocalizationLandmarkNoise : public LWF::State<LWF::VectorElement<2>> {
 public:
  static constexpr unsigned int _pix = 0;
  LocalizationLandmarkNoise() {
    static_assert(_pix+1==E_,"Error with indices");
    this->template getName<_pix>() = "pix";
  }
  inline const Eigen::Vector2d& pix() const {
    return this->template get<_pix>();
  }
  virtual ~LocalizationLandmarkNoise() {}
};

class LocalizationLandmarkOutlierDetection : public LWF::OutlierDetection<
    LWF::ODEntry<
        LocalizationLandmarkInnovation::template getId<
            LocalizationLandmarkInnovation::_pix>(), 2>> {
 public:
  virtual ~LocalizationLandmarkOutlierDetection(){};
};

/**
 * G: Inertial frame of localization map
 * W: Odometry frame of ROVIO
 * M: IMU-coordinate frame
 * C: Camera frame.
 */
template<typename FILTERSTATE>
using LocalizationLandmarkUpdateBase =
    LWF::Update<LocalizationLandmarkInnovation, FILTERSTATE,
    LocalizationLandmarkMeasurement, LocalizationLandmarkNoise,
    LocalizationLandmarkOutlierDetection, false>;

template<typename FILTERSTATE>
class LocalizationLandmarkUpdate :
    public LocalizationLandmarkUpdateBase<FILTERSTATE> {
 public:
  typedef LocalizationLandmarkUpdateBase<FILTERSTATE> Base;
  using Base::eval;
  using Base::meas_;
  // This is the update covariance as used by the Kalman functions.
  using Base::updnoiP_;

  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  typedef typename Base::mtInputTuple mtInputTuple;
  typedef typename Base::mtModelBase mtModelBase;

  LocalizationLandmarkUpdate()
    : localization_pixel_sigma_(1.0),
      filter_state_memory_(LWF::FilteringMode::ModeEKF),
      force_ekf_updates_(false),
      multi_cameras_(nullptr) {
    double localization_pixel_sigma;
    Base::doubleRegister_.registerScalar(
        "localization_pixel_sigma", localization_pixel_sigma_);

    // Remove some properties that are inherited from the Update base.
    Base::doubleRegister_.removeScalarByStr("alpha");
    Base::doubleRegister_.removeScalarByStr("beta");
    Base::doubleRegister_.removeScalarByStr("kappa");
    Base::doubleRegister_.removeScalarByStr("UpdateNoise.pix_0");
    Base::doubleRegister_.removeScalarByStr("UpdateNoise.pix_1");

    Base::boolRegister_.registerScalar("forceEKFupdate", force_ekf_updates_);
  }
  virtual ~LocalizationLandmarkUpdate() {}

  void refreshProperties() override {
    Base::updnoiP_.setZero();
    Base::updnoiP_.diagonal().setConstant(
        localization_pixel_sigma_ * localization_pixel_sigma_);
  }

  void setCamera(MultiCamera<mtState::nCam_>* multi_cameras){
    CHECK_NOTNULL(multi_cameras);
    multi_cameras_ = multi_cameras;
  }

  void setMeasurement(const LocalizationLandmarkMeasurement& measurement) {
    measurement_ = measurement;
  }

  bool evaluateModel(const mtState& state, mtInnovation* innovation,
                     MXD* jacobian) const {
    // G: Inertial frame of localization map
    // W: Odometry frame of ROVIO
    // M: IMU-coordinate frame
    // C: Camera frame.
    //
    // Residual: r = f_p(T_CM * T_MW * T_WG * G_l)
    const size_t camera_index = measurement_.camera_index();
    const QPD& qCM = state.qCM(camera_index);
    const V3D& MrMC = state.MrMC(camera_index);

    const Eigen::Vector3d W_l =
        state.qWG().rotate(measurement_.G_landmark()) + state.WrWG();
    const Eigen::Vector3d M_l = state.qWM().inverseRotate(V3D(W_l - state.WrWM()));
    const Eigen::Vector3d C_l = qCM.rotate(V3D(M_l - MrMC));

    cv::Point2f predicted_keypoint_cv;
    CHECK_LT(camera_index, mtState::nCam_);
    Eigen::Matrix<double, 2, 3> d_r__d_C_l;
    const bool projection_success =
        CHECK_NOTNULL(multi_cameras_)->cameras_[camera_index].
          bearingToPixel(C_l, predicted_keypoint_cv, d_r__d_C_l);

    if (!projection_success) {
      LOG(WARNING) << "Projection failed.";
      return false;
    }

    if (innovation != nullptr) {
      Eigen::Vector2d predicted_keypoint(
              predicted_keypoint_cv.x, predicted_keypoint_cv.y);
      innovation->pix() = (predicted_keypoint - measurement_.keypoint());
    }

    if (jacobian != nullptr) {
      jacobian->setZero();

      // d_r__d_T_WG
      const size_t index_WrWG = mtState::template getId<mtState::_pmp>();
      jacobian->block<2,3>(0, index_WrWG) =
          d_r__d_C_l * MPD(qCM * state.qWM().inverted()).matrix();

      const size_t index_qWG = mtState::template getId<mtState::_pma>();
      jacobian->block<2,3>(0, index_qWG) =
          -d_r__d_C_l *
          MPD(qCM * state.qWM().inverted()).matrix() *
          gSM(state.qWG().rotate(measurement_.G_landmark()));

      // d_r__d_T_WM
      const size_t index_WrWM = mtState::template getId<mtState::_pos>();
      jacobian->block<2,3>(0, index_WrWM) =
         -d_r__d_C_l * MPD(qCM * state.qWM().inverted()).matrix();

      const size_t index_qWM = mtState::template getId<mtState::_att>();
      jacobian->block<2,3>(0, index_qWM) =
          d_r__d_C_l * MPD(qCM * state.qWM().inverted()).matrix() *
          gSM(W_l - state.WrWM());

      // d_r__d_T_MC
      const size_t index_MrMC = mtState::template getId<mtState::_vep>() +
          3u * camera_index;
      jacobian->block<2,3>(0, index_MrMC) = -d_r__d_C_l * MPD(qCM).matrix();

      const size_t index_qCM = mtState::template getId<mtState::_vea>() +
           3u * camera_index;
      jacobian->block<2,3>(0, index_qCM) =
          -d_r__d_C_l * gSM(qCM.rotate(V3D(M_l - MrMC)));
    }
    return true;
  }

  bool evalInnovation(mtInnovation& y, const mtState& state,
                      const mtNoise& noise) const {
    bool success = evaluateModel(state,  &y, /*jacobian=*/nullptr);
    y.pix() += noise.pix();
    return success;
  }

  void jacState(MXD& F, const mtState& state) const {
    CHECK_EQ(F.rows(), mtInnovation::D_);
    CHECK_EQ(F.cols(), mtState::D_);
    evaluateModel(state, /*innovation=*/nullptr, &F);
  }

  void jacStateFD(MXD& F, const mtState& state, double epsilon = 1e-4,
                  double dt = 0.0) const {
    CHECK_EQ(F.rows(), mtInnovation::D_);
    CHECK_EQ(F.cols(), mtState::D_);
    mtInputTuple input_tuples;
    std::get<0>(input_tuples) = state;
    this->template jacInputFD<0, 0, mtState::D_>(F, input_tuples, dt, epsilon);
  }

  void jacNoise(MXD& G, const mtState& state) const {
    G.setIdentity();
  }

  void preProcess(mtFilterState& filterstate, const mtMeas& meas,
                  bool& isFinished) {
    isFinished = false;

    // Buffer the current measurement.
    measurement_ = meas;

    // Synchronize the camera extrinsics.
    filterstate.state_.updateMultiCameraExtrinsics(multi_cameras_);

    // We perform the update in EKF mode, regardless of the external settings.
    if (force_ekf_updates_) {
      filter_state_memory_ = filterstate.mode_ ;
      filterstate.mode_ = LWF::ModeEKF;
    }
  }

  void postProcess(mtFilterState& filterstate, const mtMeas& meas,
      const mtOutlierDetection& outlierDetection,
      bool& isFinished) {
    // Restore the previous update settings.
    if (force_ekf_updates_) {
      filterstate.mode_ = filter_state_memory_;
    }

    // Synchronize the camera extrinsics.
    filterstate.state_.updateMultiCameraExtrinsics(multi_cameras_);

    // Visualize the keypoint localization and the localization landmark
    // reprojection.
    // TODO(schneith): Disable drawing if visualization is disabled.
    {
      cv::Mat image = filterstate.img_[measurement_.camera_index()];
      cv::circle(image, cv::Point(measurement_.keypoint()(0,0),
                                  measurement_.keypoint()(1,0)),
                 /*radius=*/6, /*color=*/cv::Scalar(0,0,255), /*thickness=*/6,
                 /*line_type=*/cv::LINE_AA, /*shift=*/0);

      mtInnovation innovation;
      evaluateModel(filterstate.state_, &innovation, /*jacobian=*/nullptr);
      const Eigen::Vector2d reprojected_landmark =
          innovation.pix() + measurement_.keypoint();
      const cv::Point reprojected_landmark_cv(
          reprojected_landmark(0,0), reprojected_landmark(1,0));
      cv::circle(image, reprojected_landmark_cv, /*radius=*/3,
                 /*color=*/cv::Scalar(0,255,0), /*thickness=*/5,
                 /*line_type=*/cv::LINE_AA, /*shift=*/0);
    }

    // Do not perform additional update loops.
    isFinished = true;
  }

 private:
  double localization_pixel_sigma_;
  LocalizationLandmarkMeasurement measurement_;

  LWF::FilteringMode filter_state_memory_;
  bool force_ekf_updates_;

  // Pointer to the camera models.
  MultiCamera<mtState::nCam_>* multi_cameras_;
};
}

#endif /* ROVIO_LOCALIZATION_LANDMARK_UPDATE_HPP_ */
