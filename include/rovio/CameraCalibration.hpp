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

#ifndef ROVIO_CAMERA_CALIBRATION_HPP_
#define ROVIO_CAMERA_CALIBRATION_HPP_

#include <Eigen/Core>
#include <kindr/Core>
#include <yaml-cpp/yaml.h>

#include "rovio/FilterConfiguration.hpp"
#include "rovio/Memory.hpp"

namespace rovio {

/** \brief Distortion model of the camera.
 * */
enum class DistortionModel {
  RADTAN,   //!< Radial tangential distortion model.
  EQUIDIST, //!< Equidistant distortion model.
  FOV,      //!< Field of view model.
  NUM       //!< NOT A DISTORTION MODEL!
};

static constexpr size_t NUM_DISTORTION_MODELS =
    static_cast<size_t>(DistortionModel::NUM);

const std::array<size_t, NUM_DISTORTION_MODELS> NUM_DISTORTION_MODEL_PARAMS = {
    {/*RADTAN (k1, k2, p1, p2, k3)*/ 5u,
     /*EQUIDIST (k1, k2, k3, k4)*/ 4u,
     /*FOV (w)*/ 1u}};

const std::array<std::string, NUM_DISTORTION_MODELS> DISTORTION_MODEL_NAME = {
    {/*RADTAN*/ "plumb_bob",
     /*EQUIDIST*/ "equidistant"
     /*FOV*/ "fov"}};

// YAML keywords.
static const std::string CAMERA_MATRIX = "camera_matrix";
static const std::string DIST_COEFFS = "distortion_coefficients";
static const std::string DATA = "data";

struct CameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraCalibration() {}

  CameraCalibration(const std::string &calibration_yaml_file) {
    loadFromFile(calibration_yaml_file);
  }

  /** \brief Does this struct contain intrinsics and distortion parameter?
   * */
  bool hasIntrinsics_ = false;

  /** \brief Camera intrinsics.
   * */
  Eigen::Matrix3d K_;

  /** \brief Camera distortion model.
   * */
  DistortionModel distortionModel_;

  /** \brief Variable size distortion param vector.
   * */
  Eigen::VectorXd distortionParams_;

  /** \brief Optional camera extrinsics. This information might also be
   * contained in the rovio filter settings, however if they are set here, it
   * will overwrite the existing settings.
   * */
  bool hasExtrinsics_ = false;
  Eigen::Vector3d MrMC_;
  kindr::RotationQuaternionPD qCM_;

  /** \brief Returns the size of the distortion_parameter vector.
   * */
  size_t getNumDistortionParam() {
    return NUM_DISTORTION_MODEL_PARAMS[static_cast<int>(distortionModel_)];
  }

  /** \brief Sets the camera extrinsics.
   */
  void setCameraExtrinsics(const Eigen::Vector3d &MrMC,
                           const kindr::RotationQuaternionPD &qCM) {
    hasExtrinsics_ = true;
    MrMC_ = MrMC;
    qCM_ = qCM;
  }

  /** \brief Sets the camera extrinsics based on the filter settings.
   */
  void getCameraExtrinsicsFromFilterConfiguration(
      const int camID, const FilterConfiguration &filter_configuration) {
    CHECK(filter_configuration.getqCM_xFromCamera(camID, &(qCM_.x())));
    CHECK(filter_configuration.getqCM_yFromCamera(camID, &(qCM_.y())));
    CHECK(filter_configuration.getqCM_zFromCamera(camID, &(qCM_.z())));
    CHECK(filter_configuration.getqCM_wFromCamera(camID, &(qCM_.w())));

    CHECK(filter_configuration.getMrMC_xFromCamera(camID, &(MrMC_.x())));
    CHECK(filter_configuration.getMrMC_yFromCamera(camID, &(MrMC_.y())));
    CHECK(filter_configuration.getMrMC_zFromCamera(camID, &(MrMC_.z())));

    hasExtrinsics_ = true;
  }

  /** \brief Loads and sets intrinsics, the distortion model and the
   * corresponding distortion coefficients from yaml-file.
   *
   *   @param filename - Path to the yaml-file, containing the distortion model
   * and distortion coefficient data.
   */
  void loadFromFile(const std::string &calibration_yaml_file);

  /** \brief Loads and sets the distortion parameters {k1_, k2_, k3_, p1_, p2_}
   * for the Radtan distortion model from yaml-file.
   *   @param filename - Path to the yaml-file, containing the distortion
   * coefficients.
   */
  void loadRadTanDistortion(const std::string &calibration_yaml_file);

  /** \brief Loads and sets the distortion parameters {k1_, k2_, k3_, k4_} for
   * the Equidistant distortion model from yaml-file.
   *   @param filename - Path to the yaml-file, containing the distortion
   * coefficients.
   */
  void loadEquidistDistortion(const std::string &calibration_yaml_file);

  /** \brief Loads and sets the distortion parameter {w} for
   * the Fov distortion model from yaml-file.
   *   @param filename - Path to the yaml-file, containing the distortion
   * coefficients.
   */
  void loadFovDistortion(const std::string &calibration_yaml_file);

  /** \brief Loads and sets the intrinsic parameter matrix K_ from yaml-file.
   *
   *   @param filename - Path to the yaml-file, containing the intrinsic
   * parameter matrix coefficients.
   */
  void loadCameraMatrix(const std::string &calibration_yaml_file);
};

typedef Aligned<std::vector, CameraCalibration> CameraCalibrationVector;

} // namespace rovio

#endif // ROVIO_CAMERA_CALIBRATION_HPP_
