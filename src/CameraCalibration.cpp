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

#include "rovio/CameraCalibration.hpp"

#include <iostream>

#include <Eigen/Core>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace rovio {

void CameraCalibration::loadFromFile(const std::string& calibration_yaml_file) {
  CHECK(!calibration_yaml_file.empty());

  loadCameraMatrix(calibration_yaml_file);

  YAML::Node config = YAML::LoadFile(calibration_yaml_file);

  std::string distortionModelName;
  distortionModelName = config["distortion_model"].as<std::string>();

  if (distortionModelName == "plumb_bob") {
    loadRadTanDistortion(calibration_yaml_file);
  } else if (distortionModelName == "equidistant") {
    loadRadTanDistortion(calibration_yaml_file);
  } else {
    std::cout << "ERROR: no camera Model detected!" << std::endl;
    return;
  }
  hasIntrinsics_ = true;
}

void CameraCalibration::loadRadTanDistortion(
    const std::string &calibration_yaml_file) {
  CHECK(!calibration_yaml_file.empty());
  YAML::Node config = YAML::LoadFile(calibration_yaml_file);

  distortionModel_ = DistortionModel::RADTAN;
  distortionParams_.resize(getNumDistortionParam());

  distortionParams_[0] = config[DIST_COEFFS][DATA][0].as<double>();
  distortionParams_[1] = config[DIST_COEFFS][DATA][1].as<double>();
  distortionParams_[2] = config[DIST_COEFFS][DATA][2].as<double>();
  distortionParams_[3] = config[DIST_COEFFS][DATA][3].as<double>();
  distortionParams_[4] = config[DIST_COEFFS][DATA][4].as<double>();
}

void CameraCalibration::loadEquidistDistortion(
    const std::string &calibration_yaml_file) {
  CHECK(!calibration_yaml_file.empty());
  YAML::Node config = YAML::LoadFile(calibration_yaml_file);

  distortionModel_ = DistortionModel::EQUIDIST;
  distortionParams_.resize(getNumDistortionParam());

  distortionParams_[0] = config[DIST_COEFFS][DATA][0].as<double>();
  distortionParams_[1] = config[DIST_COEFFS][DATA][1].as<double>();
  distortionParams_[2] = config[DIST_COEFFS][DATA][2].as<double>();
  distortionParams_[3] = config[DIST_COEFFS][DATA][3].as<double>();
}

void CameraCalibration::loadFovDistortion(
    const std::string &calibration_yaml_file) {
  CHECK(!calibration_yaml_file.empty());
  YAML::Node config = YAML::LoadFile(calibration_yaml_file);

  distortionModel_ = DistortionModel::FOV;
  distortionParams_.resize(getNumDistortionParam());

  distortionParams_[0] = config[DIST_COEFFS][DATA][0].as<double>();
}

void CameraCalibration::loadCameraMatrix(
    const std::string &calibration_yaml_file) {
  CHECK(!calibration_yaml_file.empty());
  YAML::Node config = YAML::LoadFile(calibration_yaml_file);

  // Load camera intrinsics.
  K_(0, 0) = config[CAMERA_MATRIX][DATA][0].as<double>();
  K_(0, 1) = config[CAMERA_MATRIX][DATA][1].as<double>();
  K_(0, 2) = config[CAMERA_MATRIX][DATA][2].as<double>();
  K_(1, 0) = config[CAMERA_MATRIX][DATA][3].as<double>();
  K_(1, 1) = config[CAMERA_MATRIX][DATA][4].as<double>();
  K_(1, 2) = config[CAMERA_MATRIX][DATA][5].as<double>();
  K_(2, 0) = config[CAMERA_MATRIX][DATA][6].as<double>();
  K_(2, 1) = config[CAMERA_MATRIX][DATA][7].as<double>();
  K_(2, 2) = config[CAMERA_MATRIX][DATA][8].as<double>();
}

} // namespace rovio
