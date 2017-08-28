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

#ifndef ROVIO_FILTER_CONFIGURATION_HPP_
#define ROVIO_FILTER_CONFIGURATION_HPP_

#include <glog/logging.h>
#include <string>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace rovio {
// ImuSigmas struct ported from maplab so that it's available within rovio.
struct ImuSigmas {
  double gyro_noise;
  double gyro_bias;
  double acc_noise;
  double acc_bias;

  inline ImuSigmas(
      const double _gyro_noise, const double _gyro_bias,
      const double _acc_noise, const double _acc_bias)
      : gyro_noise(_gyro_noise),
        gyro_bias(_gyro_bias),
        acc_noise(_acc_noise),
        acc_bias(_acc_bias) {}

  inline ImuSigmas() : ImuSigmas(0., 0., 0., 0.) {}

  inline bool check() const {
    return gyro_noise > 0. && gyro_bias > 0. && acc_noise > 0. && acc_bias > 0.;
  }

  inline bool operator==(const ImuSigmas& other) const {
    return gyro_noise == other.gyro_noise && gyro_bias == other.gyro_bias &&
           acc_noise == other.acc_noise && acc_bias == other.acc_bias;
  }
};

struct FilterConfiguration : public boost::property_tree::ptree {
  typedef boost::property_tree::ptree ptree;

  FilterConfiguration() : boost::property_tree::ptree() {}

  FilterConfiguration(const std::string &config_file)
      : boost::property_tree::ptree() {
    loadFromFile(config_file);
  }

  void loadFromFile(const std::string &config_file) {
    try {
      ptree *propertyTreePtr = this;
      read_info(config_file, *propertyTreePtr);
    } catch (boost::property_tree::ptree_error& e) {
      std::cout << "Unable to load the filter configuration from "
                << config_file << "! Exception: " << e.what() << std::endl;
    }
  }

  void setImuSigmas(const ImuSigmas &imu_sigmas) {
    // TODO(eggerk,schneith): properly set these values.
    static_cast<void>(imu_sigmas);
  }

#define GETTER_AND_SETTER(name, settings_string, data_type)    \
                                                               \
  inline data_type get##name(const data_type& default_value) { \
    return get(#settings_string, default_value);               \
  }                                                            \
                                                               \
  inline bool get##name(data_type* value_ptr) const {          \
    CHECK_NOTNULL(value_ptr);                                  \
    try {                                                      \
      *value_ptr = get<data_type>(#settings_string);           \
    } catch (...) {                                            \
      return false;                                            \
    }                                                          \
    return true;                                               \
  }                                                            \
                                                               \
  inline void set##name(const data_type& value) {              \
    put(#settings_string, value);                              \
  }

#define CAMERA_GETTER_AND_SETTER(name, camera_variable_string, data_type)     \
                                                                              \
  inline bool get##name##FromCamera(const int camID, data_type* value_ptr)    \
      const {                                                                 \
    CHECK_NOTNULL(value_ptr);                                                 \
                                                                              \
    const std::string value_key =                                             \
        "Camera" + std::to_string(camID) + "." + #camera_variable_string;     \
    try {                                                                     \
      *value_ptr = get<data_type>(value_key);                                 \
    } catch (...) {                                                           \
      LOG(ERROR) << "Unable to find camera variable at " << value_key;        \
      return false;                                                           \
    }                                                                         \
    return true;                                                              \
  }                                                                           \
                                                                              \
  inline void set##name##ForCamera(const int camID, const data_type& value) { \
    const std::string value_key =                                             \
        "Camera" + std::to_string(camID) + "." + #camera_variable_string;     \
    put(value_key, value);                                                    \
  }

  GETTER_AND_SETTER(DoVisualization, PoseUpdate.doVisualization, bool);

  GETTER_AND_SETTER(
      PredictionPositionCovarianceX, Prediction.PredictionNoise.pos_0, double);
  GETTER_AND_SETTER(
      PredictionPositionCovarianceY, Prediction.PredictionNoise.pos_1, double);
  GETTER_AND_SETTER(
      PredictionPositionCovarianceZ, Prediction.PredictionNoise.pos_2, double);

  GETTER_AND_SETTER(
      PredictionVelocityCovarianceX, Prediction.PredictionNoise.vel_0, double);
  GETTER_AND_SETTER(
      PredictionVelocityCovarianceY, Prediction.PredictionNoise.vel_1, double);
  GETTER_AND_SETTER(
      PredictionVelocityCovarianceZ, Prediction.PredictionNoise.vel_2, double);

  GETTER_AND_SETTER(
      PredictionAccelerometerCovarianceX, Prediction.PredictionNoise.acb_0,
      double);
  GETTER_AND_SETTER(
      PredictionAccelerometerCovarianceY, Prediction.PredictionNoise.acb_1,
      double);
  GETTER_AND_SETTER(
      PredictionAccelerometerCovarianceZ, Prediction.PredictionNoise.acb_2,
      double);

  GETTER_AND_SETTER(
      PredictionGyroscopeCovarianceX, Prediction.PredictionNoise.gyb_0, double);
  GETTER_AND_SETTER(
      PredictionGyroscopeCovarianceY, Prediction.PredictionNoise.gyb_1, double);
  GETTER_AND_SETTER(
      PredictionGyroscopeCovarianceZ, Prediction.PredictionNoise.gyb_2, double);

  CAMERA_GETTER_AND_SETTER(qCM_x, qCM_x, double);
  CAMERA_GETTER_AND_SETTER(qCM_y, qCM_y, double);
  CAMERA_GETTER_AND_SETTER(qCM_z, qCM_z, double);
  CAMERA_GETTER_AND_SETTER(qCM_w, qCM_w, double);

  CAMERA_GETTER_AND_SETTER(MrMC_x, MrMC_x, double);
  CAMERA_GETTER_AND_SETTER(MrMC_y, MrMC_y, double);
  CAMERA_GETTER_AND_SETTER(MrMC_z, MrMC_z, double);
};

}  // namespace rovio

#endif  // ROVIO_FILTER_CONFIGURATION_HPP_
