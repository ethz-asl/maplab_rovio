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
    } catch (boost::property_tree::ptree_error &e) {
      std::cout << "Unable to load the filter configuration from "
                << config_file << "! Exception: " << e.what() << std::endl;
    }
  }

#define GETTER_AND_SETTER(name, settings_string, data_type)                    \
  inline data_type get##name(const data_type &default_value) {                 \
    return get(#settings_string, default_value);                               \
  }                                                                            \
  inline bool get##name(data_type *value_ptr) const {                          \
    CHECK_NOTNULL(value_ptr);                                                  \
    try {                                                                      \
      *value_ptr = get<data_type>(#settings_string);                           \
    } catch (...) {                                                            \
      return false;                                                            \
    }                                                                          \
    return true;                                                               \
  }                                                                            \
  inline void set##name(const data_type &value) {                              \
    put(#settings_string, value);                                              \
  }

#define CAMERA_GETTER_AND_SETTER(name, camera_variable_string, data_type)      \
                                                                               \
  inline bool get##name##FromCamera(const int camID, data_type *value_ptr)     \
      const {                                                                  \
    CHECK_NOTNULL(value_ptr);                                                  \
                                                                               \
    const std::string value_key =                                              \
        "Camera" + std::to_string(camID) + "." + #camera_variable_string;      \
    try {                                                                      \
      *value_ptr = get<data_type>(value_key);                                  \
    } catch (...) {                                                            \
      LOG(ERROR) << "Unable to find camera variable at " << value_key;         \
      return false;                                                            \
    }                                                                          \
    return true;                                                               \
  }                                                                            \
                                                                               \
  inline void set##name##ForCamera(const int camID, const data_type &value) {  \
    const std::string value_key =                                              \
        "Camera" + std::to_string(camID) + "." + #camera_variable_string;      \
    put(value_key, value);                                                     \
  }

  GETTER_AND_SETTER(DoFrameVisualization, ImgUpdate.doFrameVisualisation, bool);
  GETTER_AND_SETTER(DoVisualization, PoseUpdate.doVisualization, bool);

  // Camera extrinsics.
  CAMERA_GETTER_AND_SETTER(qCM_x, qCM_x, double);
  CAMERA_GETTER_AND_SETTER(qCM_y, qCM_y, double);
  CAMERA_GETTER_AND_SETTER(qCM_z, qCM_z, double);
  CAMERA_GETTER_AND_SETTER(qCM_w, qCM_w, double);
  CAMERA_GETTER_AND_SETTER(MrMC_x, MrMC_x, double);
  CAMERA_GETTER_AND_SETTER(MrMC_y, MrMC_y, double);
  CAMERA_GETTER_AND_SETTER(MrMC_z, MrMC_z, double);

  // IMU prediction noise.
  // Covariance parameter of attitude prediction [rad^2/s].
  GETTER_AND_SETTER(
      GyroCovarianceX, Prediction.PredictionNoise.att_0, double);
  GETTER_AND_SETTER(
      GyroCovarianceY, Prediction.PredictionNoise.att_1, double);
  GETTER_AND_SETTER(GyroCovarianceZ, Prediction.PredictionNoise.att_2, double);
  // Covariance parameter of gyroscope bias prediction [rad^2/s^3].
  GETTER_AND_SETTER(
      GyroBiasCovarianceX, Prediction.PredictionNoise.gyb_0, double);
  GETTER_AND_SETTER(
      GyroBiasCovarianceY, Prediction.PredictionNoise.gyb_1, double);
  GETTER_AND_SETTER(
      GyroBiasCovarianceZ, Prediction.PredictionNoise.gyb_2, double);
  // Covariance parameter of velocity prediction [m^2/s^3].
  GETTER_AND_SETTER(AccCovarianceX, Prediction.PredictionNoise.vel_0, double);
  GETTER_AND_SETTER(AccCovarianceY, Prediction.PredictionNoise.vel_1, double);
  GETTER_AND_SETTER(AccCovarianceZ, Prediction.PredictionNoise.vel_2, double);
  // Covariance parameter of accelerometer bias prediction [m^2/s^5].
  GETTER_AND_SETTER(
      AccBiasCovarianceX, Prediction.PredictionNoise.acb_0, double);
  GETTER_AND_SETTER(
      AccBiasCovarianceY, Prediction.PredictionNoise.acb_1, double);
  GETTER_AND_SETTER(
      AccBiasCovarianceZ, Prediction.PredictionNoise.acb_2, double);
};

} // namespace rovio

#endif // ROVIO_FILTER_CONFIGURATION_HPP_
