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
                                                                               \
  inline data_type get##name(const data_type &default_value) {                 \
    return get(#settings_string, default_value);                               \
  }                                                                            \
                                                                               \
  inline bool get##name(data_type *value_ptr) {                                \
    CHECK_NOTNULL(value_ptr);                                                  \
    try {                                                                      \
      *value_ptr = get<data_type>(#settings_string);                           \
    } catch (...) {                                                            \
      return false;                                                            \
    }                                                                          \
    return true;                                                               \
  }                                                                            \
                                                                               \
  inline void set##name(const data_type &value) {                              \
    put(#settings_string, value);                                              \
  }

  GETTER_AND_SETTER(DoVisualization, PoseUpdate.doVisualization, bool);
};

} // namespace rovio

#endif // ROVIO_FILTER_CONFIGURATION_HPP_
