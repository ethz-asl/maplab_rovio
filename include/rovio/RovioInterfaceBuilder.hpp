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

#ifndef ROVIO_ROVIOINTERFACEBUILDER_HPP_
#define ROVIO_ROVIOINTERFACEBUILDER_HPP_

#include <Eigen/Core>
#include <glog/logging.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-value"
#include "rovio/CameraCalibration.hpp"
#include "rovio/FilterConfiguration.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/RovioFilter.hpp"
#include "rovio/RovioInterface.hpp"
#include "rovio/RovioInterfaceImpl.hpp"
#pragma GCC diagnostic pop

namespace rovio {
template<
  // kNumCameras: number of cameras.
  size_t kNumCameras,
  // kLocalizationMode: 0-off, 1: estimate baseframe, 2: (1) + sensor offset
  int kLocalizationMode,
  // Maximal number of considered features in the filter state.
  size_t kMaxNumFeatures,
  // Total number of pyramid levels considered.
  size_t kPyramidLevels,
  // Edge length of the patches (in pixel). Must be a multiple of 2!
  size_t kFeaturePatchSizePx
  > static RovioInterface* createRovioInterface(
    const FilterConfiguration& filter_config,
    const CameraCalibrationVector& camera_calibrations) {
  CHECK_EQ(camera_calibrations.size(), kNumCameras);
  typedef rovio::RovioFilter<
      rovio::FilterState<kMaxNumFeatures, kPyramidLevels, kFeaturePatchSizePx,
          kNumCameras, kLocalizationMode>> FilterType;
  return new RovioInterfaceImpl<FilterType>(filter_config, camera_calibrations);
}
} // namespace rovio
#endif  // ROVIO_ROVIOINTERFACEBUILDER_HPP_
