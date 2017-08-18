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

namespace rovio {

/** \brief Distortion model of the camera.
 * */
enum class DistortionModel {
  RADTAN,   //!< Radial tangential distortion model.
  EQUIDIST, //!< Equidistant distortion model.
  COUNT     //!< NOT A DISTORTION MODEL!
};

static constexpr size_t NUM_DISTORTION_MODELS =
    static_cast<size_t>(DistortionModel::COUNT);

const std::array<size_t, NUM_DISTORTION_MODELS> NUM_DISTORTION_PARAMS = {
    {/*RADTAN (k1, k2, p1, p2, k3)*/ 5u,
     /*EQUIDIST (k1, k2, k3, k4)*/ 4u}};

struct CameraCalibration {
  DistortionModel distortionModel_;

  Eigen::VectorXd parameters_;
};

} // namespace rovio

#endif // ROVIO_CAMERA_CALIBRATION_HPP_
