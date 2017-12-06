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
#include <Eigen/Dense>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "rovio/FilterStates.hpp"
#include "rovio/LocalizationLandmarkUpdate.hpp"
#include "rovio/MultiCamera.hpp"

typedef rovio::FilterState</*nMax=*/1,/*nLevels=*/2,/*patchSize=*/20,
    /*nCam=*/2,/*nPose=*/1,/*enableMapLocalization=*/true> FilterState;
typedef rovio::LocalizationLandmarkUpdate<FilterState>
LocalizationLandmarkUpdate;
typedef typename FilterState::mtState State;
typedef typename FilterState::mtState Noise;
typedef typename LocalizationLandmarkUpdate::mtMeas Measurement;
typedef typename LocalizationLandmarkUpdate::mtInnovation Innovation;

TEST(LocalizationLandmarkUpdate, Jacobian) {
  rovio::MultiCamera<State::nCam_> multi_cameras;
  for (int cam_idx = 0u; cam_idx < State::nCam_; ++cam_idx) {
    multi_cameras.setExtrinsics(cam_idx, V3D::Random(), QPD().setRandom());
  }

  LocalizationLandmarkUpdate update;
  update.setCamera(&multi_cameras);

  unsigned int seed = 1;
  State state;
  state.setRandom(seed);
  state.updateMultiCameraExtrinsics(&multi_cameras);

  // Test the update function for each camera separately.
  for (int cam_idx = 0u; cam_idx < State::nCam_; ++cam_idx) {
    // Generate a measurement that is in front of the camera.
    const cv::Point2f keypoint_distorted(1.0, 2.0);
    Eigen::Vector3d C_l;
    CHECK(multi_cameras.cameras_[cam_idx].pixelToBearing(
        keypoint_distorted, C_l));
    C_l *= 10.0;

    const Eigen::Vector3d C_landmark(1.0,2.0,1.0);
    const Eigen::Vector3d M_landmark =
        state.qCM(cam_idx).inverseRotate(C_landmark) + state.MrMC(cam_idx);
    const Eigen::Vector3d W_landmark =
        state.qWM().rotate(M_landmark) + state.WrWM();
    const Eigen::Vector3d G_landmark =
        state.qWG().inverseRotate(V3D(W_landmark - state.WrWG()));

    // The keypoint measurement is disturbed to test the computation of the
    // innovation.
    constexpr double kKeypointDisturbance = 2.5;

    Measurement measurement;
    measurement.G_landmark() = G_landmark;
    measurement.keypoint() << keypoint_distorted.x, keypoint_distorted.y;
    measurement.keypoint() += Eigen::Vector2d::Constant(kKeypointDisturbance);
    measurement.camera_index() = cam_idx;
    update.setMeasurement(measurement);

    // The innovation evaluation should yield the disturbance.
    Innovation y;
    update.evalInnovationShort(y, state);
    EXPECT_TRUE(y.pix().isApproxToConstant(-kKeypointDisturbance));

    // Calculate the Jacobian analytically and using finite differences.
    Eigen::MatrixXd J(Innovation::D_, State::D_);
    update.jacState(J, state);

    double epsilon = 1e-4;
    Eigen::MatrixXd J_FD(Innovation::D_, State::D_);
    update.jacStateFD(J_FD, state, epsilon);

    // Compare the two Jacobians.
    constexpr double kThreshold = 0.1;

    std::vector<std::pair<std::string, size_t>> blockname_index_list{
      {"index_WrWG", State::template getId<State::_pmp>()},
      {"index_qWG", State::template getId<State::_pma>()},
      {"index_WrWM", State::template getId<State::_pos>()},
      {"index_qWM", State::template getId<State::_att>()},
    };

    for (int cam_idx = 0u; cam_idx < State::nCam_; ++cam_idx) {
      const size_t index_MrMC = State::template getId<State::_vep>() +
          3u * cam_idx;
      const size_t index_qCM = State::template getId<State::_vea>() +
           3u * cam_idx;
      std::string block_name_MrMC = "index_MrMC_" + std::to_string(cam_idx);
      std::string block_name_q_CM = "index_qCM_" + std::to_string(cam_idx);
      blockname_index_list.emplace_back(block_name_MrMC, index_MrMC);
      blockname_index_list.emplace_back(block_name_q_CM, index_qCM);
    }

    for (const auto& blockname_index : blockname_index_list) {
      typedef Eigen::Ref<Eigen::Matrix<double, 2, 3>> JacobianBlockRef;
      const std::string& block_name = blockname_index.first;
      const size_t start_index = blockname_index.second;

      CHECK_LT(start_index, J.cols());
      JacobianBlockRef Jblock = J.block<2,3>(0, start_index);
      JacobianBlockRef Jblock_FD = J_FD.block<2,3>(0, start_index);

      const bool is_same_approx = Jblock.isApprox(Jblock_FD, kThreshold);
      if (!is_same_approx) {
        LOG(ERROR) << "JACOBIAN MISMATCH:";
        LOG(INFO) << block_name << " (analytical):\n" << Jblock;
        LOG(INFO) << block_name << " (finite-diff):\n" << Jblock_FD << std::endl;
      }
      EXPECT_TRUE(is_same_approx);
    }

    // Set all block that are expected to be nonzero to zero and make sure the
    // nothing remains non-zero.
    for (const auto& blockname_index : blockname_index_list) {
      J.block<2,3>(0, blockname_index.second).setZero();
    }
    EXPECT_TRUE(J.isApproxToConstant(0.0, kThreshold));
  }
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(*argv);
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  return RUN_ALL_TESTS();
}
