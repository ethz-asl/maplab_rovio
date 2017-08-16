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

#ifndef ROVIO_ROVIONODE_HPP_
#define ROVIO_ROVIONODE_HPP_

#include <functional>
#include <memory>
#include <queue>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/RovioFilter.hpp"
#include "rovio/SrvResetToPose.h"
#include "rovio/rovio-interface.h"

namespace rovio {

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template <typename FILTER> class RovioNode {
public:
  bool forceOdometryPublishing_;
  bool forcePoseWithCovariancePublishing_;
  bool forceTransformPublishing_;
  bool forceExtrinsicsPublishing_;
  bool forceImuBiasPublishing_;
  bool forcePclPublishing_;
  bool forceMarkersPublishing_;
  bool forcePatchPublishing_;
  bool gotFirstMessages_;

  // Nodes, Subscriber, Publishers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg0_;
  ros::Subscriber subImg1_;
  ros::Subscriber subGroundtruth_;
  ros::Subscriber subGroundtruthOdometry_;
  ros::Subscriber subVelocity_;
  ros::ServiceServer srvResetFilter_;
  ros::ServiceServer srvResetToPoseFilter_;
  ros::Publisher pubOdometry_;
  ros::Publisher pubTransform_;
  ros::Publisher pubPoseWithCovStamped_;
  ros::Publisher pub_T_J_W_transform;
  tf::TransformBroadcaster tb_;
  ros::Publisher
      pubPcl_; /**<Publisher: Ros point cloud, visualizing the landmarks.*/
  ros::Publisher pubPatch_;   /**<Publisher: Patch data.*/
  ros::Publisher pubMarkers_; /**<Publisher: Ros line marker, indicating the
                                 depth uncertainty of a landmark.*/
  ros::Publisher pubExtrinsics_[mtState::nCam_];
  ros::Publisher pubImuBias_;

  // Ros Messages
  geometry_msgs::TransformStamped transformMsg_;
  geometry_msgs::TransformStamped T_J_W_Msg_;
  nav_msgs::Odometry odometryMsg_;
  geometry_msgs::PoseWithCovarianceStamped
      estimatedPoseWithCovarianceStampedMsg_;
  geometry_msgs::PoseWithCovarianceStamped extrinsicsMsg_[mtState::nCam_];
  sensor_msgs::PointCloud2 pclMsg_;
  sensor_msgs::PointCloud2 patchMsg_;
  visualization_msgs::Marker markerMsg_;
  sensor_msgs::Imu imuBiasMsg_;
  int msgSeq_;

  // ROS names for output tf frames.
  std::string map_frame_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string imu_frame_;

  /** \brief Constructor
   */
  RovioNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
            std::shared_ptr<mtFilter> mpFilter);

  /** \brief Destructor
   */
  virtual ~RovioNode() {}

  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  void makeTest() { rovio_interface_.makeTest(); }

  /** \brief Callback for IMU-Messages. Adds IMU measurements (as prediction
   * measurements) to the filter.
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);

  /** \brief Image callback. Adds images (as update measurements) to the filter.
   *
   *   @param img   - Image message.
   *   @param camID - Camera ID.
   */
  void imgCallback(const sensor_msgs::ImageConstPtr &img, const int camID);

  /** \brief Callback for external groundtruth as TransformStamped
   *
   *  @param transform - Groundtruth message.
   */
  void groundtruthCallback(
      const geometry_msgs::TransformStamped::ConstPtr &transform);

  /** \brief Callback for external groundtruth as Odometry
   *
   * @param odometry - Groundtruth message.
   */
  void
  groundtruthOdometryCallback(const nav_msgs::Odometry::ConstPtr &odometry);

  /** \brief Callback for external velocity measurements
   *
   *  @param transform - Groundtruth message.
   */
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &velocity);

  /** \brief ROS service handler for resetting the filter.
   */
  bool resetServiceCallback(std_srvs::Empty::Request & /*request*/,
                            std_srvs::Empty::Response & /*response*/);

  /** \brief ROS service handler for resetting the filter to a given pose.
   */
  bool
  resetToPoseServiceCallback(rovio::SrvResetToPose::Request &request,
                             rovio::SrvResetToPose::Response & /*response*/);

  /** \brief Executes the update step of the filter and publishes the updated
   * data.
   */
  void publishState(const FilterUpdateState &state);

private:
  RovioInterface rovio_interface_;
};
}

#endif /* ROVIO_ROVIONODE_HPP_ */
