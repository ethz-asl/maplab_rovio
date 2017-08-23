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
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include "RovioInterfaceImpl.hpp"
#include "RovioInterfaceStates.hpp"
#include "rovio/RovioFilter.hpp"
#include "rovio/SrvResetToPose.h"

namespace rovio {

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template <typename FILTER> class RovioRosNode {
public:
  typedef FILTER mtFilter;

  // The ROS node does not own this interface, it needs to be
  // constructed/destroyed outside of this class.
  RovioInterface *rovio_interface_;

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
  ros::Publisher pubExtrinsics_[RovioStateImpl<FILTER>::kNumCameras];
  ros::Publisher pubImuBias_;

  // Ros Messages
  geometry_msgs::TransformStamped transformMsg_;
  geometry_msgs::TransformStamped T_J_W_Msg_;
  nav_msgs::Odometry odometryMsg_;
  geometry_msgs::PoseWithCovarianceStamped
      estimatedPoseWithCovarianceStampedMsg_;
  geometry_msgs::PoseWithCovarianceStamped
      extrinsicsMsg_[RovioStateImpl<FILTER>::kNumCameras];
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
  RovioRosNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
               RovioInterface *rovio_interface);

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
  void imgCallback0(const sensor_msgs::ImageConstPtr &img);
  void imgCallback1(const sensor_msgs::ImageConstPtr &img);

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

  /** \brief Get const access to the underlying rovio interface
  */
  /*inline RovioInterface& getRovioInterface() {
    CHECK(rovio_interface_ != nullptr);
    return *rovio_interface_;
  }

  inline const RovioInterface<FILTER> &getRovioInterface() const {
    CHECK(rovio_interface_ != nullptr);
    return *rovio_interface_;
  }*/

  /** \brief Executes the update step of the filter and publishes the updated
   * data.
   */
  void publishState(const RovioState &state);

private:
  static void publishStateCallback(const RovioState &state,
                                   void *this_pointer) {
    RovioRosNode<FILTER> *self =
        static_cast<RovioRosNode<FILTER> *>(this_pointer);
    self->publishState(state);
  }
};

template <typename FILTER>
RovioRosNode<FILTER>::RovioRosNode(ros::NodeHandle &nh,
                                   ros::NodeHandle &nh_private,
                                   RovioInterface *rovio_interface)
    : rovio_interface_(rovio_interface), nh_(nh), nh_private_(nh_private) {

#ifndef NDEBUG
  ROS_WARN("====================== Debug Mode ======================");
#endif

  forceOdometryPublishing_ = false;
  forcePoseWithCovariancePublishing_ = false;
  forceTransformPublishing_ = false;
  forceExtrinsicsPublishing_ = false;
  forceImuBiasPublishing_ = false;
  forcePclPublishing_ = false;
  forceMarkersPublishing_ = false;
  forcePatchPublishing_ = false;
  gotFirstMessages_ = false;

  // Subscribe topics
  subImu_ = nh_.subscribe("imu0", 1000, &RovioRosNode::imuCallback, this);
  subImg0_ =
      nh_.subscribe("cam0/image_raw", 1000, &RovioRosNode::imgCallback0, this);
  subImg1_ =
      nh_.subscribe("cam1/image_raw", 1000, &RovioRosNode::imgCallback1, this);
  subGroundtruth_ =
      nh_.subscribe("pose", 1000, &RovioRosNode::groundtruthCallback, this);
  subGroundtruthOdometry_ = nh_.subscribe(
      "odometry", 1000, &RovioRosNode::groundtruthOdometryCallback, this);
  subVelocity_ =
      nh_.subscribe("abss/twist", 1000, &RovioRosNode::velocityCallback, this);

  // Initialize ROS service servers.
  srvResetFilter_ = nh_.advertiseService(
      "rovio/reset", &RovioRosNode::resetServiceCallback, this);
  srvResetToPoseFilter_ = nh_.advertiseService(
      "rovio/reset_to_pose", &RovioRosNode::resetToPoseServiceCallback, this);

  // Advertise topics
  pubTransform_ =
      nh_.advertise<geometry_msgs::TransformStamped>("rovio/transform", 1);
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("rovio/odometry", 1);
  pubPoseWithCovStamped_ =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "rovio/pose_with_covariance_stamped", 1);
  pubPcl_ = nh_.advertise<sensor_msgs::PointCloud2>("rovio/pcl", 1);
  pubPatch_ = nh_.advertise<sensor_msgs::PointCloud2>("rovio/patch", 1);
  pubMarkers_ = nh_.advertise<visualization_msgs::Marker>("rovio/markers", 1);

  pub_T_J_W_transform =
      nh_.advertise<geometry_msgs::TransformStamped>("rovio/T_G_W", 1);
  for (int camID = 0; camID < RovioStateImpl<FILTER>::kNumCameras; camID++) {
    pubExtrinsics_[camID] =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "rovio/extrinsics" + std::to_string(camID), 1);
  }
  pubImuBias_ = nh_.advertise<sensor_msgs::Imu>("rovio/imu_biases", 1);

  // Handle coordinate frame naming
  map_frame_ = "/map";
  world_frame_ = "/world";
  camera_frame_ = "/camera";
  imu_frame_ = "/imu";
  nh_private_.param("map_frame", map_frame_, map_frame_);
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("camera_frame", camera_frame_, camera_frame_);
  nh_private_.param("imu_frame", imu_frame_, imu_frame_);

  // Initialize messages
  transformMsg_.header.frame_id = world_frame_;
  transformMsg_.child_frame_id = imu_frame_;

  T_J_W_Msg_.child_frame_id = world_frame_;
  T_J_W_Msg_.header.frame_id = map_frame_;

  odometryMsg_.header.frame_id = world_frame_;
  odometryMsg_.child_frame_id = imu_frame_;
  msgSeq_ = 1;
  for (int camID = 0; camID < RovioStateImpl<FILTER>::kNumCameras; camID++) {
    extrinsicsMsg_[camID].header.frame_id = imu_frame_;
  }
  imuBiasMsg_.header.frame_id = world_frame_;
  imuBiasMsg_.orientation.x = 0;
  imuBiasMsg_.orientation.y = 0;
  imuBiasMsg_.orientation.z = 0;
  imuBiasMsg_.orientation.w = 1;
  for (int i = 0; i < 9; i++) {
    imuBiasMsg_.orientation_covariance[i] = 0.0;
  }

  // PointCloud message.
  pclMsg_.header.frame_id = imu_frame_;
  pclMsg_.height = 1; // Unordered point cloud.
  pclMsg_.width =
      RovioStateImpl<FILTER>::kMaxNumFeatures; // Number of features/points.
  const int nFieldsPcl = 18;
  std::string namePcl[nFieldsPcl] = {
      "id",  "camId", "rgb",  "status", "x",    "y",    "z",    "b_x",  "b_y",
      "b_z", "d",     "c_00", "c_01",   "c_02", "c_11", "c_12", "c_22", "c_d"};
  int sizePcl[nFieldsPcl] = {4, 4, 4, 4, 4, 4, 4, 4, 4,
                             4, 4, 4, 4, 4, 4, 4, 4, 4};
  int countPcl[nFieldsPcl] = {1, 1, 1, 1, 1, 1, 1, 1, 1,
                              1, 1, 1, 1, 1, 1, 1, 1, 1};
  int datatypePcl[nFieldsPcl] = {
      sensor_msgs::PointField::INT32,   sensor_msgs::PointField::INT32,
      sensor_msgs::PointField::UINT32,  sensor_msgs::PointField::UINT32,
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::FLOAT32,
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::FLOAT32,
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::FLOAT32,
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::FLOAT32,
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::FLOAT32,
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::FLOAT32,
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::FLOAT32};
  pclMsg_.fields.resize(nFieldsPcl);
  int byteCounter = 0;
  for (int i = 0; i < nFieldsPcl; i++) {
    pclMsg_.fields[i].name = namePcl[i];
    pclMsg_.fields[i].offset = byteCounter;
    pclMsg_.fields[i].count = countPcl[i];
    pclMsg_.fields[i].datatype = datatypePcl[i];
    byteCounter += sizePcl[i] * countPcl[i];
  }
  pclMsg_.point_step = byteCounter;
  pclMsg_.row_step = pclMsg_.point_step * pclMsg_.width;
  pclMsg_.data.resize(pclMsg_.row_step * pclMsg_.height);
  pclMsg_.is_dense = false;

  // PointCloud message.
  patchMsg_.header.frame_id = "";
  patchMsg_.height = 1; // Unordered point cloud.
  patchMsg_.width =
      RovioStateImpl<FILTER>::kMaxNumFeatures; // Number of features/points.
  const int nFieldsPatch = 5;
  std::string namePatch[nFieldsPatch] = {"id", "patch", "dx", "dy", "error"};
  int sizePatch[nFieldsPatch] = {4, 4, 4, 4, 4};
  int countPatch[nFieldsPatch] = {
      1, RovioStateImpl<FILTER>::kPatchAreaTimesLevels,
      RovioStateImpl<FILTER>::kPatchAreaTimesLevels,
      RovioStateImpl<FILTER>::kPatchAreaTimesLevels,
      RovioStateImpl<FILTER>::kPatchAreaTimesLevels};
  int datatypePatch[nFieldsPatch] = {
      sensor_msgs::PointField::INT32, sensor_msgs::PointField::FLOAT32,
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::FLOAT32,
      sensor_msgs::PointField::FLOAT32};
  patchMsg_.fields.resize(nFieldsPatch);
  byteCounter = 0;
  for (int i = 0; i < nFieldsPatch; i++) {
    patchMsg_.fields[i].name = namePatch[i];
    patchMsg_.fields[i].offset = byteCounter;
    patchMsg_.fields[i].count = countPatch[i];
    patchMsg_.fields[i].datatype = datatypePatch[i];
    byteCounter += sizePatch[i] * countPatch[i];
  }
  patchMsg_.point_step = byteCounter;
  patchMsg_.row_step = patchMsg_.point_step * patchMsg_.width;
  patchMsg_.data.resize(patchMsg_.row_step * patchMsg_.height);
  patchMsg_.is_dense = false;

  // Marker message (vizualization of uncertainty)
  markerMsg_.header.frame_id = imu_frame_;
  markerMsg_.id = 0;
  markerMsg_.type = visualization_msgs::Marker::LINE_LIST;
  markerMsg_.action = visualization_msgs::Marker::ADD;
  markerMsg_.pose.position.x = 0;
  markerMsg_.pose.position.y = 0;
  markerMsg_.pose.position.z = 0;
  markerMsg_.pose.orientation.x = 0.0;
  markerMsg_.pose.orientation.y = 0.0;
  markerMsg_.pose.orientation.z = 0.0;
  markerMsg_.pose.orientation.w = 1.0;
  markerMsg_.scale.x = 0.04; // Line width.
  markerMsg_.color.a = 1.0;
  markerMsg_.color.r = 0.0;
  markerMsg_.color.g = 1.0;
  markerMsg_.color.b = 0.0;

  // Register state update callback.
  typename RovioInterface::RovioStateCallback callback = std::bind(
      &RovioRosNode<FILTER>::publishStateCallback, std::placeholders::_1, this);
  CHECK_NOTNULL(rovio_interface_)->registerStateUpdateCallback(callback);
}

template <typename FILTER>
void RovioRosNode<FILTER>::imuCallback(
    const sensor_msgs::Imu::ConstPtr &imu_msg) {
  Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                      imu_msg->linear_acceleration.y,
                      imu_msg->linear_acceleration.z);
  Eigen::Vector3d gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
                      imu_msg->angular_velocity.z);
  const double time_s = imu_msg->header.stamp.toSec();

  constexpr bool kUpdateFilter = true;
  CHECK_NOTNULL(rovio_interface_)->processImuUpdate(acc, gyr, time_s,
                                                    kUpdateFilter);
}

template <typename FILTER>
void RovioRosNode<FILTER>::imgCallback0(const sensor_msgs::ImageConstPtr &img) {
  constexpr int camID = 0;
  if (camID < RovioStateImpl<FILTER>::kNumCameras) {
    imgCallback(img, camID);
  } else {
    LOG_EVERY_N(WARNING, 100)
        << "Discarding image callback with camID " << camID;
  }
}

template <typename FILTER>
void RovioRosNode<FILTER>::imgCallback1(const sensor_msgs::ImageConstPtr &img) {
  constexpr int camID = 1;
  if (camID < RovioStateImpl<FILTER>::kNumCameras) {
    imgCallback(img, camID);
  } else {
    LOG_EVERY_N(WARNING, 100)
        << "Discarding image callback with camID " << camID;
  }
}

template <typename FILTER>
void RovioRosNode<FILTER>::imgCallback(const sensor_msgs::ImageConstPtr &img,
                                       const int camID) {
  CHECK_LT(camID, RovioStateImpl<FILTER>::kNumCameras);

  // Get image from msg
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat cv_img;
  cv_ptr->image.copyTo(cv_img);

  const double time_s = img->header.stamp.toSec();

  CHECK_NOTNULL(rovio_interface_)->processImageUpdate(camID, cv_img, time_s);
}

template <typename FILTER>
void RovioRosNode<FILTER>::groundtruthCallback(
    const geometry_msgs::TransformStamped::ConstPtr &transform) {

  Eigen::Vector3d JrJV(transform->transform.translation.x,
                       transform->transform.translation.y,
                       transform->transform.translation.z);
  QPD qJV(transform->transform.rotation.w, transform->transform.rotation.x,
          transform->transform.rotation.y, transform->transform.rotation.z);
  const double time_s = transform->header.stamp.toSec();

  CHECK_NOTNULL(rovio_interface_)->processGroundTruthUpdate(JrJV, qJV, time_s);
}

template <typename FILTER>
void RovioRosNode<FILTER>::groundtruthOdometryCallback(
    const nav_msgs::Odometry::ConstPtr &odometry) {

  Eigen::Vector3d JrJV(odometry->pose.pose.position.x,
                       odometry->pose.pose.position.y,
                       odometry->pose.pose.position.z);

  QPD qJV(odometry->pose.pose.orientation.w, odometry->pose.pose.orientation.x,
          odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z);

  const Eigen::Matrix<double, 6, 6> measuredCov =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
          odometry->pose.covariance.data());

  const double time_s = odometry->header.stamp.toSec();

  CHECK_NOTNULL(rovio_interface_)
      ->processGroundTruthOdometryUpdate(JrJV, qJV, measuredCov, time_s);
}

template <typename FILTER>
void RovioRosNode<FILTER>::velocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr &velocity) {
  Eigen::Vector3d AvM(velocity->twist.linear.x, velocity->twist.linear.y,
                      velocity->twist.linear.z);
  const double time_s = velocity->header.stamp.toSec();

  CHECK_NOTNULL(rovio_interface_)->processVelocityUpdate(AvM, time_s);
}

template <typename FILTER>
bool RovioRosNode<FILTER>::resetServiceCallback(
    std_srvs::Empty::Request & /*request*/,
    std_srvs::Empty::Response & /*response*/) {
  CHECK_NOTNULL(rovio_interface_)->requestReset();
  return true;
}

template <typename FILTER>
bool RovioRosNode<FILTER>::resetToPoseServiceCallback(
    rovio::SrvResetToPose::Request &request,
    rovio::SrvResetToPose::Response & /*response*/) {
  V3D WrWM(request.T_WM.position.x, request.T_WM.position.y,
           request.T_WM.position.z);
  QPD qWM(request.T_WM.orientation.w, request.T_WM.orientation.x,
          request.T_WM.orientation.y, request.T_WM.orientation.z);

  CHECK_NOTNULL(rovio_interface_)->requestResetToPose(WrWM, qWM.inverted());

  return true;
}

template <typename FILTER>
void RovioRosNode<FILTER>::publishState(const RovioState &state) {
  CHECK_NOTNULL(rovio_interface_);

  // Update the settings that determine if the current state contains
  // information about the patches and features or not.
  // This will only have an effect next time publishState is called.
  const bool publishFeatureState =
      pubPcl_.getNumSubscribers() > 0 || pubMarkers_.getNumSubscribers() > 0 ||
      forcePclPublishing_ || forceMarkersPublishing_;
  rovio_interface_->setEnableFeatureUpdateOutput(publishFeatureState);
  const bool publishPatchState =
      pubPatch_.getNumSubscribers() > 0 || forcePatchPublishing_;
  rovio_interface_->setEnablePatchUpdateOutput(publishPatchState);

  if (!state.getIsInitialized()) {
    return;
  }

  ros::Time rosTimeAfterUpdate = ros::Time(state.getTimestamp());

  // TODO(mfehr): there has to be a better way than typedef-ing.
  typedef typename mtFilter::mtFilterState::mtState mtState;
  typedef StandardOutput mtOutput;

  // Send Map (Pose Sensor, I) to World (rovio-intern, W) transformation
  if (state.getHasInertialPose()) {

    const Eigen::Vector3d &IrIW = state.get_IrIW();
    const QPD &qWI = state.get_qWI();

    tf::StampedTransform tf_transform_WI;
    tf_transform_WI.frame_id_ = map_frame_;
    tf_transform_WI.child_frame_id_ = world_frame_;
    tf_transform_WI.stamp_ = rosTimeAfterUpdate;
    tf_transform_WI.setOrigin(tf::Vector3(IrIW(0), IrIW(1), IrIW(2)));
    tf_transform_WI.setRotation(
        tf::Quaternion(qWI.x(), qWI.y(), qWI.z(), -qWI.w()));

    // Publish.
    tb_.sendTransform(tf_transform_WI);
  }

  // Send IMU pose.
  {
    const Eigen::Vector3d &WrWB = state.get_WrWB();
    const kindr::RotationQuaternionPD &qBW = state.get_qBW();

    tf::StampedTransform tf_transform_MW;
    tf_transform_MW.frame_id_ = world_frame_;
    tf_transform_MW.child_frame_id_ = imu_frame_;
    tf_transform_MW.stamp_ = rosTimeAfterUpdate;
    tf_transform_MW.setOrigin(tf::Vector3(WrWB(0), WrWB(1), WrWB(2)));
    tf_transform_MW.setRotation(
        tf::Quaternion(qBW.x(), qBW.y(), qBW.z(), -qBW.w()));

    // Publish.
    tb_.sendTransform(tf_transform_MW);
  }

  // Send camera extrinsics.
  for (int camID = 0; camID < RovioStateImpl<FILTER>::kNumCameras; camID++) {
    const Eigen::Vector3d &MrMC = state.get_MrMC(camID);
    const kindr::RotationQuaternionPD &qCM = state.get_qCM(camID);

    tf::StampedTransform tf_transform_CM;
    tf_transform_CM.frame_id_ = imu_frame_;
    tf_transform_CM.child_frame_id_ = camera_frame_ + std::to_string(camID);
    tf_transform_CM.stamp_ = rosTimeAfterUpdate;
    tf_transform_CM.setOrigin(tf::Vector3(MrMC(0), MrMC(1), MrMC(2)));
    tf_transform_CM.setRotation(
        tf::Quaternion(qCM.x(), qCM.y(), qCM.z(), -qCM.w()));

    // Publish.
    tb_.sendTransform(tf_transform_CM);
  }

  // Publish Odometry
  if (pubOdometry_.getNumSubscribers() > 0 || forceOdometryPublishing_) {
    const Eigen::MatrixXd &imuCovariance = state.getImuCovariance();
    const Eigen::Vector3d &WrWB = state.get_WrWB();
    const kindr::RotationQuaternionPD &qBW = state.get_qBW();
    const Eigen::Vector3d &BvB = state.get_BvB();
    const Eigen::Vector3d &BwWB = state.get_BwWB();

    odometryMsg_.header.seq = msgSeq_;
    odometryMsg_.header.stamp = rosTimeAfterUpdate;
    odometryMsg_.pose.pose.position.x = WrWB(0);
    odometryMsg_.pose.pose.position.y = WrWB(1);
    odometryMsg_.pose.pose.position.z = WrWB(2);
    odometryMsg_.pose.pose.orientation.w = -qBW.w();
    odometryMsg_.pose.pose.orientation.x = qBW.x();
    odometryMsg_.pose.pose.orientation.y = qBW.y();
    odometryMsg_.pose.pose.orientation.z = qBW.z();
    for (unsigned int i = 0; i < 6; i++) {
      unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>() + i;
      if (i >= 3)
        ind1 = mtOutput::template getId<mtOutput::_att>() + i - 3;
      for (unsigned int j = 0; j < 6; j++) {
        unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>() + j;
        if (j >= 3)
          ind2 = mtOutput::template getId<mtOutput::_att>() + j - 3;

        CHECK_LT(ind1, imuCovariance.rows());
        CHECK_GE(ind1, 0);
        CHECK_LT(ind2, imuCovariance.cols());
        CHECK_GE(ind2, 0);
        odometryMsg_.pose.covariance[j + 6 * i] = imuCovariance(ind1, ind2);
      }
    }
    odometryMsg_.twist.twist.linear.x = BvB(0);
    odometryMsg_.twist.twist.linear.y = BvB(1);
    odometryMsg_.twist.twist.linear.z = BvB(2);
    odometryMsg_.twist.twist.angular.x = BwWB(0);
    odometryMsg_.twist.twist.angular.y = BwWB(1);
    odometryMsg_.twist.twist.angular.z = BwWB(2);
    for (unsigned int i = 0; i < 6; i++) {
      unsigned int ind1 = mtOutput::template getId<mtOutput::_vel>() + i;
      if (i >= 3)
        ind1 = mtOutput::template getId<mtOutput::_ror>() + i - 3;
      for (unsigned int j = 0; j < 6; j++) {
        unsigned int ind2 = mtOutput::template getId<mtOutput::_vel>() + j;
        if (j >= 3)
          ind2 = mtOutput::template getId<mtOutput::_ror>() + j - 3;

        CHECK_LT(ind1, imuCovariance.rows());
        CHECK_GE(ind1, 0);
        CHECK_LT(ind2, imuCovariance.cols());
        CHECK_GE(ind2, 0);
        odometryMsg_.twist.covariance[j + 6 * i] = imuCovariance(ind1, ind2);
      }
    }

    // Publish.
    pubOdometry_.publish(odometryMsg_);
  }

  if (pubPoseWithCovStamped_.getNumSubscribers() > 0 ||
      forcePoseWithCovariancePublishing_) {
    const Eigen::MatrixXd &imuCovariance = state.getImuCovariance();
    const Eigen::Vector3d &WrWB = state.get_WrWB();
    const kindr::RotationQuaternionPD &qBW = state.get_qBW();

    estimatedPoseWithCovarianceStampedMsg_.header.seq = msgSeq_;
    estimatedPoseWithCovarianceStampedMsg_.header.stamp = rosTimeAfterUpdate;
    estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.x = WrWB(0);
    estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.y = WrWB(1);
    estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.z = WrWB(2);
    estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.w = -qBW.w();
    estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.x = qBW.x();
    estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.y = qBW.y();
    estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.z = qBW.z();

    for (unsigned int i = 0; i < 6; i++) {
      unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>() + i;
      if (i >= 3)
        ind1 = mtOutput::template getId<mtOutput::_att>() + i - 3;
      for (unsigned int j = 0; j < 6; j++) {
        unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>() + j;
        if (j >= 3)
          ind2 = mtOutput::template getId<mtOutput::_att>() + j - 3;

        CHECK_LT(ind1, imuCovariance.rows());
        CHECK_GE(ind1, 0);
        CHECK_LT(ind2, imuCovariance.cols());
        CHECK_GE(ind2, 0);
        estimatedPoseWithCovarianceStampedMsg_.pose.covariance[j + 6 * i] =
            imuCovariance(ind1, ind2);
      }
    }

    // Publish.
    pubPoseWithCovStamped_.publish(estimatedPoseWithCovarianceStampedMsg_);
  }

  // Send IMU pose message.
  if (pubTransform_.getNumSubscribers() > 0 || forceTransformPublishing_) {
    const Eigen::Vector3d &WrWB = state.get_WrWB();
    const kindr::RotationQuaternionPD &qBW = state.get_qBW();

    transformMsg_.header.seq = msgSeq_;
    transformMsg_.header.stamp = rosTimeAfterUpdate;
    transformMsg_.transform.translation.x = WrWB(0);
    transformMsg_.transform.translation.y = WrWB(1);
    transformMsg_.transform.translation.z = WrWB(2);
    transformMsg_.transform.rotation.x = qBW.x();
    transformMsg_.transform.rotation.y = qBW.y();
    transformMsg_.transform.rotation.z = qBW.z();
    transformMsg_.transform.rotation.w = -qBW.w();

    // Publish.
    pubTransform_.publish(transformMsg_);
  }

  if (pub_T_J_W_transform.getNumSubscribers() > 0 ||
      forceTransformPublishing_) {
    if (state.getHasInertialPose()) {

      const Eigen::Vector3d &IrIW = state.get_IrIW();
      const QPD &qWI = state.get_qWI();

      T_J_W_Msg_.header.seq = msgSeq_;
      T_J_W_Msg_.header.stamp = rosTimeAfterUpdate;
      T_J_W_Msg_.transform.translation.x = IrIW(0);
      T_J_W_Msg_.transform.translation.y = IrIW(1);
      T_J_W_Msg_.transform.translation.z = IrIW(2);
      T_J_W_Msg_.transform.rotation.x = qWI.x();
      T_J_W_Msg_.transform.rotation.y = qWI.y();
      T_J_W_Msg_.transform.rotation.z = qWI.z();
      T_J_W_Msg_.transform.rotation.w = -qWI.w();

      // Publish.
      pub_T_J_W_transform.publish(T_J_W_Msg_);
    }
  }

  // Publish Extrinsics

  for (int camID = 0; camID < RovioStateImpl<FILTER>::kNumCameras; camID++) {
    const Eigen::Vector3d &MrMC = state.get_MrMC(camID);
    const kindr::RotationQuaternionPD &qCM = state.get_qCM(camID);
    const Eigen::MatrixXd &filterCovariance = state.getFilterCovariance();

    if (pubExtrinsics_[camID].getNumSubscribers() > 0 ||
        forceExtrinsicsPublishing_) {

      extrinsicsMsg_[camID].header.seq = msgSeq_;
      extrinsicsMsg_[camID].header.stamp = rosTimeAfterUpdate;
      extrinsicsMsg_[camID].pose.pose.position.x = MrMC(0);
      extrinsicsMsg_[camID].pose.pose.position.y = MrMC(1);
      extrinsicsMsg_[camID].pose.pose.position.z = MrMC(2);
      extrinsicsMsg_[camID].pose.pose.orientation.x = qCM.x();
      extrinsicsMsg_[camID].pose.pose.orientation.y = qCM.y();
      extrinsicsMsg_[camID].pose.pose.orientation.z = qCM.z();
      extrinsicsMsg_[camID].pose.pose.orientation.w = -qCM.w();

      for (unsigned int i = 0; i < 6; i++) {
        unsigned int ind1 = mtState::template getId<mtState::_vep>(camID) + i;
        if (i >= 3)
          ind1 = mtState::template getId<mtState::_vea>(camID) + i - 3;
        for (unsigned int j = 0; j < 6; j++) {
          unsigned int ind2 = mtState::template getId<mtState::_vep>(camID) + j;
          if (j >= 3)
            ind2 = mtState::template getId<mtState::_vea>(camID) + j - 3;

          CHECK_LT(ind1, filterCovariance.rows());
          CHECK_GE(ind1, 0);
          CHECK_LT(ind2, filterCovariance.cols());
          CHECK_GE(ind2, 0);
          extrinsicsMsg_[camID].pose.covariance[j + 6 * i] =
              filterCovariance(ind1, ind2);
        }
      }

      // Publish.
      pubExtrinsics_[camID].publish(extrinsicsMsg_[camID]);
    }
  }

  // Publish IMU biases
  if (pubImuBias_.getNumSubscribers() > 0 || forceImuBiasPublishing_) {
    const Eigen::Vector3d &gyb = state.getGyb();
    const Eigen::Vector3d &acb = state.getAcb();
    const Eigen::MatrixXd &filterCovariance = state.getFilterCovariance();

    imuBiasMsg_.header.seq = msgSeq_;
    imuBiasMsg_.header.stamp = rosTimeAfterUpdate;
    imuBiasMsg_.angular_velocity.x = gyb(0);
    imuBiasMsg_.angular_velocity.y = gyb(1);
    imuBiasMsg_.angular_velocity.z = gyb(2);
    imuBiasMsg_.linear_acceleration.x = acb(0);
    imuBiasMsg_.linear_acceleration.y = acb(1);
    imuBiasMsg_.linear_acceleration.z = acb(2);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        imuBiasMsg_.angular_velocity_covariance[3 * i + j] =
            filterCovariance(mtState::template getId<mtState::_gyb>() + i,
                             mtState::template getId<mtState::_gyb>() + j);
      }
    }
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        imuBiasMsg_.linear_acceleration_covariance[3 * i + j] =
            filterCovariance(mtState::template getId<mtState::_acb>() + i,
                             mtState::template getId<mtState::_acb>() + j);
      }
    }

    // Publish.
    pubImuBias_.publish(imuBiasMsg_);
  }

  // PointCloud message.
  if (state.hasFeatureState() && publishFeatureState) {
    const RovioFeatureState &feature_state = state.getFeatureState();

    // Prepare point cloud message and markers.
    pclMsg_.header.seq = msgSeq_;
    pclMsg_.header.stamp = rosTimeAfterUpdate;

    markerMsg_.header.seq = msgSeq_;
    markerMsg_.header.stamp = rosTimeAfterUpdate;
    markerMsg_.points.clear();
    float badPoint = std::numeric_limits<float>::quiet_NaN(); // Invalid point.
    int offset = 0;

    FeatureDistance distance;
    double d, d_minus, d_plus;
    const double stretchFactor = 3;
    for (unsigned int i = 0; i < RovioStateImpl<FILTER>::kMaxNumFeatures;
         i++, offset += pclMsg_.point_step) {
      if (feature_state.get_isFeatureValid(i)) {
        const int camID = feature_state.get_FeatureObservrCamID(i);
        const Eigen::Vector3d &CrCPm = feature_state.get_CrCPm(i);
        const Eigen::Vector3d &CrCPp = feature_state.get_CrCPp(i);
        const Eigen::Vector3f &bearing = feature_state.get_bearings(i);
        const Eigen::Vector3f &MrMP = feature_state.get_MrMP(i);
        const Eigen::Matrix3f &cov_MrMP = feature_state.get_cov_MrMP(i);
        const float distance = feature_state.get_Distance(i);
        const float distance_cov = feature_state.get_DistanceCov(i);
        const uint32_t status = feature_state.get_Status(i);
        const int idx = feature_state.get_FeatureIndex(i);

        // Write feature id, camera id, and rgb
        uint8_t gray = 255;
        uint32_t rgb = (gray << 16) | (gray << 8) | gray;
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &idx,
               sizeof(int)); // id
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[1].offset], &camID,
               sizeof(int)); // cam id
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[2].offset], &rgb,
               sizeof(uint32_t)); // rgb
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[3].offset], &status,
               sizeof(int)); // status

        // Write coordinates to pcl message.
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[4].offset], &MrMP[0],
               sizeof(float)); // x
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[5].offset], &MrMP[1],
               sizeof(float)); // y
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[6].offset], &MrMP[2],
               sizeof(float)); // z

        // Add feature bearing vector and distance
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[7].offset], &bearing[0],
               sizeof(float)); // x
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[8].offset], &bearing[1],
               sizeof(float)); // y
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[9].offset], &bearing[2],
               sizeof(float)); // z
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[10].offset], &distance,
               sizeof(float)); // d

        // Add the corresponding covariance (upper triangular)
        int mCounter = 11;
        for (int row = 0; row < 3; row++) {
          for (int col = row; col < 3; col++) {
            memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset],
                   &cov_MrMP(row, col), sizeof(float));
            mCounter++;
          }
        }

        // Add distance uncertainty
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset],
               &distance_cov, sizeof(float));

        // Line markers (Uncertainty rays).
        geometry_msgs::Point point_near_msg;
        point_near_msg.x = float(CrCPp[0]);
        point_near_msg.y = float(CrCPp[1]);
        point_near_msg.z = float(CrCPp[2]);
        markerMsg_.points.push_back(point_near_msg);
        geometry_msgs::Point point_far_msg;
        point_far_msg.x = float(CrCPm[0]);
        point_far_msg.y = float(CrCPm[1]);
        point_far_msg.z = float(CrCPm[2]);
        markerMsg_.points.push_back(point_far_msg);
      } else {
        // If current feature is not valid copy NaN
        int id = -1;
        memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &id,
               sizeof(int)); // id
        for (int j = 1; j < pclMsg_.fields.size(); j++) {
          memcpy(&pclMsg_.data[offset + pclMsg_.fields[j].offset], &badPoint,
                 sizeof(float));
        }
      }
    }

    // Publish.
    pubPcl_.publish(pclMsg_);
    pubMarkers_.publish(markerMsg_);
  }

  if (state.hasPatchState() && publishPatchState) {
    const RovioPatchState &patch_state = state.getPatchState();

    patchMsg_.header.seq = msgSeq_;
    patchMsg_.header.stamp = rosTimeAfterUpdate;
    int offset = 0;
    for (unsigned int i = 0; i < RovioStateImpl<FILTER>::kMaxNumFeatures;
         i++, offset += patchMsg_.point_step) {

      if (patch_state.get_isFeatureValid(i)) {
        const int patch_index = patch_state.get_PatchIndex(i);
        memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset],
               &patch_index, sizeof(int)); // id
        // Add patch data
        for (int l = 0; l < RovioStateImpl<FILTER>::kNumPatchLevels; l++) {
          for (int y = 0; y < RovioStateImpl<FILTER>::kPatchSize; y++) {
            for (int x = 0; x < RovioStateImpl<FILTER>::kPatchSize; x++) {

              const float pixel = patch_state.get_PatchPixel(
                  i, l, y * RovioStateImpl<FILTER>::kPatchSize + x);
              const float pixel_dx = patch_state.get_PatchDx(
                  i, l, y * RovioStateImpl<FILTER>::kPatchSize + x);
              const float pixel_dy = patch_state.get_PatchDy(
                  i, l, y * RovioStateImpl<FILTER>::kPatchSize + x);
              const float pixel_err = patch_state.get_PatchPixel(
                  i, l, y * RovioStateImpl<FILTER>::kPatchSize + x);

              memcpy(
                  &patchMsg_.data[offset + patchMsg_.fields[1].offset +
                                  (l * RovioStateImpl<FILTER>::kPatchArea +
                                   y * RovioStateImpl<FILTER>::kPatchSize + x) *
                                      4],
                  &pixel,
                  sizeof(float)); // Patch
              memcpy(
                  &patchMsg_.data[offset + patchMsg_.fields[2].offset +
                                  (l * RovioStateImpl<FILTER>::kPatchArea +
                                   y * RovioStateImpl<FILTER>::kPatchSize + x) *
                                      4],
                  &pixel_dx,
                  sizeof(float)); // dx
              memcpy(
                  &patchMsg_.data[offset + patchMsg_.fields[3].offset +
                                  (l * RovioStateImpl<FILTER>::kPatchArea +
                                   y * RovioStateImpl<FILTER>::kPatchSize + x) *
                                      4],
                  &pixel_dy,
                  sizeof(float)); // dy

              // TODO(mfehr): Make sure this is correct, the patches above and
              // below are retreived via a different path, but should be the
              // same.
              memcpy(
                  &patchMsg_.data[offset + patchMsg_.fields[4].offset +
                                  (l * RovioStateImpl<FILTER>::kPatchArea +
                                   y * RovioStateImpl<FILTER>::kPatchSize + x) *
                                      4],
                  &pixel_err,
                  sizeof(float)); // error
            }
          }
        }
      } else {
        // If current feature is not valid copy NaN
        int id = -1;
        memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset], &id,
               sizeof(int)); // id
      }
    }

    // Publish.
    pubPatch_.publish(patchMsg_);
  }

  // First time we successfully publish a filter state.
  gotFirstMessages_ = true;
}

} // namespace rovio

#endif /* ROVIO_ROVIONODE_HPP_ */
