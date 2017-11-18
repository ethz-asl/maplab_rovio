
#include <glog/logging.h>

#include "rovio/Camera.hpp"
#include "yaml-cpp/yaml.h"

namespace rovio{

  Camera::Camera(){
    k1_ = 0.0; k2_ = 0.0; k3_ = 0.0; k4_ = 0.0; k5_ = 0.0; k6_ = 0.0;
    p1_ = 0.0; p2_ = 0.0; s1_ = 0.0; s2_ = 0.0; s3_ = 0.0; s4_ = 0.0;
    K_.setIdentity();
    type_ = DistortionModel::RADTAN;
  };

  Camera::~Camera(){};

  bool Camera::loadCalibrationFromFile(const std::string &filename) {
    CameraCalibration calibration;
    calibration.loadFromFile(filename);
    return setCalibration(calibration);
  }

  bool Camera::setCalibration(const CameraCalibration &calibration) {
    CHECK(calibration.hasIntrinsics_);

    // Set camera intrinsics.
    K_ = calibration.K_;

    // Set distortion model.
    type_ = calibration.distortionModel_;

    switch (type_) {
    case DistortionModel::RADTAN:
      CHECK_EQ(calibration.distortionParams_.size(),
               NUM_DISTORTION_MODEL_PARAMS[static_cast<int>(type_)]);

      k1_ = calibration.distortionParams_[0];
      k2_ = calibration.distortionParams_[1];
      p1_ = calibration.distortionParams_[2];
      p2_ = calibration.distortionParams_[3];
      k3_ = calibration.distortionParams_[4];
      std::cout << "Set distortion parameters (Radtan) to: k1(" << k1_
                << "), k2(" << k2_ << "), k3(" << k3_ << "), p1(" << p1_
                << "), p2(" << p2_ << ")" << std::endl;
      break;
    case DistortionModel::EQUIDIST:
      CHECK_EQ(calibration.distortionParams_.size(),
               NUM_DISTORTION_MODEL_PARAMS[static_cast<int>(type_)]);

      k1_ = calibration.distortionParams_[0];
      k2_ = calibration.distortionParams_[1];
      k3_ = calibration.distortionParams_[2];
      k4_ = calibration.distortionParams_[3];
      std::cout << "Set distortion parameters (Equidist) to: k1(" << k1_
                << "), k2(" << k2_ << "), k3(" << k3_ << "), k4(" << k4_ << ")"
                << std::endl;
      break;

    case DistortionModel::FOV:
      CHECK_EQ(calibration.distortionParams_.size(),
               NUM_DISTORTION_MODEL_PARAMS[static_cast<int>(type_)]);

      k1_ = calibration.distortionParams_[0];
      std::cout << "Set distortion parameters (fov) to: w(" << k1_ << ")"
                << std::endl;
      break;
    default:
      std::cout << "ERROR: unknown camera Model detected! (model: "
                << static_cast<int>(type_) << ")" << std::endl;
      return false;
    }
    return true;
  }

  void Camera::distortRadtan(const Eigen::Vector2d& in, Eigen::Vector2d& out) const{
    const double x2 = in(0) * in(0);
    const double y2 = in(1) * in(1);
    const double xy = in(0) * in(1);
    const double r2 = x2 + y2;
    const double kr = (1 + ((k3_ * r2 + k2_) * r2 + k1_) * r2);
    out(0) = in(0) * kr + p1_ * 2 * xy + p2_ * (r2 + 2 * x2);
    out(1) = in(1) * kr + p1_ * (r2 + 2 * y2) + p2_ * 2 * xy;
  }

  void Camera::distortRadtan(const Eigen::Vector2d& in, Eigen::Vector2d& out, Eigen::Matrix2d& J) const{
    const double x2 = in(0) * in(0);
    const double y2 = in(1) * in(1);
    const double xy = in(0) * in(1);
    const double r2 = x2 + y2;
    const double kr = (1 + ((k3_ * r2 + k2_) * r2 + k1_) * r2);
    out(0) = in(0) * kr + p1_ * 2 * xy + p2_ * (r2 + 2 * x2);
    out(1) = in(1) * kr + p1_ * (r2 + 2 * y2) + p2_ * 2 * xy;
    J(0,0) = kr + 2.0 * k1_ * x2 + 4.0 * k2_ * x2 * r2 + 6.0 * k3_ * x2 * r2 * r2 + 2.0 * p1_ * in(1) + 6.0 * p2_ * in(0);
    J(0,1) = 2.0 * k1_ * xy + 4.0 * k2_ * xy * r2 + 6.0 * k3_ * xy * r2 * r2 + 2 * p1_ * in(0) + 2 * p2_ * in(1);
    J(1,0) = J(0,1);
    J(1,1) = kr + 2.0 * k1_ * y2 + 4.0 * k2_ * y2 * r2 + 6.0 * k3_ * y2 * r2 * r2 + 6.0 * p1_ * in(1) + 2.0 * p2_ * in(0);
  }

  void Camera::distortEquidist(const Eigen::Vector2d& in, Eigen::Vector2d& out) const{
    const double x2 = in(0) * in(0);
    const double y2 = in(1) * in(1);
    const double r = std::sqrt(x2 + y2); // 1/r*x

    if(r < 1e-8){
      out(0) = in(0);
      out(1) = in(1);
      return;
    }

    const double th = atan(r); // 1/(r^2 + 1)
    const double th2 = th*th;
    const double th4 = th2*th2;
    const double th6 = th2*th4;
    const double th8 = th2*th6;
    const double thd = th * (1.0 + k1_ * th2 + k2_ * th4 + k3_ * th6 + k4_ * th8);
    const double s = thd/r;

    out(0) = in(0) * s;
    out(1) = in(1) * s;
  }

  void Camera::distortEquidist(const Eigen::Vector2d& in, Eigen::Vector2d& out, Eigen::Matrix2d& J) const{
    const double x2 = in(0) * in(0);
    const double y2 = in(1) * in(1);
    const double r = std::sqrt(x2 + y2);

    if(r < 1e-8){
      out(0) = in(0);
      out(1) = in(1);
      J.setIdentity();
      return;
    }

    const double r_x = 1/r*in(0);
    const double r_y = 1/r*in(1);

    const double th = atan(r); // 1/(r^2 + 1)
    const double th_r = 1/(r*r+1);
    const double th2 = th*th;
    const double th4 = th2*th2;
    const double th6 = th2*th4;
    const double th8 = th2*th6;
    const double thd = th * (1.0 + k1_ * th2 + k2_ * th4 + k3_ * th6 + k4_ * th8);
    const double thd_th = 1.0 + 3 * k1_ * th2 + 5* k2_ * th4 + 7 * k3_ * th6 + 9 * k4_ * th8;
    const double s = thd/r;
    const double s_r = thd_th*th_r/r - thd/(r*r);

    out(0) = in(0) * s;
    out(1) = in(1) * s;

    J(0,0) = s + in(0)*s_r*r_x;
    J(0,1) = in(0)*s_r*r_y;
    J(1,0) = in(1)*s_r*r_x;
    J(1,1) = s + in(1)*s_r*r_y;
  }

  void Camera::distortFov(const Eigen::Vector2d& in, Eigen::Vector2d& out) const {
    const double& w = k1_;
    const double r_u = in.norm();
    const double r_u_cubed = r_u * r_u * r_u;
    const double tanwhalf = tan(w / 2.);
    const double tanwhalfsq = tanwhalf * tanwhalf;
    const double atan_wrd = atan(2. * tanwhalf * r_u);
    double r_rd;

    if (w * w < 1e-5) {
      // Limit w > 0.
      r_rd = 1.0;
    } else {
      if (r_u * r_u < 1e-5) {
        // Limit r_u > 0.
        r_rd = 2. * tanwhalf / w;
      } else {
        r_rd = atan_wrd / (r_u * w);
      }
    }
    out = in * r_rd;
  }

  void Camera::distortFov(const Eigen::Vector2d& in, Eigen::Vector2d& out, Eigen::Matrix2d& J) const{
    const double& w = k1_;
    const double r_u = in.norm();
    const double r_u_cubed = r_u * r_u * r_u;
    const double tanwhalf = tan(w / 2.);
    const double tanwhalfsq = tanwhalf * tanwhalf;
    const double atan_wrd = atan(2. * tanwhalf * r_u);
    double r_rd;

    if (w * w < 1e-5) {
      // Limit w > 0.
      r_rd = 1.0;
    } else {
      if (r_u * r_u < 1e-5) {
        // Limit r_u > 0.
        r_rd = 2. * tanwhalf / w;
      } else {
        r_rd = atan_wrd / (r_u * w);
      }
    }
    out = in * r_rd;

    // Jacobian.
    const double& u = in(0);
    const double& v = in(1);

    J.resize(Eigen::NoChange, 2);
    if (w * w < 1e-5) {
      J.setIdentity();
    }
    else if (r_u * r_u < 1e-5) {
      J.setIdentity();
      // The coordinates get multiplied by an expression not depending on r_u.
      J *= (2. * tanwhalf / w);
    } else {
      const double duf_du = (atan_wrd) / (w * r_u)
                - (u * u * atan_wrd) / (w * r_u_cubed)
                + (2 * u * u * tanwhalf)
                / (w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1));
      const double duf_dv = (2 * u * v * tanwhalf)
                / (w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1))
                - (u * v * atan_wrd) / (w * r_u_cubed);
      const double dvf_du = (2 * u * v * tanwhalf)
                / (w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1))
                - (u * v * atan_wrd) / (w * r_u_cubed);
      const double dvf_dv = (atan_wrd) / (w * r_u)
                - (v * v * atan_wrd) / (w * r_u_cubed)
                + (2 * v * v * tanwhalf)
                / (w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1));

      J << duf_du, duf_dv,
           dvf_du, dvf_dv;
    }
  }

  void Camera::distort(const Eigen::Vector2d& in, Eigen::Vector2d& out) const{
    switch(type_){
      case DistortionModel::RADTAN:
        distortRadtan(in,out);
        break;
      case DistortionModel::EQUIDIST:
        distortEquidist(in,out);
        break;
      case DistortionModel::FOV:
        distortFov(in,out);
        break;
      default:
        LOG(FATAL);
        break;
    }
  }

  void Camera::distort(const Eigen::Vector2d& in, Eigen::Vector2d& out, Eigen::Matrix2d& J) const{
    switch(type_){
      case DistortionModel::RADTAN:
        distortRadtan(in,out,J);
        break;
      case DistortionModel::EQUIDIST:
        distortEquidist(in,out,J);
        break;
      case DistortionModel::FOV:
        distortFov(in,out,J);
        break;
      default:
        LOG(FATAL);
        break;
    }
  }

  bool Camera::bearingToPixel(const Eigen::Vector3d& vec, cv::Point2f& c) const{
    // Project
    if(vec(2)<=0) return false;
    const Eigen::Vector2d undistorted = Eigen::Vector2d(vec(0)/vec(2),vec(1)/vec(2));

    // Distort
    Eigen::Vector2d distorted;
    distort(undistorted,distorted);

    // Shift origin and scale
    c.x = static_cast<float>(K_(0, 0)*distorted(0) + K_(0, 2));
    c.y = static_cast<float>(K_(1, 1)*distorted(1) + K_(1, 2));
    return true;
  }

  bool Camera::bearingToPixel(const Eigen::Vector3d& vec, cv::Point2f& c, Eigen::Matrix<double,2,3>& J) const{
    // Project
    if(vec(2)<=0) return false;
    const Eigen::Vector2d undistorted = Eigen::Vector2d(vec(0)/vec(2),vec(1)/vec(2));
    Eigen::Matrix<double,2,3> J1; J1.setZero();
    J1(0,0) = 1.0/vec(2);
    J1(0,2) = -vec(0)/pow(vec(2),2);
    J1(1,1) = 1.0/vec(2);
    J1(1,2) = -vec(1)/pow(vec(2),2);

    // Distort
    Eigen::Vector2d distorted;
    Eigen::Matrix2d J2;
    distort(undistorted,distorted,J2);

    // Shift origin and scale
    c.x = static_cast<float>(K_(0, 0)*distorted(0) + K_(0, 2));
    c.y = static_cast<float>(K_(1, 1)*distorted(1) + K_(1, 2));
    Eigen::Matrix2d J3; J3.setZero();
    J3(0,0) = K_(0, 0);
    J3(1,1) = K_(1, 1);

    J = J3*J2*J1;

    return true;
  }

  bool Camera::bearingToPixel(const LWF::NormalVectorElement& n, cv::Point2f& c) const{
    return bearingToPixel(n.getVec(),c);
  }

  bool Camera::bearingToPixel(const LWF::NormalVectorElement& n, cv::Point2f& c, Eigen::Matrix<double,2,2>& J) const{
    Eigen::Matrix<double,3,2> J1;
    J1 = n.getM();
    Eigen::Matrix<double,2,3> J2;
    const bool success = bearingToPixel(n.getVec(),c,J2);
    J = J2*J1;
    return success;
  }

  bool Camera::pixelToBearing(const cv::Point2f& c,Eigen::Vector3d& vec) const{
    // Shift origin and scale
    Eigen::Vector2d y;
    y(0) = (static_cast<double>(c.x) - K_(0, 2)) / K_(0, 0);
    y(1) = (static_cast<double>(c.y) - K_(1, 2)) / K_(1, 1);

    // Undistort by optimizing
    const int max_iter = 100;
    const double tolerance = 1e-10;
    Eigen::Vector2d ybar = y; // current guess (undistorted)
    Eigen::Matrix2d J;
    Eigen::Vector2d y_tmp; // current guess (distorted)
    Eigen::Vector2d e;
    Eigen::Vector2d du;
    bool success = false;
    for (int i = 0; i < max_iter; i++) {
      distort(ybar,y_tmp,J);
      e = y - y_tmp;
      du = (J.transpose() * J).inverse() * J.transpose() * e;
      ybar += du;
      if (e.dot(e) <= tolerance){
        success = true;
        break;
      }
    }
    if(success){
      y = ybar;
      vec = Eigen::Vector3d(y(0),y(1),1.0).normalized();
    }
    return success;
  }

  bool Camera::pixelToBearing(const cv::Point2f& c,LWF::NormalVectorElement& n) const{
    Eigen::Vector3d vec;
    bool success = pixelToBearing(c,vec);
    n.setFromVector(vec);
    return success;
  }

  void Camera::testCameraModel(){
    double d = 1e-4;
    LWF::NormalVectorElement b_s;
    LWF::NormalVectorElement b_s1;
    LWF::NormalVectorElement b_s2;
    Eigen::Vector3d v_s;
    Eigen::Vector3d v_s1;
    Eigen::Vector3d v_s2;
    LWF::NormalVectorElement b_e;
    Eigen::Matrix2d J1;
    Eigen::Matrix2d J1_FD;
    Eigen::Matrix<double,2,3> J2;
    Eigen::Matrix<double,2,3> J2_FD;
    cv::Point2f p_s;
    cv::Point2f p_s1;
    cv::Point2f p_s2;
    cv::Point2f p_s3;
    Eigen::Vector2d diff;
    for(unsigned int s = 1; s<10;){
      b_s.setRandom(s);
      if(b_s.getVec()(2)<0) b_s = b_s.inverted();
      bearingToPixel(b_s,p_s,J1);
      pixelToBearing(p_s,b_e);
      b_s.boxMinus(b_e,diff);
      std::cout << b_s.getVec().transpose() << std::endl;
      std::cout << "Error after back and forward mapping: " << diff.norm() << std::endl;
      diff = Eigen::Vector2d(d,0);
      b_s.boxPlus(diff,b_s1);
      bearingToPixel(b_s1,p_s1);
      J1_FD(0,0) = static_cast<double>((p_s1-p_s).x)/d;
      J1_FD(1,0) = static_cast<double>((p_s1-p_s).y)/d;
      diff = Eigen::Vector2d(0,d);
      b_s.boxPlus(diff,b_s2);
      bearingToPixel(b_s2,p_s2);
      J1_FD(0,1) = static_cast<double>((p_s2-p_s).x)/d;
      J1_FD(1,1) = static_cast<double>((p_s2-p_s).y)/d;
      std::cout << J1 << std::endl;
      std::cout << J1_FD << std::endl;

      v_s = b_s.getVec();
      bearingToPixel(v_s,p_s,J2);
      bearingToPixel(v_s + Eigen::Vector3d(d,0,0),p_s1);
      bearingToPixel(v_s + Eigen::Vector3d(0,d,0),p_s2);
      bearingToPixel(v_s + Eigen::Vector3d(0,0,d),p_s3);
      J2_FD(0,0) = static_cast<double>((p_s1-p_s).x)/d;
      J2_FD(1,0) = static_cast<double>((p_s1-p_s).y)/d;
      J2_FD(0,1) = static_cast<double>((p_s2-p_s).x)/d;
      J2_FD(1,1) = static_cast<double>((p_s2-p_s).y)/d;
      J2_FD(0,2) = static_cast<double>((p_s3-p_s).x)/d;
      J2_FD(1,2) = static_cast<double>((p_s3-p_s).y)/d;
      std::cout << J2 << std::endl;
      std::cout << J2_FD << std::endl;
    }
  }
}
