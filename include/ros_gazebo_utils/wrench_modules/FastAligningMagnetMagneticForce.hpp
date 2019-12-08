/*
 File name: FastAligningMagnetMagneticForce.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: tiryaki@is.mpg.de or m.efetiryaki@gmail.com
 Date created: 25.01.2019
 Date last modified: 15.04.2019
 */

#pragma once

// c++
#include <mutex>
#include <string>
#include <vector>
#include <math.h>

// ROS Messages

#include <std_msgs/Float64MultiArray.h>

#include "ros_gazebo_utils/wrench_modules/WrenchModuleBase.hpp"

namespace gazebo {
namespace wrench {

class FastAligningMagnetMagneticForce : public WrenchModuleBase
{
 public:
  FastAligningMagnetMagneticForce(ros::NodeHandle* nodeHandle, wrench::WrenchLink* link)
      : WrenchModuleBase(nodeHandle, link),
        M_(1.0),
        volume_(0.0),
        B_0_(Eigen::Vector3d::Zero()),
        B_0_direction_(Eigen::Vector3d(1.0, 0.0, 0.0)),
        B_grad_(Eigen::Vector3d::Zero())

  {
    this->name_ = "fast_aligning_magnet_magnetic_force";
  }

  virtual ~FastAligningMagnetMagneticForce()
  {
  }

  virtual void readParameters() override
  {
    paramRead(this->nodeHandle_, "/physics/magnetic/magnetization", M_);
    paramRead(this->nodeHandle_, "/physics/magnetic/magnet_size", volume_);
    paramRead(this->nodeHandle_, "/physics/magnetic/B_0", B_0_);
    paramRead(this->nodeHandle_, "/physics/magnetic/magnet_position", origin_);

  }

  virtual void initialize() override
  {
    if (B_0_.norm() == 0) {
      B_0_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
      B_0_direction_ = B_0_.normalized();
    }
  }

  virtual void initializeSubscribers() override
  {
    // TODO Efe : add magnetic field here as well
    magneticGradientSubscriber_ = this->nodeHandle_->subscribe(
        "/gazebo/magnetic_gradient", 10, &FastAligningMagnetMagneticForce::magneticGradientCallback, this);
  }

  virtual void advance(double dt) override
  {
    // TODO M.Efe Tiryaki : density might be variable in future
    calculateMagnetization();
    Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
    // TODO : CHECK THIS
    m.block(0, 0, 3, 1) = B_grad_;
    force_ = M_ * volume_ * m * B_0_direction_;
    torque_ = (this->link_->getPositionWorldtoBase()
        + this->link_->getOrientationWorldtoBase() * origin_
        - this->link_->getCoMPositionWorldtoBase()).cross(force_);
  }

  virtual void calculateMagnetization()
  {
    //M_ = 1.05 * 1000000.0;  // A/m
  }

  //CALLBACK
  void magneticGradientCallback(const std_msgs::Float64MultiArray& msg)
  {
    if (msg.data.size() == 3) {
      for (int i = 0; i < msg.data.size(); i++) {
        B_grad_[i] = msg.data[i];
      }
    } else {
      //std::cerr << "\033[0;31m" << "Magnetic Gradient should be 3D" << "\033[0m" << std::endl;
    }
  }

 protected:
  double M_;
  double volume_;
  Eigen::Vector3d B_0_;
  Eigen::Vector3d B_0_direction_;
  Eigen::Vector3d B_grad_;

  // Subscriber
  ros::Subscriber magneticGradientSubscriber_;
};
}  // namespace wrench
}  // namespace gazebo
