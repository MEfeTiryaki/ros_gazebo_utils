/*
 File name: MagneticWrench.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: tiryaki@is.mpg.de or m.efetiryaki@gmail.com
 Date created: 14.01.2019
 Date last modified: 15.04.2019
 */

#pragma once

// c++
#include <math.h>
#include <mutex>
#include <string>
#include <vector>

#include "ros_gazebo_utils/wrench_modules/WrenchModuleBase.hpp"
#include <std_msgs/Float64MultiArray.h>

namespace gazebo {
namespace wrench {

class MagneticWrench : public WrenchModuleBase {
public:
  MagneticWrench(ros::NodeHandle *nodeHandle, wrench::WrenchLink *link)
      : WrenchModuleBase(nodeHandle, link), M_(1.0), volume_(0.0),
        B_0_(Eigen::Vector3d::Zero()), B_grad_(Eigen::Vector3d::Zero())

  {
    this->name_ = "magnetic Wrench";
  };

  virtual ~MagneticWrench(){};

  virtual void readParameters() override {
    paramRead(this->nodeHandle_, "/physics/magnetic/magnetization", M_);
    paramRead(this->nodeHandle_, "/physics/magnetic/magnet_size", volume_);
  }

  virtual void initializeSubscribers() override {
    magneticFieldSubscriber_ = this->nodeHandle_->subscribe(
        "magnetic_field", 10, &MagneticWrench::magneticFieldCallback, this);
    magneticGradientSubscriber_ = this->nodeHandle_->subscribe(
        "magnetic_gradient", 10, &MagneticWrench::magneticGradientCallback,
        this);
  }

  virtual void advance(double dt) override {
    Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
    m.block(0, 0, 3, 3) = B_grad_;
    force_ = M_ * volume_ * m *
             (link_->getOrientationWorldtoBase() * Eigen::Vector3d::UnitZ());

    torque_ = (link_->getOrientationWorldtoBase() * Eigen::Vector3d::UnitZ())
                  .cross(B_0_);
  }

  // CALLBACK
  void magneticFieldCallback(const std_msgs::Float64MultiArray &msg) {
    if (msg.data.size() == 3) {
      for (int i = 0; i < msg.data.size(); i++) {
        B_0_[i] = msg.data[i];
      }
    } else {
      // ERROR("Magnetic Gradient should be 3D");
    }
  };
  void magneticGradientCallback(const std_msgs::Float64MultiArray &msg) {
    if (msg.data.size() == 3) {
      for (int i = 0; i < msg.data.size(); i++) {
        B_grad_[i] = msg.data[i];
      }
    } else {
      // ERROR("Magnetic Gradient should be 3D");
    }
  };

protected:
  double M_;
  double volume_;
  Eigen::Vector3d B_0_;
  Eigen::Vector3d B_grad_;

  // Subscriber
  ros::Subscriber magneticFieldSubscriber_;
  ros::Subscriber magneticGradientSubscriber_;
};
} // namespace wrench
} // namespace gazebo
