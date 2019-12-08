/*
 File name: WrenchModuleBase.hpp
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

#include <Eigen/Dense>

#include "ros_gazebo_utils/WrenchLink.hpp"
#include "ros_node_utils/RosNodeModuleBase.hpp"
#include "ros_node_utils/ros_node_utils.hpp"

using namespace ros_node_utils;
namespace gazebo {
namespace wrench {

class WrenchModuleBase : public ros_node_utils::RosNodeModuleBase {
public:
  WrenchModuleBase(ros::NodeHandle *nodeHandle, wrench::WrenchLink *link)
      : ros_node_utils::RosNodeModuleBase(nodeHandle), link_(link),
        force_(Eigen::Vector3d::Zero()), torque_(Eigen::Vector3d::Zero()) {}

  virtual ~WrenchModuleBase() {}

  virtual void initialize() override {
    ros_node_utils::RosNodeModuleBase::initialize();
  }

  virtual void advance(double dt) {}

  virtual void setName(std::string name) { name_ = name; }
  virtual std::string getName() { return name_; }

  virtual void setOrigin(Eigen::Vector3d origin) { origin_ = origin; }
  virtual Eigen::Vector3d getOrigin() { return origin_; }

  virtual void setPosition(Eigen::Vector3d position) { position_ = position; }

  virtual Eigen::Vector3d getPosition() {
    return link_->getPositionWorldtoBase();
  }

  virtual void setOrientation(Eigen::Quaterniond orientation) {
    orientation_ = orientation;
  }

  virtual Eigen::Quaterniond getOrientation() {
    return link_->getOrientationWorldtoBase();
  }

  virtual void setLinearVelocity(Eigen::Vector3d linearVelocity) {
    linearVelocity_ = linearVelocity;
  }

  virtual Eigen::Vector3d getLinearVelocity() {
    return link_->getLinearVelocityOfBaseInBaseFrame();
  }

  virtual void setAngularVelocity(Eigen::Vector3d angularVelocity) {
    angularVelocity_ = angularVelocity;
  }
  virtual Eigen::Vector3d getAngularVelocity() {
    return link_->getAngularVelocityOfBaseInBaseFrame();
  }
  virtual Eigen::Vector3d getForceInWorldFrame() { return force_; }
  virtual Eigen::Vector3d getTorqueInWorldFrame() { return torque_; }

protected:
  std::string name_;

  wrench::WrenchLink *link_;

  Eigen::Vector3d origin_;
  Eigen::Vector3d force_;
  Eigen::Vector3d torque_;

  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;

  Eigen::Vector3d linearVelocity_;
  Eigen::Vector3d angularVelocity_;
};
} // namespace wrench
} // namespace gazebo
