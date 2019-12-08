/*
 File name: WrenchLink.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: tiryaki@is.mpg.de or m.efetiryaki@gmail.com
 Date created: 25.01.2019
 Date last modified:  15.04.2019
 */

#pragma once

// c++
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>


namespace gazebo {
namespace wrench {

class WrenchLink
{
 public:
  WrenchLink():
    mass_(0.0)
  {

  }
  ;

  virtual ~WrenchLink()
  {
  }
  ;

  virtual void initialize()
  {

  }
  ;

  void setPositionWorldtoBase(Eigen::Vector3d position)
  {
    positionWorldtoBaseInWorldFrame_ = position;
  }
  ;

  Eigen::Vector3d getPositionWorldtoBase()
  {
    return positionWorldtoBaseInWorldFrame_;
  }
  ;

  void setCoMPositionWorldtoBase(Eigen::Vector3d position)
  {
    positionWorldtoCoMInWorldFrame_ = position;
  }
  ;

  Eigen::Vector3d getCoMPositionWorldtoBase()
  {
    return positionWorldtoCoMInWorldFrame_;
  }
  ;

  void setOrientationWorldtoBase(Eigen::Quaterniond orientation)
  {
    orientationWorldtoBaseInWorldFrame_ = orientation;
  }
  ;

  Eigen::Quaterniond getOrientationWorldtoBase()
  {
    return orientationWorldtoBaseInWorldFrame_;
  }
  ;

  void setLinearVelocityOfBaseInBaseFrame(Eigen::Vector3d position)
  {
    linearVelocityOfBaseInBaseFrame_ = position;
  }
  ;

  Eigen::Vector3d getLinearVelocityOfBaseInBaseFrame()
  {
    return linearVelocityOfBaseInBaseFrame_;
  }
  ;

  void setAngularVelocityOfBaseInBaseFrame(Eigen::Vector3d position)
  {
    angularVelocityOfBaseInBaseFrame_ = position;
  }
  ;

  Eigen::Vector3d getAngularVelocityOfBaseInBaseFrame()
  {
    return angularVelocityOfBaseInBaseFrame_;
  }
  ;

  void setMass(double mass)
  {
    mass_ = mass;
  }
  ;

  double getMass()
  {
    return mass_;
  }
  ;

  void setGravity(Eigen::Vector3d gravity)
  {
    gravity_ = gravity;
  }
  ;

  Eigen::Vector3d getGravity()
  {
    return gravity_;
  }
  ;

 protected:

  Eigen::Vector3d positionWorldtoBaseInWorldFrame_;
  Eigen::Vector3d positionWorldtoCoMInWorldFrame_;
  Eigen::Quaterniond orientationWorldtoBaseInWorldFrame_;
  Eigen::Vector3d linearVelocityOfBaseInBaseFrame_;
  Eigen::Vector3d angularVelocityOfBaseInBaseFrame_;

  double mass_;
  Eigen::Vector3d gravity_;
};
}  // namespace wrench
}  // namespace gazebo
