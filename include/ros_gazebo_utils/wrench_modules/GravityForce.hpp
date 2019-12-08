/*
 File name: GravityForce.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: tiryaki@is.mpg.de or m.efetiryaki@gmail.com
 Date created: 14.04.2019
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

/*! Dummy Gravity Force Class. It is only used for visulization purposes.
 *  No force exerted.
 *
 *
 */
class GravityForce : public WrenchModuleBase
{
 public:

  GravityForce(ros::NodeHandle* nodeHandle, wrench::WrenchLink* link)
      : WrenchModuleBase(nodeHandle, link)
  {
    this->name_ = "gravity";
  }



  virtual ~GravityForce()
  {
  }


  virtual void readParameters() override
  {

  }

  virtual void initialize() override
  {

  }

  virtual void advance(double dt)
  {
    force_ = this->link_->getMass() * this->link_->getGravity();
  }

};
}  // namespace wrench
}  // namespace gazebo
