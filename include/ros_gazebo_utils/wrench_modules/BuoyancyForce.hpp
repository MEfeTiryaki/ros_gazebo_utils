/*
 File name: BuoyancyForce.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: tiryaki@is.mpg.de or m.efetiryaki@gmail.com
 Date created: 25.01.2019
 Date last modified:  15.04.2019
 */

#pragma once

// c++
#include <mutex>
#include <string>
#include <vector>
#include <math.h>

#include "ros_gazebo_utils/wrench_modules/WrenchModuleBase.hpp"

using namespace ros_node_utils;

namespace gazebo {
namespace wrench {

class BuoyancyForce : public WrenchModuleBase
{
 public:

  /*! \~english
   Constructor
   */
  BuoyancyForce(ros::NodeHandle* nodeHandle, wrench::WrenchLink* link)
      : WrenchModuleBase(nodeHandle, link),
        volume_(0.0),
        fluidDensity_(1000)
  {
    this->name_ = "buoyancy";
  }
  ;

  /*! \~english

   */
  virtual ~BuoyancyForce()
  {
  }
  ;

  virtual void readParameters() override
  {
    paramRead(this->nodeHandle_, "/physics/shape/volume", volume_);
    paramRead(this->nodeHandle_, "/physics/fluid/fluid_density", fluidDensity_);
    paramRead(this->nodeHandle_, "/physics/fluid/bouyancy_position", origin_);

  }


  virtual void advance(double dt) override
  {
    // TODO M.Efe Tiryaki : density might be variable in future
    calculateFluidDensity();
    force_ =  volume_ * fluidDensity_ *- 1 * this->link_->getGravity();
    torque_ = (this->link_->getPositionWorldtoBase()- this->link_->getCoMPositionWorldtoBase()).cross(force_);
  }

  /*! \~english

   */
  void calculateFluidDensity()
  {
    // TODO : For temperature dependent fluids
    //fluidDensity_ = 960;
  }


 protected:
  double volume_;
  double fluidDensity_;

};
}
}
