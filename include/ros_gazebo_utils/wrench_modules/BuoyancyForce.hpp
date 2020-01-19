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

#include <std_msgs/Float64.h>


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
    double radius = 0.001;
    paramRead(this->nodeHandle_, "/physics/shape/radius", radius);
    // paramRead(this->nodeHandle_, "/physics/shape/volume", volume_);
    paramRead(this->nodeHandle_, "/physics/fluid/fluid_density", fluidDensity_);
    paramRead(this->nodeHandle_, "/physics/fluid/bouyancy_position", origin_);
    paramRead(this->nodeHandle_, "/physics/temperature", temperature_);

    volume_  = radius*radius*radius*4/3*M_PI;
    updateDensity();
  }


    void initializeSubscribers(){
      temperatureSubscriber_ = getNodeHandle()->subscribe(
          "/gazebo/temperature", 1, &BuoyancyForce::temperatureCallback,
          this);
    }

  virtual void advance(double dt) override
  {
    // TODO M.Efe Tiryaki : density might be variable in future
    calculateFluidDensity();
    force_ =  volume_ * fluidDensity_ *- 1 * this->link_->getGravity();
    torque_ = (this->link_->getPositionWorldtoBase()- this->link_->getCoMPositionWorldtoBase()).cross(force_);
  }

protected:

  /*! \~english

   */
  void calculateFluidDensity()
  {
    // TODO : For temperature dependent fluids
    //fluidDensity_ = 960;
  }


  void updateDensity(){
    //https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4909168/pdf/jresv97n3p335_a1b.pdf
    fluidDensity_ = 999.85308 + 6.32693*pow(10,-2)*temperature_
      - 8.523829*pow(10,-3) *temperature_*temperature_
      + 6.943248*pow(10,-5) *temperature_*temperature_*temperature_
      - 3.821216*pow(10,-7) *temperature_*temperature_*temperature_*temperature_;
    // std::cout << "Density : " << fluidDensity_ << std::endl;
  }

  // CALLBACK
  virtual void temperatureCallback(const std_msgs::Float64 &msg) {
    temperature_ = msg.data;
    updateDensity();
  }

 protected:
  double temperature_;
  double volume_;
  double fluidDensity_;




  ros::Subscriber temperatureSubscriber_;
};
}
}
