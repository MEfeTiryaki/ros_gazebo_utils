/*
 File name: DragWrench.hpp
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

#include "ros_gazebo_utils/wrench_modules/WrenchModuleBase.hpp"

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

namespace gazebo {
namespace wrench {

class DragWrench : public WrenchModuleBase
{
 public:
  DragWrench(ros::NodeHandle* nodeHandle, wrench::WrenchLink* link)
      : WrenchModuleBase(nodeHandle, link),
        C_D_(0.0),
        C_L_(0.0),
        C_P_(0.0),
        C_R_x_(0.0),
        C_R_z_(0.0),
        viscosity_(-1.0),
        density_(-1.0),
        incidenceAngle_(0.0),
        rotationAngle_(0.0),
        dragModel_(1),
        temperature_(36)
  {
    this->name_ = "drag";
  }


  virtual ~DragWrench()
  {
  }


  virtual void readParameters() override
  {
    paramRead(this->nodeHandle_, "/physics/shape/radius", radius_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_D", c_D_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_L", c_L_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_P", c_P_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_R_x", c_R_x_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_R_x", c_R_z_);
    paramRead(this->nodeHandle_, "/physics/fluid/fluid_density", density_);
    paramRead(this->nodeHandle_, "/physics/fluid/fluid_viscosity", viscosity_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag_position", origin_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/drag_model", dragModel_);
    paramRead(this->nodeHandle_, "/physics/temperature", temperature_);

    updateViscosity();

    fluidVelocity_ = Eigen::Vector3d::Zero() ;
  }

  void initializeSubscribers(){
    temperatureSubscriber_ = getNodeHandle()->subscribe(
        "/gazebo/temperature", 1, &DragWrench::temperatureCallback,
        this);
    fluidVelocitySubscriber_ = getNodeHandle()->subscribe(
        "/gazebo/fluid_velocity", 1, &DragWrench::fluidVelocityCallback,
        this);
  }


  virtual void advance(double dt) override
  {
    {
      calculateDragForceCoefficient();
      double relativeLinearSpeed = ( this->link_->getLinearVelocityOfBaseInBaseFrame()-fluidVelocity_).norm();
      Eigen::Vector3d force = Eigen::Vector3d::Zero();
      Eigen::Vector3d torque = Eigen::Vector3d::Zero();
      if (relativeLinearSpeed != 0) {
        auto relativeVelocityDirection = (this->link_->getLinearVelocityOfBaseInBaseFrame()-fluidVelocity_).normalized();

        Eigen::Vector3d headingDirection =  Eigen::Vector3d::UnitZ();
        Eigen::Vector3d lateralDirection =  (relativeVelocityDirection.cross(headingDirection).
                                        cross(relativeVelocityDirection)).normalized();

        /*
        std::cout << "relativeLinearSpeed : " << relativeLinearSpeed << std::endl;
        std::cout << "angleOfAttack : " << asin((velocityDirection.cross(headingDirection)).norm()) << std::endl;
        std::cout << "v : " << velocityDirection.transpose() << std::endl;
        std::cout << "l : " << lateralDirection.transpose() << std::endl;
        std::cout << "_____________________________ " << std::endl;
        */

        double Re = relativeLinearSpeed * 2 *radius_* fluidDensity_ / viscosity_;
        double Area = M_PI * radius_* radius_;
        C_D_ = 24.0/Re + 6.0/(1.0+sqrt(Re)) + 0.4;

        force = -1 * C_D_/2 * fluidDensity_* Area * relativeLinearSpeed* relativeLinearSpeed * relativeVelocityDirection;

        // force = -1 * C_D_ * relativeLinearSpeed * relativeVelocityDirection;

        //torque = -1 * C_P_ * relativeLinearSpeed * zDirection.cross(relativeVelocityDirection);
      }

      force_ = force;
      torque_ = torque;
    }
    {
      calculateDragTorqueCoefficient();
      double angularSpeed = this->link_->getAngularVelocityOfBaseInBaseFrame().norm();
      if (angularSpeed != 0) {
        auto angularVelocityDirection = this->link_->getAngularVelocityOfBaseInBaseFrame().normalized();

        torque_ += -1 * C_R_x_ *angularSpeed * angularSpeed * angularVelocityDirection[0]
            * Eigen::Vector3d::UnitX();
        torque_ += -1 * C_R_x_ * angularSpeed * angularSpeed * angularVelocityDirection[1]
            * Eigen::Vector3d::UnitY();
        torque_ += -1 * C_R_z_ * angularSpeed * angularSpeed * angularVelocityDirection[2]
            * Eigen::Vector3d::UnitZ();
      }
    }

    force_ = this->link_->getOrientationWorldtoBase() * force_;
    torque_ = this->link_->getOrientationWorldtoBase() * torque_ ;

  }

protected:
  void calculateDragForceCoefficient()
  {
    //reynold_ = density_*
    Eigen::Vector3d zDirection = this->link_->getOrientationWorldtoBase()
        * Eigen::Vector3d::UnitZ();
    incidenceAngle_ = 0.0;
    if (this->link_->getLinearVelocityOfBaseInBaseFrame().norm() != 0) {
      incidenceAngle_ = acos(
          (this->link_->getLinearVelocityOfBaseInBaseFrame().normalized()).dot(zDirection));
    }


    //C_P_ = c_P_(0) * incidenceAngle_ * incidenceAngle_ + c_P_(1) * incidenceAngle_ + c_P_(2);

  }

  void calculateDragTorqueCoefficient()
  {

    C_R_x_ = c_R_x_(0);

    C_R_z_ = c_R_z_(0);
  }

 void updateViscosity(){
   // https://www.engineersedge.com/physics/water__density_viscosity_specific_weight_13146.htm
   viscosity_ =  2.414e-5* pow (10,247.8/((temperature_+273)-140));

   fluidDensity_ = 999.85308 + 6.32693*pow(10,-2)*temperature_
     - 8.523829*pow(10,-3) *temperature_*temperature_
     + 6.943248*pow(10,-5) *temperature_*temperature_*temperature_
     - 3.821216*pow(10,-7) *temperature_*temperature_*temperature_*temperature_;
   // C_D_ = 6 * M_PI * radius_* mu ;
   C_L_ = c_L_(0);
   // std::cout << "Drag : "<< C_D_ << std::endl;
 }

 // CALLBACK
 virtual void temperatureCallback(const std_msgs::Float64 &msg) {
   temperature_ = msg.data;
   updateViscosity();
 }

 virtual void fluidVelocityCallback(const std_msgs::Float64MultiArray &msg) {
   fluidVelocity_ = Eigen::Vector3d(msg.data[0],msg.data[1],msg.data[2]);
 }

 protected:

  double temperature_;
  double density_;
  double fluidDensity_;
  double viscosity_;
  Eigen::VectorXd c_D_;
  Eigen::VectorXd c_L_;
  Eigen::VectorXd c_P_;
  Eigen::VectorXd c_R_x_;
  Eigen::VectorXd c_R_z_;

  double radius_;
  double C_D_;
  double C_L_;
  double C_P_;
  double C_R_x_;
  double C_R_z_;

  double incidenceAngle_;
  double rotationAngle_;

  int dragModel_;

  Eigen::Vector3d fluidVelocity_;


  ros::Subscriber temperatureSubscriber_;
  ros::Subscriber fluidVelocitySubscriber_;
};

}  // namespace wrench
}  // namespace gazebo
