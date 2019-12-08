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
        dragModel_(1)
  {
    this->name_ = "drag";
  }


  virtual ~DragWrench()
  {
  }


  virtual void readParameters() override
  {
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_D", c_D_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_L", c_L_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_P", c_P_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_R_x", c_R_x_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/C_R_x", c_R_z_);
    paramRead(this->nodeHandle_, "/physics/fluid/fluid_density", density_);
    paramRead(this->nodeHandle_, "/physics/fluid/fluid_viscosity", viscosity_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag_position", origin_);
    paramRead(this->nodeHandle_, "/physics/fluid/drag/drag_model", dragModel_);


  }

  virtual void advance(double dt) override
  {
    {
      calculateDragForceCoefficient();
      double linearSpeed = this->link_->getLinearVelocityOfBaseInBaseFrame().norm();
      Eigen::Vector3d force = Eigen::Vector3d::Zero();
      Eigen::Vector3d torque = Eigen::Vector3d::Zero();
      if (linearSpeed != 0) {
        auto velocityDirection = this->link_->getLinearVelocityOfBaseInBaseFrame().normalized();

        Eigen::Vector3d headingDirection =  Eigen::Vector3d::UnitZ();
        Eigen::Vector3d lateralDirection =  (velocityDirection.cross(headingDirection).
                                        cross(velocityDirection)).normalized();

        /*
        std::cout << "linearSpeed : " << linearSpeed << std::endl;
        std::cout << "angleOfAttack : " << asin((velocityDirection.cross(headingDirection)).norm()) << std::endl;
        std::cout << "v : " << velocityDirection.transpose() << std::endl;
        std::cout << "l : " << lateralDirection.transpose() << std::endl;
        std::cout << "_____________________________ " << std::endl;
        */

        if(dragModel_==1){
          force = -1 * c_D_(0) * linearSpeed * velocityDirection;
        }else if(dragModel_==2){
          force = -1 * c_D_(1) * linearSpeed * linearSpeed * velocityDirection;
          force += -1 * C_L_ * linearSpeed *  lateralDirection;
        }

        //torque = -1 * C_P_ * linearSpeed * zDirection.cross(velocityDirection);
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

    C_D_ = c_D_(0);

    C_L_ = c_L_(0);

    //C_P_ = c_P_(0) * incidenceAngle_ * incidenceAngle_ + c_P_(1) * incidenceAngle_ + c_P_(2);

  }

  void calculateDragTorqueCoefficient()
  {

    C_R_x_ = c_R_x_(0);

    C_R_z_ = c_R_z_(0);
  }


 protected:

  double density_;
  double viscosity_;
  Eigen::VectorXd c_D_;
  Eigen::VectorXd c_L_;
  Eigen::VectorXd c_P_;
  Eigen::VectorXd c_R_x_;
  Eigen::VectorXd c_R_z_;

  double C_D_;
  double C_L_;
  double C_P_;
  double C_R_x_;
  double C_R_z_;

  double incidenceAngle_;
  double rotationAngle_;

  int dragModel_;
};

}  // namespace wrench
}  // namespace gazebo
