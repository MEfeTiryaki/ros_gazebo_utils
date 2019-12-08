/*
 File name: GlobalWrenchModuleBase.hpp
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

#include <Eigen/Dense>

#include "ros_node_utils/RosNodeModuleBase.hpp"

namespace gazebo {
namespace wrench {

class GlobalWrenchModuleBase : public ros_node_utils::RosNodeModuleBase
{
 public:
  GlobalWrenchModuleBase(ros::NodeHandle* nodeHandle)
      : ros_node_utils::RosNodeModuleBase(nodeHandle)
  {

  }
  ;

  virtual ~GlobalWrenchModuleBase()
  {
  }
  ;

  virtual void initialize() override
  {
    ros_node_utils::RosNodeModuleBase::initialize();
  }
  ;

  virtual void advance(double dt)
  {

  }

  virtual void setName(std::string name)
  {
    name_ = name;
  }
  ;

  virtual std::string getName()
  {
    return name_;
  }
  ;

  virtual void setOrigins(std::vector<Eigen::Vector3d> origin)
  {
    origins_ = origin;
  }
  ;
  virtual std::vector<Eigen::Vector3d> getOrigins()
  {
    return origins_;
  }
  ;

  virtual void setPositions(std::vector<Eigen::Vector3d> position)
  {
    positionsInWorldFrame_ = position;
  }
  ;

  virtual std::vector<Eigen::Vector3d> getPositions()
  {
    return positionsInWorldFrame_;
  }
  ;

  virtual void setOrientations(std::vector<Eigen::Quaterniond> orientation)
  {
    orientationsInWorldFrame_ = orientation;
  }
  ;

  virtual std::vector<Eigen::Quaterniond> getOrientations()
  {
    return orientationsInWorldFrame_;
  }
  ;

  virtual void setLinearVelocities(std::vector<Eigen::Vector3d> linearVelocity)
  {
    linearVelocitiesInBaseFrame_ = linearVelocity;
  }
  ;

  virtual std::vector<Eigen::Vector3d> getLinearVelocities()
  {
    return linearVelocitiesInBaseFrame_;
  }
  ;

  virtual void setAngularVelocities(std::vector<Eigen::Vector3d> angularVelocity)
  {
    angularVelocitiesInBaseFrame_ = angularVelocity;
  }
  ;
  virtual std::vector<Eigen::Vector3d> getAngularVelocities()
  {
    return angularVelocitiesInBaseFrame_;
  }
  ;

  virtual std::vector<Eigen::Vector3d> getForces()
  {
    return std::vector<Eigen::Vector3d>();
  }
  ;

  virtual std::vector<Eigen::Vector3d> getTorques()
  {
    return std::vector<Eigen::Vector3d>();
  }
  ;


 protected:

  std::string name_;

  std::vector<Eigen::Vector3d> origins_;
  std::vector<Eigen::Vector3d> forces_;
  std::vector<Eigen::Vector3d> torques_;

  std::vector<Eigen::Vector3d> positionsInWorldFrame_;
  std::vector<Eigen::Quaterniond> orientationsInWorldFrame_;

  std::vector<Eigen::Vector3d> linearVelocitiesInBaseFrame_;
  std::vector<Eigen::Vector3d> angularVelocitiesInBaseFrame_;

};
}  // namespace wrench
}  // namespace gazebo
