/*
 File name: GazeboWorldPluginBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 29.01.2019
 Date last modified:  15.04.2019
 */
#pragma once

// c++
#include <mutex>
#include <string>
#include <condition_variable>
#include <vector>
#include <math.h>

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/Element.hh>
// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>

// urdf
#include <urdf/model.h>

#include "ros_node_utils/RosNodeBase.hpp"
#include "ros_node_utils/ros_node_utils.hpp"
#include "ros_gazebo_utils/wrench_modules/GlobalWrenchModuleBase.hpp"

using namespace ros_node_utils;

namespace gazebo {

/*! \~english
 This class is a Gazebo plugin base class
 */
class GazeboWorldPluginBase : public WorldPlugin, public ros_node_utils::RosNodeBase
{
 public:
  /*! \~english
   Constructor
   */
  GazeboWorldPluginBase()
      : WorldPlugin(),
        ros_node_utils::RosNodeBase()
  {

  }
  ;

  /*! \~english
   Destructor
   */
  virtual ~GazeboWorldPluginBase()
  {

  }
  ;

  /*! \~english
   Implements Gazebo virtual load function.
   */
  virtual void Load(physics::WorldPtr world, sdf::ElementPtr sdf)
  {

    nodeHandle_ = new ros::NodeHandle("~");

    // Model
    this->world_ = world;

    create();

    readParameters(sdf);

    initialize();
    // initialize ROS pub/sub/services
    initializeSubscribers();
    initializePublishers();
    initializeServices();

    // reset simulation variables
    Reset();
  }
  ;

  /*! \~english
   * Overrides Gazebo init function.
   */
  virtual void Init()
  {

  }
  ;

  /*! \~english
   * Overrides Gazebo reset function.
   */
  //
  virtual void Reset()
  {

  }
  ;
  /*! \~english
   */
  virtual void OnUpdate()
  {
    updateModels();
    readSimulation();
    writeSimulation();
    publish();
    publishTf();
  }
  ;

 protected:

  /*! \~english
   * Reads parameters from the parameter server and sdf element.
   */
  virtual void readParameters(sdf::ElementPtr sdf)
  {

  }
  ;

  /*! \~english
   * Read simulation state.
   */
  virtual void readSimulation()
  {
    for (int i = 0; models_.size(); i++) {
      auto pose = models_[i]->WorldPose();
      this->modelPositionsInWorldFrame_[i] = Eigen::Vector3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
      this->modelOrientationsInWorldFrame_[i] = Eigen::Quaterniond(pose.Rot().W(), pose.Rot().X(),
                                                                   pose.Rot().Y(), pose.Rot().Z());

      ignition::math::Vector3d linearVel = models_[i]->RelativeLinearVel();
      ignition::math::Vector3d angularVel = models_[i]->RelativeAngularVel();
      this->modelLinearVelocitiesInBaseFrame_[i] = Eigen::Vector3d(linearVel.X(), linearVel.Y(),
                                                                linearVel.Z());
      this->modelAngularVelocitiesInBaseFrame_[i] = Eigen::Vector3d(angularVel.X(), angularVel.Y(),
                                                                 angularVel.Z());

    }
  }
  ;
  /*! \~english
   * Writes simulation state.
   */
  virtual void writeSimulation()
  {
    for (auto wrench : globalWrenchModules_) {
      //std::cout << eff->getName() << std::endl;
      wrench->setPositions(this->modelPositionsInWorldFrame_);
      wrench->setOrientations(this->modelOrientationsInWorldFrame_);
      wrench->setLinearVelocities(this->modelLinearVelocitiesInBaseFrame_);
      wrench->setAngularVelocities(this->modelAngularVelocitiesInBaseFrame_);
      std::vector<Eigen::Vector3d> origins = wrench->getOrigins();
      std::vector<Eigen::Vector3d> forcesInWorldFrame = wrench->getForces();
      std::vector<Eigen::Vector3d> torquesInWorldFrame = wrench->getTorques();

      for (int i = 0; i < models_.size(); i++) {
        this->baseLinks_[i]->AddForceAtRelativePosition(
            ignition::math::Vector3<double>(forcesInWorldFrame[i][0], forcesInWorldFrame[i][1],
                          forcesInWorldFrame[i][2]),
                          ignition::math::Vector3<double>(origins[i][0], origins[i][1], origins[i][2]));
        this->baseLinks_[i]->AddRelativeTorque(
            ignition::math::Vector3<double>(torquesInWorldFrame[i][0], torquesInWorldFrame[i][1],
                          torquesInWorldFrame[i][2]));
      }

    }
  }
  ;

  /*! \~english
   * Writes simulation state.
   */
  virtual void updateModels()
  {
    if (world_->ModelCount() != models_.size()) {
      physics::Model_V models;
      models = world_->Models();
      baseLinks_.clear();
      modelPositionsInWorldFrame_.clear();
      modelOrientationsInWorldFrame_.clear();
      for (auto model : models) {
        models_.push_back(model);
        baseLinks_.push_back(physics::LinkPtr());
        physics::Link_V links = model->GetLinks();
        for (int i = 0; i < links.size(); i++) {
          if (links[i]->GetName().find("base_link") != std::string::npos) {
            baseLinks_.back() = links[i];
            CONFIRM("[GazeboWorldPluginBase] : " + model->GetName() + " contains base_link!");
            break;
          } else {
            ERROR("[GazeboWorldPluginBase] : " + model->GetName() + " doesn't contain base_link!");
          }
        }
        modelPositionsInWorldFrame_.push_back(Eigen::Vector3d::Zero());
        modelOrientationsInWorldFrame_.push_back(Eigen::Quaterniond());
        modelLinearVelocitiesInBaseFrame_.push_back(Eigen::Vector3d::Zero());
        modelAngularVelocitiesInBaseFrame_.push_back(Eigen::Vector3d::Zero());
      }
    }
  }
  ;

  /*! \~english
   * Publishes robot position, orientation and velocities.
   */
  virtual void publish()
  {

  }
  ;
  /*! \~english
   * Publish Transforms of the robot
   */
  virtual void publishTf()
  {

  }
  ;

  // Ensures gazebo methods are called sequentially
  std::recursive_mutex gazeboMutex_;
  // World update event.
  event::ConnectionPtr updateConnection_;

  // Model.
  physics::WorldPtr world_;
  std::vector<physics::ModelPtr> models_;
  std::vector<physics::LinkPtr> baseLinks_;
  std::vector<Eigen::Vector3d> modelPositionsInWorldFrame_;
  std::vector<Eigen::Quaterniond> modelOrientationsInWorldFrame_;
  std::vector<Eigen::Vector3d> modelLinearVelocitiesInBaseFrame_;
  std::vector<Eigen::Vector3d> modelAngularVelocitiesInBaseFrame_;

  // Wrench Module List
  std::vector<wrench::GlobalWrenchModuleBase*> globalWrenchModules_;

  // Gazebo time step.
  double gazeboTimeStep_ = 0.0;
  // Time step for publishing simulation state.
  double publishingTimeStep_ = 0.0;
  // Simulation time stamp taken at the start of the last updateCb() function call.
  common::Time lastStartUpdateSimTime_;
  // System time stamp taken at the start of the last updateCb() function call.
  std::chrono::time_point<std::chrono::steady_clock> lastStartUpdateSysTime_;
  // Current inter-update simulation time step.
  double updateSimTimeStep_ = 0.0;

};
}
