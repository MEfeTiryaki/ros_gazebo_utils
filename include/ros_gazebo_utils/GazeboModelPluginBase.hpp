/*
 File name: GazeboModelPluginBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 29.01.2019
 Date last modified: 15.04.2019
 */
#pragma once

// c++
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <math.h>
// required for std::this_thread
#include <thread>

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
#include "ros_gazebo_utils/wrench_modules/WrenchModuleBase.hpp"
#include "ros_gazebo_utils/wrench_modules/GravityForce.hpp"

namespace gazebo {

/*! \~english
 This class is a Gazebo plugin base class
 */
class GazeboModelPluginBase : public ModelPlugin, public ros_node_utils::RosNodeBase
{
 public:
  /*! \~english
   Constructor
   */
  GazeboModelPluginBase()
  {

  }
  ;

  /*! \~english
   Destructor
   */
  virtual ~GazeboModelPluginBase()
  {

  }
  ;

  /*! \~english
   Implements Gazebo virtual load function.
   */
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf /*_sdf*/)
  {

    nodeHandle_ = new ros::NodeHandle("~");

    // Model
    this->model_ = model;

    create();

    readParameters(sdf);

    // initialize joint structure
    initializeJointStructure();
    initializeLinkStructure();

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
   */
  virtual void initializeJointStructure()
  {

  }
  ;
  /*! \~english
   */
  virtual void initializeLinkStructure()
  {

  }
  ;
  /*! \~english
   * Read simulation state.
   */
  virtual void readSimulation()
  {

  }
  ;
  /*! \~english
   * Writes simulation state.
   */
  virtual void writeSimulation()
  {

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

  // Model.
  physics::ModelPtr model_;
  // World update event.
  event::ConnectionPtr updateConnection_;

  // Name of the robot.
  std::string robotName_;
  // ROS robot description parameter name.
  std::string robotDescriptionParamName_;
  // Robot base link.
  std::string robotBaseLink_;
  // ROS robot description URDF string.
  std::string robotDescriptionUrdfString_;
  // ROS robot description URDF model.
  urdf::Model robotDescriptionUrdfModel_;

  // Pulishers
  ros::Publisher posePublisher_;
  ros::Publisher velocityPublisher_;
  ros::Publisher markerPublisher_;

  // Robot links
  physics::LinkPtr baseLink_;
  Eigen::Vector3d positionWorldToBase_;
  Eigen::Vector3d positionWorldToCoM_;
  Eigen::Quaterniond orientationWorldToBase_;
  Eigen::Vector3d linearVelocityOfBaseInBaseFrame_;
  Eigen::Vector3d angularVelocityOfBaseInBaseFrame_;

  // gravitational force
  wrench::GravityForce* gravityForceModule_;
  // Wrench Module List
  std::vector<wrench::WrenchModuleBase*> wrenchModules_;

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
