/*
 File name: WrenchVisualizer.hpp
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

#include "ros_node_utils/RosNodeModuleBase.hpp"
#include "ros_gazebo_utils/wrench_modules/WrenchModuleBase.hpp"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/MarkerArray.h"

namespace gazebo {
namespace wrench {

class WrenchVisualizer : public ros_node_utils::RosNodeModuleBase
{
 public:
  WrenchVisualizer(ros::NodeHandle* nodeHandle)
      : ros_node_utils::RosNodeModuleBase(nodeHandle)
  {

  }
  ;

  virtual ~WrenchVisualizer()
  {
  }
  ;

  virtual void initialize() override
  {
    ros_node_utils::RosNodeModuleBase::initialize();
  }
  ;

  void addWrench(WrenchModuleBase* module, std::string linkName, Eigen::Vector4d forceColorEigen,
                 Eigen::Vector4d torqueColorEigen, double forceScale, double torqueScale)
  {
    std_msgs::ColorRGBA forceColor;
    forceColor.r = forceColorEigen[0];
    forceColor.g = forceColorEigen[1];
    forceColor.b = forceColorEigen[2];
    forceColor.a = forceColorEigen[3];

    std_msgs::ColorRGBA torqueColor;
    torqueColor.r = torqueColorEigen[0];
    torqueColor.g = torqueColorEigen[1];
    torqueColor.b = torqueColorEigen[2];
    torqueColor.a = torqueColorEigen[3];

    wrenchModules_.push_back(module);
    linkNames_.push_back(linkName);
    forceColors_.push_back(forceColor);
    torqueColors_.push_back(torqueColor);
    forceScales_.push_back(forceScale);
    torqueScales_.push_back(torqueScale);
  }
  ;

  void addWrench(WrenchModuleBase* module, std::string linkName, Eigen::Vector4d forceColor,
                 Eigen::Vector4d torqueColor, double scale)
  {
    addWrench(module, linkName, forceColor, torqueColor, scale, scale);
  }
  ;

  void addWrench(WrenchModuleBase* module, std::string linkName, Eigen::Vector4d forceColor,
                 Eigen::Vector4d torqueColor)
  {
    addWrench(module, linkName, forceColor, torqueColor, 1, 1);
  }
  ;

  virtual void calculateMarkers(visualization_msgs::MarkerArray& markerArray)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(2);

    for (int id = 0; id < wrenchModules_.size(); id++) {
      Eigen::Vector3d origin = wrenchModules_[id]->getOrigin();
      Eigen::Quaterniond orientationWorldtoBase = wrenchModules_[id]->getOrientation();

      Eigen::Vector3d forceInBaseFrame = orientationWorldtoBase.inverse()
          * wrenchModules_[id]->getForceInWorldFrame();
      Eigen::Vector3d torqueInBaseFrame = orientationWorldtoBase.inverse()
          * wrenchModules_[id]->getTorqueInWorldFrame();


      // FORCE
      marker.header.frame_id = linkNames_[id];
      marker.id = 2 * id;
      marker.points[0].x = origin[0];
      marker.points[0].y = origin[1];
      marker.points[0].z = origin[2];
      marker.points[1].x = origin[0] + forceScales_[id] * forceInBaseFrame[0];
      marker.points[1].y = origin[1] + forceScales_[id] * forceInBaseFrame[1];
      marker.points[1].z = origin[2] + forceScales_[id] * forceInBaseFrame[2];
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1 * forceScales_[id] * forceInBaseFrame.norm();
      marker.scale.y = 0.2 * forceScales_[id] * forceInBaseFrame.norm();
      marker.scale.z = 0.0;
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color = forceColors_[id];
      markerArray.markers.push_back(marker);

      // TORQUE
      marker.id = 2 * id + 1;
      marker.points[0].x = origin[0];
      marker.points[0].y = origin[1];
      marker.points[0].z = origin[2];
      marker.points[1].x = origin[0] + torqueScales_[id] * torqueInBaseFrame[0];
      marker.points[1].y = origin[1] + torqueScales_[id] * torqueInBaseFrame[1];
      marker.points[1].z = origin[2] + torqueScales_[id] * torqueInBaseFrame[2];
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1 * torqueScales_[id] * torqueInBaseFrame.norm();
      marker.scale.y = 0.2 * torqueScales_[id] * torqueInBaseFrame.norm();
      marker.scale.z = 0.0;
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color = torqueColors_[id];

      markerArray.markers.push_back(marker);
    }
  }
  ;

 protected:

  std::vector<WrenchModuleBase*> wrenchModules_;
  std::vector<std::string> linkNames_;
  std::vector<std_msgs::ColorRGBA> forceColors_;
  std::vector<std_msgs::ColorRGBA> torqueColors_;
  std::vector<double> forceScales_;
  std::vector<double> torqueScales_;

};
}  // namespace wrench
}  // namespace gazebo
