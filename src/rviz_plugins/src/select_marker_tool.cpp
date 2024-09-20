#include "select_marker_tool.h"

/**
 * \file     select_marker_tool.cpp
 * \mainpage Implementation for tool that gives a feedback to an external ROS node about which marker has been selected
 * \author   Fraunhofer Italia (2022)
 * 
 * \note     Useful resources:
 *           The existing tools in particular the ones for selection
 *           https://github.com/ros-visualization/rviz/tree/noetic-devel/src/rviz/default_plugin/tools
 *           The existing tools for markers and the corresponding selection handler:
 *           https://github.com/ros-visualization/rviz/tree/noetic-devel/src/rviz/default_plugin/markers
 *           https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/markers/marker_selection_handler.cpp
 *           The official visualization tutorials package:
 *           https://github.com/ros-visualization/visualization_tutorials/tree/noetic-devel/rviz_plugin_tutorials
 *           The corresponding official tutorials:
 *           http://docs.ros.org/en/kinetic/api/rviz_plugin_tutorials/html/
 *           The selected_points_publisher package:
 *           https://github.com/tu-rbo/turbo-ros-pkg/tree/master/selected_points_publisher
*/

#include <cstdint>
#include <string>

#include "concert_msgs/msg/selected_markers.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "geometry_msgs/msg/pose.hpp"


#include <fstream>
#include <iostream>

namespace rviz_plugins {

  SelectMarkerTool::SelectMarkerTool(std::string const& selected_markers_topic)
    : rviz_default_plugins::tools::SelectionTool(),
      is_selecting_{false},
      selected_markers_topic_{selected_markers_topic},
      qos_profile_{5} {
    return;
  }

  SelectMarkerTool::~SelectMarkerTool() {
    return;
  }

  void SelectMarkerTool::onInitialize() {
    raw_node_ =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
    initPublisher_();
    return;
  }

  void SelectMarkerTool::activate() {
    setStatus( "Select a mesh marker to interact with it." );
    context_->getSelectionManager()->setTextureSize(512);
    is_selecting_ = false;
    return;
  }

  int SelectMarkerTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    int const flags = rviz_default_plugins::tools::SelectionTool::SelectionTool::processMouseEvent(event);

    if (event.alt()) {
      is_selecting_ = false;
    } else if (event.rightDown()) {
      is_selecting_ = true;
    }

    if (is_selecting_ && event.rightUp()) {
      auto sel_manager = context_->getSelectionManager();
      // std::string const frame_id = context_->getFixedFrame().toStdString();
      rviz_common::properties::PropertyTreeModel* model = sel_manager->getPropertyModel();
      int const num_points = model->rowCount();
      
      selected_markers_ = concert_msgs::msg::SelectedMarkers();
      for (int i = 0; i < num_points; ++i) {
        if (model->hasIndex(i, 0)) {
          QModelIndex const child_index = model->index(i, 0);
          rviz_common::properties::Property* child = model->getProp(child_index);
          selected_markers_.names.push_back(child->getNameStd());
        }
      }

      // extract position of selected marker geometry and publish it on a topic
      for(auto i:selected_markers_.names) {

        std::string position_path = i;
        std::string delimiter = "//";

        size_t pos = 0;
        std::string token;

        while ((pos = position_path.find(delimiter)) != std::string::npos) {
          token = position_path.substr(0, pos);
          position_path.erase(0, pos + delimiter.length());
        }

        delimiter = ".";

        while ((pos = position_path.find(delimiter)) != std::string::npos) {
          position_path = position_path.substr(0, pos);
        }
        
        std::fstream newfile;
        newfile.open(position_path+".txt",std::ios::in);

        if (newfile.is_open()){

          selected_markers_pub_->publish(selected_markers_);
      
          std::string tp;
          //std_msgs::msg::String msg;
          geometry_msgs::msg::Pose msg;
          std::vector <std::string> pos_rot_array;
          while(getline(newfile, tp, ';')){
            pos_rot_array.push_back(tp);     
            //msg.data = tp;
            //position_pub_->publish(msg);

          }
          msg.position.x = std::stof(pos_rot_array[0]);
          msg.position.y = std::stof(pos_rot_array[1]);
          msg.position.z = std::stof(pos_rot_array[2]);
          msg.orientation.x = std::stof(pos_rot_array[4]);
          msg.orientation.y = std::stof(pos_rot_array[5]);
          msg.orientation.z = std::stof(pos_rot_array[6]);
          msg.orientation.w = std::stof(pos_rot_array[3]);
          position_pub_ -> publish(msg);
          //msg.data = pos_rot_array[0];
          //position_pub_ -> publish(msg);
          //msg.data = pos_rot_array[1];
          //rotation_pub_ -> publish(msg);
          newfile.close();
        }
      } 
    }
    selected_markers_pub_->publish(selected_markers_);
    return flags;
  }
  
  void SelectMarkerTool::initPublisher_() {
    selected_markers_pub_ = raw_node_->create_publisher<concert_msgs::msg::SelectedMarkers>(
      selected_markers_topic_, qos_profile_);

    position_pub_ = raw_node_->create_publisher<geometry_msgs::msg::Pose>(
      "/geometry_position", qos_profile_);
    
    return;
  }
} // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::SelectMarkerTool, rviz_common::Tool)
