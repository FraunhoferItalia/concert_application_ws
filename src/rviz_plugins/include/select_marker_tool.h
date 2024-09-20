#ifndef SELECT_MARKER_TOOL_H
#define SELECT_MARKER_TOOL_H

/**
 * \file     select_marker_tool.h
 * \mainpage Rviz tool for giving a feedback to an external ROS node about which marker has been selected
 * \author   Fraunhofer Italia (2022)
*/

#include <string>
#include <rclcpp/rclcpp.hpp>

#include <concert_msgs/msg/selected_markers.hpp>

#include "rviz_default_plugins/tools/select/selection_tool.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace rviz_plugins {

  /**\class SelectMarkerTool
   * \brief Rviz tool that allows you to give feedback to an external ROS node about which marker has been selected
   *        By default this topic is given by the default argument in the constructor ("/selected_markers").
  */
  class SelectMarkerTool: public rviz_default_plugins::tools::SelectionTool {
    Q_OBJECT
    public:
      /**\fn        SelectMarkerTool
       * \brief     (Default) constructor
       *
       * \param[in] selected_markers_topic   The default topic where the markers should be read from
      */
      SelectMarkerTool(std::string const& selected_markers_topic = "/selected_markers");

      /**\fn    ~SelectMarkerTool
       * \brief Destructor
      */
      virtual ~SelectMarkerTool();

      /**\fn      onInitialize
       * \brief   Function that is called when initializing the plug-in
       * \warning Has to exist as it overrides initialization routine of SelectionTool
      */
      void onInitialize() override;

      /**\fn    onInitialize
       * \brief Function that is called when the tool is activated: Mainly changing the internal variables
      */
      void activate() override;

      /**\fn    processMouseEvent
       * \brief Function that is called when the tool is activated: Mainly changing the internal variables
       * 
       * \param[in] event   Mouse event to be processed
       * \return    Flags to be evaluated (Not overriden: Uses the ones of SelectionTool base class)
      */
      int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
    protected:
      /**\fn    updatePublisher_
       * \brief Function for updating the internal publisher and its topic
      */
      void initPublisher_();

      bool is_selecting_;
      std::string selected_markers_topic_;

      rclcpp::Node::SharedPtr raw_node_;
      rclcpp::QoS qos_profile_;

      rclcpp::Publisher<concert_msgs::msg::SelectedMarkers>::SharedPtr selected_markers_pub_;
      //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr position_pub_;
      rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr position_pub_;

      concert_msgs::msg::SelectedMarkers selected_markers_;
  };

} // end namespace rviz_plugins

#endif // SELECT_MARKER_TOOL_H
