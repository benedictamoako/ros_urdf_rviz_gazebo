#ifndef RECONBOT_CONTROL_INTERFACE_PLUGIN_HPP
#define RECONBOT_CONTROL_INTERFACE_PLUGIN_HPP

#include <memory>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
  class RECONBOTControlInterfacePlugin : public ModelPlugin
  {
  public:
    RECONBOTControlInterfacePlugin() {}
    virtual ~RECONBOTControlInterfacePlugin() {}
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  private:
    void OnLeftHorizontalPropellerCmd(const std_msgs::msg::Float64::SharedPtr msg);
    void OnRightHorizontalPropellerCmd(const std_msgs::msg::Float64::SharedPtr msg);

    physics::ModelPtr model;
    physics::JointPtr left_horizontal_propeller_joint;
    physics::JointPtr right_horizontal_propeller_joint;

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_horizontal_propeller_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_horizontal_propeller_sub;
  };
}

#endif // RECONBOT_CONTROL_INTERFACE_PLUGIN_HPP