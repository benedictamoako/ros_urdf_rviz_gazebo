#include "reconbot_control_plugin/reconbot_control_interface_plugin.hpp"

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(RECONBOTControlInterfacePlugin)

  void RECONBOTControlInterfacePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->model = _model;

    // Get joint names from SDF
    std::string left_horizontal_propeller_joint_name = _sdf->Get<std::string>("left_horizontal_propeller_joint");
    std::string right_horizontal_propeller_joint_name = _sdf->Get<std::string>("right_horizontal_propeller_joint");

    this->left_horizontal_propeller_joint = this->model->GetJoint(left_horizontal_propeller_joint_name);
    this->right_horizontal_propeller_joint = this->model->GetJoint(right_horizontal_propeller_joint_name);

    if (!this->left_horizontal_propeller_joint || !this->right_horizontal_propeller_joint)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RECONBOTControlInterfacePlugin"), "Could not find propeller joints");
      return;
    }

    // Initialize ROS 2 node
    this->node = std::make_shared<rclcpp::Node>("reconbot_control_interface_plugin");

    // ROS 2 subscribers for propeller commands
    this->left_horizontal_propeller_sub = this->node->create_subscription<std_msgs::msg::Float64>(
      "/left_horizontal_propeller_cmd", 1, std::bind(&RECONBOTControlInterfacePlugin::OnLeftHorizontalPropellerCmd, this, std::placeholders::_1));
    this->right_horizontal_propeller_sub = this->node->create_subscription<std_msgs::msg::Float64>(
      "/right_horizontal_propeller_cmd", 1, std::bind(&RECONBOTControlInterfacePlugin::OnRighthorizontalPropellerCmd, this, std::placeholders::_1));

    // Start a thread to spin the node
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(this->node);
    std::thread([exec]() mutable { exec.spin(); }).detach();
  }

  void RECONBOTControlInterfacePlugin::OnLeftHorizontalPropellerCmd(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->left_horizontal_propeller_joint->SetVelocity(0, msg->data);
  }

  void RECONBOTControlInterfacePlugin::OnRighthorizontalPropellerCmd(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->right_horizontal_propeller_joint->SetVelocity(0, msg->data);
  }
}
