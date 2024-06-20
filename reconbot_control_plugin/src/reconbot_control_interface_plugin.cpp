#include "/home/benedict/ros2_ws/src/reconbot_control_plugin/include/reconbot_control_plugin/reconbot_control_interface_plugin.hpp"

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
    this->node = gazebo_ros::Node::Get(_sdf);

    // ROS 2 subscribers for propeller commands
    this->cmd_vel_sub = this->node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1, std::bind(&RECONBOTControlInterfacePlugin::OnCmdVel, this, std::placeholders::_1));
    this->left_horizontal_propeller_sub = this->node->create_subscription<std_msgs::msg::Float64>(
      "/left_horizontal_propeller_cmd", 1, std::bind(&RECONBOTControlInterfacePlugin::OnLeftHorizontalPropellerCmd, this, std::placeholders::_1));
    this->right_horizontal_propeller_sub = this->node->create_subscription<std_msgs::msg::Float64>(
      "/right_horizontal_propeller_cmd", 1, std::bind(&RECONBOTControlInterfacePlugin::OnRightHorizontalPropellerCmd, this, std::placeholders::_1));

    // Register for simulation update
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RECONBOTControlInterfacePlugin::UpdateState, this));
  }

  void RECONBOTControlInterfacePlugin::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Convert cmd_vel to individual propeller velocities
    double linear = msg->linear.x;
    double angular = msg->angular.z;

    double left_velocity = linear - angular;
    double right_velocity = linear + angular;

    this->left_horizontal_propeller_joint->SetVelocity(0, left_velocity);
    this->right_horizontal_propeller_joint->SetVelocity(0, right_velocity);
  }

  void RECONBOTControlInterfacePlugin::OnLeftHorizontalPropellerCmd(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->left_horizontal_propeller_joint->SetVelocity(0, msg->data);
  }

  void RECONBOTControlInterfacePlugin::OnRightHorizontalPropellerCmd(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->right_horizontal_propeller_joint->SetVelocity(0, msg->data);
  }

  void RECONBOTControlInterfacePlugin::UpdateState()
  {
    // Publish robot state to /tf (implementation needed)
  }
}
