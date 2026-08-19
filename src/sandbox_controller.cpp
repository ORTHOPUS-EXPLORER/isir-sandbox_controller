#include "sandbox_controller/sandbox_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace sandbox
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  SandboxController::SandboxController() : controller_interface::ControllerInterface()
  {
  }

  controller_interface::InterfaceConfiguration SandboxController::command_interface_configuration()
      const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = robot_interface_->get_commands_names();
    return config;
  }

  controller_interface::InterfaceConfiguration SandboxController::state_interface_configuration()
      const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = robot_interface_->get_states_names();
    return config;
  }

  CallbackReturn SandboxController::on_init()
  {
    param_listener_ = std::make_shared<sandbox_controller::ParamListener>(get_node());

    return CallbackReturn::SUCCESS;
  }

  void SandboxController::loadParameters()
  {
    // These 3 parameters are mandatory, but others can be added in the same
    // fashion
    declare_and_get_parameters("robot_type", robot_type_, std::string("explorer_velocity"));
    declare_and_get_parameters("command_names", command_names_, std::vector<std::string>{});
    declare_and_get_parameters("base_frame", base_frame_, std::string("base_link"));

    params_ = param_listener_->get_params();
  }

  void SandboxController::setupSubscribers()
  {
    auto node = get_node();
    teleop_sub = get_node()->create_subscription<extender_msgs::msg::TeleopCommand>(
        params_.input_topic_name, 10,
        std::bind(&SandboxController::teleopCallback, this, std::placeholders::_1));
  }

  void SandboxController::setupPublishers()
  {
    auto node = get_node();
    op_vel_command_pub =
        node->create_publisher<geometry_msgs::msg::TwistStamped>("~/velocity_command", 10);
    op_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/ee_pose", 10);
    joint_pose_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/joint_pose", 10);
  }

  void SandboxController::activatePublishers()
  {
    op_vel_command_pub->on_activate();
    op_pose_pub->on_activate();
    joint_pose_pub->on_activate();
  }

  void SandboxController::deactivatePublishers()
  {
    op_vel_command_pub->on_deactivate();
    op_pose_pub->on_deactivate();
    joint_pose_pub->on_deactivate();
  }

  bool SandboxController::setupRobotInterface()
  {
    auto node = get_node();
    const auto &robot_description = get_robot_description();

    if (robot_description.empty())
    {
      RCLCPP_ERROR(node->get_logger(), "Missing robot_description");
      return false;
    }

    robot_interface_ = robot_interfaces::create_robot_component(robot_type_);
    if (!robot_interface_ || !robot_interface_->initKinematics(
                                 robot_description, node->get_parameter("tool_frame").as_string()))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot interface.");
      return false;
    }
    robot_interface_->set_commands_names(command_names_);

    return true;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SandboxController::
      on_configure(const rclcpp_lifecycle::State &)
  {
    auto node = get_node();

    loadParameters();
    setupSubscribers();
    setupPublishers();

    // Create robot interface
    if (!setupRobotInterface())
    {
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SandboxController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Assign the loaned command interfaces to the velocity interface
    robot_interface_->assign_loaned_command(command_interfaces_);
    robot_interface_->assign_loaned_state(state_interfaces_);

    activatePublishers();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SandboxController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_interface_->release_all_interfaces();
    deactivatePublishers();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type SandboxController::update(const rclcpp::Time &time,
                                                              const rclcpp::Duration &period)
  {
    robot_interface_->syncState();
    // Write above this, and put computed velocity inside latest_vel_cmd
    // latest_vel_cmd.linear = ...
    // latest_vel_cmd.angular = ...
    latest_vel_cmd.linear[0] = latest_teleop_cmd.linear.x;
    latest_vel_cmd.linear[1] = latest_teleop_cmd.linear.y;
    latest_vel_cmd.linear[2] = latest_teleop_cmd.linear.z;
    latest_vel_cmd.angular[0] = latest_teleop_cmd.angular.x;
    latest_vel_cmd.angular[1] = latest_teleop_cmd.angular.y;
    latest_vel_cmd.angular[2] = latest_teleop_cmd.angular.z;

    publishInfo();
    if (robot_interface_->setCommand(latest_vel_cmd))
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      RCLCPP_FATAL(get_node()->get_logger(), "Set command failed.");
      return controller_interface::return_type::ERROR;
    }
  }

  void SandboxController::publishInfo()
  {
    // Publish operational pose
    robot_interfaces::CartesianPosition temp_pose = robot_interface_->getCurrentEndEffectorPose();

    geometry_msgs::msg::PoseStamped pose_to_pub;

    pose_to_pub.header.stamp = get_node()->now();
    pose_to_pub.header.frame_id = base_frame_;

    pose_to_pub.pose.position.x = temp_pose.translation[0];
    pose_to_pub.pose.position.y = temp_pose.translation[1];
    pose_to_pub.pose.position.z = temp_pose.translation[2];
    pose_to_pub.pose.orientation.x = temp_pose.quaternion.x();
    pose_to_pub.pose.orientation.y = temp_pose.quaternion.y();
    pose_to_pub.pose.orientation.z = temp_pose.quaternion.z();
    pose_to_pub.pose.orientation.w = temp_pose.quaternion.w();

    op_pose_pub->publish(pose_to_pub);

    // Publish joint pose
    const auto &joint_positions = robot_interface_->getJointPositions();
    std_msgs::msg::Float64MultiArray joint_pose_to_pub;
    for (const double joint_position : joint_positions)
    {
      joint_pose_to_pub.data.push_back(joint_position);
    }
    joint_pose_pub->publish(joint_pose_to_pub);

    // Publish computed velocity
    geometry_msgs::msg::TwistStamped vel_to_pub;

    vel_to_pub.header.stamp = get_node()->now();
    vel_to_pub.header.frame_id = base_frame_;

    vel_to_pub.twist.linear.x = latest_vel_cmd.linear[0];
    vel_to_pub.twist.linear.y = latest_vel_cmd.linear[1];
    vel_to_pub.twist.linear.z = latest_vel_cmd.linear[2];
    vel_to_pub.twist.angular.x = latest_vel_cmd.angular[0];
    vel_to_pub.twist.angular.y = latest_vel_cmd.angular[1];
    vel_to_pub.twist.angular.z = latest_vel_cmd.angular[2];

    op_vel_command_pub->publish(vel_to_pub);
  }

  void SandboxController::teleopCallback(const extender_msgs::msg::TeleopCommand::SharedPtr msg)
  {
    latest_teleop_cmd = msg->twist;
  }
} // namespace sandbox

PLUGINLIB_EXPORT_CLASS(sandbox::SandboxController, controller_interface::ControllerInterface)
