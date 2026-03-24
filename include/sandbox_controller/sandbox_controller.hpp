#pragma once

// This includes are mandatory
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "extender_msgs/msg/teleop_command.hpp"
#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"

#include <sandbox_controller/sandbox_param_lib.hpp>
// Add all includes your project needs here

namespace sandbox
{
  /// @brief A ROS2 controller template to help create new ones
  class SandboxController : public controller_interface::ControllerInterface
  {
  public:
    SandboxController();
    virtual ~SandboxController() = default;

    // Configure command and state interfaces
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /// Main update loop called periodically by the controller manager
    controller_interface::return_type update(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

  private:
    /**
     * @brief Template function to declare and get parameters with default values.
     * @tparam T Type of the parameter.
     * @param name Parameter name.
     * @param variable Reference to store the parameter value.
     * @param default_value Default value if parameter is not set.
     */
    template <typename T>
    void declare_and_get_parameters(const std::string &name, T &variable, const T &default_value)
    {
      auto node = get_node();
      if (!node->has_parameter(name))
      {
        node->declare_parameter(name, default_value);
      }
      variable = node->get_parameter(name).get_value<T>();
    }

    void loadParameters();
    void setupSubscribers();
    void setupPublishers();
    bool setupRobotInterface();

    void activatePublishers();
    void deactivatePublishers();

    /// Callback to receive Twist commands from the teleop node
    void teleopCallback(const extender_msgs::msg::TeleopCommand::SharedPtr msg);
    void publishInfo();

    /// Generic component to interface with robot hardware
    std::string robot_type_{"explorer_velocity"};
    std::vector<std::string> command_names_;
    std::string base_frame_;
    std::unique_ptr<robot_interfaces::GenericComponent> robot_interface_;

    geometry_msgs::msg::Twist latest_teleop_cmd;
    robot_interfaces::CartesianVelocity latest_vel_cmd;

    // Subscribers
    rclcpp::Subscription<extender_msgs::msg::TeleopCommand>::SharedPtr teleop_sub;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        op_vel_command_pub;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr op_pose_pub;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        joint_pose_pub;

    // Param Listener
    std::shared_ptr<sandbox_controller::ParamListener> param_listener_;
    sandbox_controller::Params params_;
  };
} // namespace sandbox