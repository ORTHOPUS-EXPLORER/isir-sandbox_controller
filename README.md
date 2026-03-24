# Sandbox Controller

A ROS 2 controller that receives teleoperation commands and manages robot motion control. This controller interfaces with robot hardware through a generic robot component interface, enabling flexible control across different robot platforms.

## Overview

The Sandbox Controller is a lifecycle-aware ROS 2 controller that:
- Listens to teleop commands from one or more input devices
- Processes commands into robot-compatible velocity directives
- Publishes the robot's operational state (end-effector pose, joint angles, velocity commands)
- Abstracts robot-specific hardware through a generic component interface

The controller operates as a pluginlib plugin within the ROS 2 controller manager framework.

## Topics

### Subscriptions (Topics Listened To)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `~/input_topic_name` | `extender_msgs/TeleopCommand` | Teleop commands from input devices (joystick, tablet, etc.). The topic name is configurable via the `input_topic_name` parameter. |

### Publications (Topics Published)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `~/velocity_command` | `geometry_msgs/TwistStamped` | Computed Cartesian velocity command for the end-effector. Includes linear and angular velocity vectors with a timestamp and base frame reference. |
| `~/ee_pose` | `geometry_msgs/PoseStamped` | Current pose (position + orientation) of the end-effector in the base frame. Updated every control cycle. |
| `~/joint_pose` | `std_msgs/Float64MultiArray` | Current joint angles of all commanded robot joints. Published as an array of float64 values. |

**Note:** The `~` prefix means these topics are relative to the controller's namespace (typically determined by the controller configuration).

## Parameters

### Required Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_type` | string | `"explorer_velocity"` | Type/variant of robot, used to instantiate the correct robot interface component |
| `command_names` | vector\<string\> | `[]` | List of joint/command interface names to command |
| `base_frame` | string | `"base_link"` | Base coordinate frame for state publications |
| `tool_frame` | string | From URDF | Tool/end-effector frame for kinematics |
| `input_topic_name` | string | `"/input"` | Topic name where teleop commands are received |

### Parameter Configuration

Parameters are typically configured in YAML files located in `bringup/config/`. Example:

```yaml
sandbox_controller:
  ros__parameters:
    robot_type: "explorer_velocity"
    command_names: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    base_frame: "base_link"
    tool_frame: "tool0"
    input_topic_name: "/explorer_user_interfaces/rqt_armcontrol/input_device_velocity"
```

## Control Lifecycle

The controller follows the ROS 2 lifecycle pattern:

1. **UNCONFIGURED** → `on_init()`
   - Subscribers are registered (i.e., starts listening to teleop input)

2. **INACTIVE** → `on_configure()`
   - Parameters are loaded
   - Publishers are created
   - Robot interface is initialized with kinematics data

3. **ACTIVE** → `on_activate()`
   - Publishers are activated
   - Command and state interfaces are assigned from the controller manager
   - `update()` loop begins executing

4. **DEACTIVATE** → `on_deactivate()`
   - Publishers are deactivated
   - Interfaces are released

## How to Modify

### 1. Change Input Topic Names

Edit the configuration file `bringup/config/explorer_params.yaml`:

```yaml
sandbox_controller:
  ros__parameters:
    input_topic_name: "/my_custom_topic"
```

Or pass it via launch arguments.

### 2. Add a New Subscriber

To listen to additional topics (e.g., safety commands, mode switches):

**In `include/sandbox_controller/sandbox_controller.hpp`:**

```cpp
rclcpp::Subscription<my_msgs::msg::MyMessage>::SharedPtr new_sub;
```

**In `src/sandbox_controller.cpp` → `setupSubscribers()`:**

```cpp
void SandboxController::setupSubscribers()
{
  auto node = get_node();
  
  // Existing teleop subscriber
  teleop_sub = node->create_subscription<extender_msgs::msg::TeleopCommand>(
      params_.input_topic_name, 10,
      std::bind(&SandboxController::teleopCallback, this, std::placeholders::_1));
  
  // New subscriber
  new_sub = node->create_subscription<my_msgs::msg::MyMessage>(
      "/new_topic", 10,
      std::bind(&SandboxController::newCallback, this, std::placeholders::_1));
}
```

**Add callback method:**

```cpp
void SandboxController::newCallback(const my_msgs::msg::MyMessage::SharedPtr msg)
{
  // Handle the message
}
```

### 3. Add a New Publisher

To publish additional state or debug information:

**In `include/sandbox_controller/sandbox_controller.hpp`:**

```cpp
rclcpp_lifecycle::LifecyclePublisher<my_msgs::msg::MyMessage>::SharedPtr new_pub;
```

**In `src/sandbox_controller.cpp` → `setupPublishers()`:**

```cpp
void SandboxController::setupPublishers()
{
  auto node = get_node();
  
  // Existing publishers
  op_vel_command_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("~/velocity_command", 10);
  // ...
  
  // New publisher
  new_pub = node->create_publisher<my_msgs::msg::MyMessage>("~/my_debug_output", 10);
}
```

**Activate/deactivate the publisher:**

In `activatePublishers()` and `deactivatePublishers()`:

```cpp
void SandboxController::activatePublishers()
{
  // ...
  new_pub->on_activate();
}

void SandboxController::deactivatePublishers()
{
  // ...
  new_pub->on_deactivate();
}
```

**Publish data** in the `update()` method or in `publishInfo()`:

```cpp
void SandboxController::publishInfo()
{
  // Existing publications...
  
  // New publication
  my_msgs::msg::MyMessage msg;
  // ... populate msg ...
  new_pub->publish(msg);
}
```

### 4. Modify Control Logic

The main control algorithm runs in the `update()` method. The current implementation:

1. Executes user-defined control logic (placeholder comments show where to add your logic)
2. Calls `publishInfo()` to broadcast state
3. Sends the computed velocity to the robot via `robot_interface_->setCommand()`

**Example modification:**

```cpp
controller_interface::return_type SandboxController::update(const rclcpp::Time &time,
                                                            const rclcpp::Duration &period)
{
  // Add your control algorithm here
  // Example: scale or filter the teleop command
  latest_vel_cmd.linear[0] = latest_teleop_cmd.linear.x * 0.5;  // Scale down X velocity
  latest_vel_cmd.linear[1] = latest_teleop_cmd.linear.y * 0.5;
  latest_vel_cmd.linear[2] = latest_teleop_cmd.linear.z * 0.5;
  
  latest_vel_cmd.angular[0] = latest_teleop_cmd.angular.x * 0.3;
  latest_vel_cmd.angular[1] = latest_teleop_cmd.angular.y * 0.3;
  latest_vel_cmd.angular[2] = latest_teleop_cmd.angular.z * 0.3;
  
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
```

### 5. Change Message Types

If you need to handle different message types:

1. Update the include in the header file
2. Change the subscriber/publisher type
3. Update the message type in `create_subscription()` or `create_publisher()`
4. Modify callback or publication logic to match the new message structure

**Compilation:**

After modifications:

```bash
cd /home/megane/ros2_humble_ws
colcon build --packages-select sandbox_controller
source install/setup.bash
```

## Building and Testing

### Build

```bash
colcon build --packages-select sandbox_controller
```

### Launch

Using the provided launch file:

```bash
ros2 launch sandbox_controller explorer.launch.py
```

With simulation:

```bash
ros2 launch sandbox_controller explorer.launch.py use_simulation:=true
```

### Monitor Topics

```bash
ros2 topic list                              # List all topics
ros2 topic echo /controller_name/velocity_command   # Monitor velocity commands
ros2 topic echo /controller_name/ee_pose             # Monitor end-effector pose
ros2 topic echo /controller_name/joint_pose         # Monitor joint angles
```

### Check Controller Status

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {}
```

## Project Structure

```
sandbox_controller/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package dependencies
├── README.md                # This file
├── include/
│   └── sandbox_controller/
│       └── sandbox_controller.hpp     # Controller header
├── src/
│   ├── sandbox_controller.cpp         # Main controller implementation
│   ├── sandbox_controller.xml         # Pluginlib configuration
│   └── sandbox_parameters.yaml        # Parameter schema
├── bringup/
│   ├── config/
│   │   └── explorer_params.yaml       # Robot-specific configuration
│   └── launch/
│       └── explorer.launch.py         # Launch file
└── tablet_interface/        # (Optional) Web/mobile interface components
```

## Dependencies

- `rclcpp` - ROS 2 C++ client library
- `controller_interface` - ROS 2 controller base interface
- `pluginlib` - Plugin system for dynamic loading
- `geometry_msgs` - Standard geometry message types
- `robot_interfaces` - Generic robot component interface
- `extender_msgs` - Custom messages for this workspace
- `std_msgs` - Standard message types
