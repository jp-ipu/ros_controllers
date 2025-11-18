/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Mark Naeem
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The names of the contributors may NOT be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Mark Naeem
 */

#include <swerve_steering_controller/swerve_steering_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace swerve_steering_controller
{
  SwerveSteeringController::SwerveSteeringController():
      base_frame_id_("base_link")
    , odom_frame_id_("odom")
    , enable_odom_tf_(true)
    , publish_wheel_joint_controller_state_(false)
    , wheel_joints_size_(0)
  {
  }

  controller_interface::InterfaceConfiguration
  SwerveSteeringController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & wheel_name : wheel_joint_names_)
    {
      config.names.push_back(wheel_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    for (const auto & holder_name : holder_joint_names_)
    {
      config.names.push_back(holder_name + "/" + hardware_interface::HW_IF_POSITION);
    }

    return config;
  }

  controller_interface::InterfaceConfiguration
  SwerveSteeringController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & wheel_name : wheel_joint_names_)
    {
      config.names.push_back(wheel_name + "/" + hardware_interface::HW_IF_VELOCITY);
      config.names.push_back(wheel_name + "/" + hardware_interface::HW_IF_POSITION);
    }

    for (const auto & holder_name : holder_joint_names_)
    {
      config.names.push_back(holder_name + "/" + hardware_interface::HW_IF_POSITION);
      config.names.push_back(holder_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    return config;
  }

  controller_interface::CallbackReturn SwerveSteeringController::on_init()
  {
    try
    {
      auto_declare<std::vector<std::string>>("wheels", std::vector<std::string>());
      auto_declare<std::vector<std::string>>("holders", std::vector<std::string>());
      auto_declare<std::vector<double>>("radii", std::vector<double>());
      auto_declare<std::vector<double>>("positions", std::vector<double>());  // Flattened 2D array
      auto_declare<std::vector<bool>>("limitless", std::vector<bool>());
      auto_declare<std::vector<double>>("limits", std::vector<double>());  // Flattened 2D array
      auto_declare<std::vector<double>>("offsets", std::vector<double>());

      auto_declare<std::string>("base_frame_id", base_frame_id_);
      auto_declare<std::string>("odom_frame_id", odom_frame_id_);
      auto_declare<bool>("enable_odom_tf", true);
      auto_declare<double>("publish_rate", 50.0);
      auto_declare<int>("velocity_rolling_window_size", 10);
      auto_declare<double>("infinity_tolerance", 1000.0);
      auto_declare<double>("intersection_tolerance", 0.1);
      auto_declare<bool>("publish_wheel_joint_controller_state", false);

      // Velocity and acceleration limits
      auto_declare<bool>("linear.x.has_velocity_limits", false);
      auto_declare<bool>("linear.x.has_acceleration_limits", false);
      auto_declare<double>("linear.x.max_velocity", 0.0);
      auto_declare<double>("linear.x.min_velocity", 0.0);
      auto_declare<double>("linear.x.max_acceleration", 0.0);
      auto_declare<double>("linear.x.min_acceleration", 0.0);

      auto_declare<bool>("linear.y.has_velocity_limits", false);
      auto_declare<bool>("linear.y.has_acceleration_limits", false);
      auto_declare<double>("linear.y.max_velocity", 0.0);
      auto_declare<double>("linear.y.min_velocity", 0.0);
      auto_declare<double>("linear.y.max_acceleration", 0.0);
      auto_declare<double>("linear.y.min_acceleration", 0.0);

      auto_declare<bool>("angular.z.has_velocity_limits", false);
      auto_declare<bool>("angular.z.has_acceleration_limits", false);
      auto_declare<double>("angular.z.max_velocity", 0.0);
      auto_declare<double>("angular.z.min_velocity", 0.0);
      auto_declare<double>("angular.z.max_acceleration", 0.0);
      auto_declare<double>("angular.z.min_acceleration", 0.0);

      // Odometry covariance
      auto_declare<std::vector<double>>("pose_covariance_diagonal",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      auto_declare<std::vector<double>>("twist_covariance_diagonal",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn SwerveSteeringController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto node = get_node();

    // Get joint names
    wheel_joint_names_ = node->get_parameter("wheels").as_string_array();
    holder_joint_names_ = node->get_parameter("holders").as_string_array();

    if (wheel_joint_names_.empty() || holder_joint_names_.empty())
    {
      RCLCPP_ERROR(node->get_logger(), "Wheel or holder joint names not specified");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (wheel_joint_names_.size() != holder_joint_names_.size())
    {
      RCLCPP_ERROR(node->get_logger(), "Number of wheels and holders must match");
      return controller_interface::CallbackReturn::ERROR;
    }

    wheel_joints_size_ = wheel_joint_names_.size();

    // Get wheel parameters
    if (!getWheelParams())
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to get wheel parameters");
      return controller_interface::CallbackReturn::ERROR;
    }

    // Configure speed limiters
    limiter_lin_x_.has_velocity_limits = node->get_parameter("linear.x.has_velocity_limits").as_bool();
    limiter_lin_x_.has_acceleration_limits = node->get_parameter("linear.x.has_acceleration_limits").as_bool();
    limiter_lin_x_.max_velocity = node->get_parameter("linear.x.max_velocity").as_double();
    limiter_lin_x_.min_velocity = node->get_parameter("linear.x.min_velocity").as_double();
    limiter_lin_x_.max_acceleration = node->get_parameter("linear.x.max_acceleration").as_double();
    limiter_lin_x_.min_acceleration = node->get_parameter("linear.x.min_acceleration").as_double();

    limiter_lin_y_.has_velocity_limits = node->get_parameter("linear.y.has_velocity_limits").as_bool();
    limiter_lin_y_.has_acceleration_limits = node->get_parameter("linear.y.has_acceleration_limits").as_bool();
    limiter_lin_y_.max_velocity = node->get_parameter("linear.y.max_velocity").as_double();
    limiter_lin_y_.min_velocity = node->get_parameter("linear.y.min_velocity").as_double();
    limiter_lin_y_.max_acceleration = node->get_parameter("linear.y.max_acceleration").as_double();
    limiter_lin_y_.min_acceleration = node->get_parameter("linear.y.min_acceleration").as_double();

    limiter_ang_.has_velocity_limits = node->get_parameter("angular.z.has_velocity_limits").as_bool();
    limiter_ang_.has_acceleration_limits = node->get_parameter("angular.z.has_acceleration_limits").as_bool();
    limiter_ang_.max_velocity = node->get_parameter("angular.z.max_velocity").as_double();
    limiter_ang_.min_velocity = node->get_parameter("angular.z.min_velocity").as_double();
    limiter_ang_.max_acceleration = node->get_parameter("angular.z.max_acceleration").as_double();
    limiter_ang_.min_acceleration = node->get_parameter("angular.z.min_acceleration").as_double();

    // Get other parameters
    base_frame_id_ = node->get_parameter("base_frame_id").as_string();
    odom_frame_id_ = node->get_parameter("odom_frame_id").as_string();
    enable_odom_tf_ = node->get_parameter("enable_odom_tf").as_bool();
    publish_wheel_joint_controller_state_ = node->get_parameter("publish_wheel_joint_controller_state").as_bool();

    double publish_rate = node->get_parameter("publish_rate").as_double();
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);

    int velocity_rolling_window_size = node->get_parameter("velocity_rolling_window_size").as_int();
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    infinity_tol_ = node->get_parameter("infinity_tolerance").as_double();
    intersection_tol_ = node->get_parameter("intersection_tolerance").as_double();

    // Setup publishers
    setOdomPubFields();

    // Setup command velocity subscriber
    cmd_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
      std::bind(&SwerveSteeringController::cmd_callback, this, std::placeholders::_1));

    // Initialize previous velocity vectors
    wheels_velocities_previous_.resize(wheel_joints_size_, 0.0);
    holders_velocities_previous_.resize(wheel_joints_size_, 0.0);
    wheels_desired_velocities_previous_.resize(wheel_joints_size_, 0.0);
    holders_desired_velocities_previous_.resize(wheel_joints_size_, 0.0);
    holders_desired_positions_previous_.resize(wheel_joints_size_, 0.0);

    RCLCPP_INFO(node->get_logger(), "Swerve steering controller configured successfully");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn SwerveSteeringController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto node = get_node();

    // Claim command interfaces
    wheel_velocity_command_interfaces_.reserve(wheel_joints_size_);
    holder_position_command_interfaces_.reserve(wheel_joints_size_);

    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      auto it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [this, i](const auto & interface) {
          return interface.get_prefix_name() == wheel_joint_names_[i] &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });

      if (it == command_interfaces_.end())
      {
        RCLCPP_ERROR(node->get_logger(), "Could not find velocity command interface for wheel %s",
                     wheel_joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      wheel_velocity_command_interfaces_.emplace_back(*it);

      it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [this, i](const auto & interface) {
          return interface.get_prefix_name() == holder_joint_names_[i] &&
                 interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });

      if (it == command_interfaces_.end())
      {
        RCLCPP_ERROR(node->get_logger(), "Could not find position command interface for holder %s",
                     holder_joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      holder_position_command_interfaces_.emplace_back(*it);
    }

    // Claim state interfaces
    wheel_velocity_state_interfaces_.reserve(wheel_joints_size_);
    wheel_position_state_interfaces_.reserve(wheel_joints_size_);
    holder_position_state_interfaces_.reserve(wheel_joints_size_);
    holder_velocity_state_interfaces_.reserve(wheel_joints_size_);

    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      // Wheel velocity state is REQUIRED for odometry
      auto it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [this, i](const auto & interface) {
          return interface.get_prefix_name() == wheel_joint_names_[i] &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });
      if (it == state_interfaces_.end())
      {
        RCLCPP_ERROR(node->get_logger(), "Could not find velocity state interface for wheel %s",
                     wheel_joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      wheel_velocity_state_interfaces_.emplace_back(*it);

      // Holder position state is REQUIRED for odometry and control
      it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [this, i](const auto & interface) {
          return interface.get_prefix_name() == holder_joint_names_[i] &&
                 interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });
      if (it == state_interfaces_.end())
      {
        RCLCPP_ERROR(node->get_logger(), "Could not find position state interface for holder %s",
                     holder_joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      holder_position_state_interfaces_.emplace_back(*it);

      // Wheel position state is OPTIONAL (only needed for controller state publishing)
      it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [this, i](const auto & interface) {
          return interface.get_prefix_name() == wheel_joint_names_[i] &&
                 interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });
      if (it != state_interfaces_.end()) wheel_position_state_interfaces_.emplace_back(*it);

      // Holder velocity state is OPTIONAL (only needed for controller state publishing)
      it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [this, i](const auto & interface) {
          return interface.get_prefix_name() == holder_joint_names_[i] &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });
      if (it != state_interfaces_.end()) holder_velocity_state_interfaces_.emplace_back(*it);
    }

    // Validate we have the optional interfaces if controller state publishing is enabled
    if (publish_wheel_joint_controller_state_)
    {
      if (wheel_position_state_interfaces_.size() != wheel_joints_size_ ||
          holder_velocity_state_interfaces_.size() != wheel_joints_size_)
      {
        RCLCPP_WARN(node->get_logger(),
                    "Controller state publishing enabled but not all state interfaces available. "
                    "Disabling controller state publishing.");
        publish_wheel_joint_controller_state_ = false;
      }
    }

    // Initialize odometry
    odometry_.init(node->now(), infinity_tol_, intersection_tol_);
    last_state_publish_time_ = node->now();
    time_previous_ = node->now();

    RCLCPP_INFO(node->get_logger(), "Swerve steering controller activated");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn SwerveSteeringController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    set_to_initial_state();

    wheel_velocity_command_interfaces_.clear();
    holder_position_command_interfaces_.clear();
    wheel_velocity_state_interfaces_.clear();
    wheel_position_state_interfaces_.clear();
    holder_position_state_interfaces_.clear();
    holder_velocity_state_interfaces_.clear();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type SwerveSteeringController::update(
      const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    auto node = get_node();

    // Get current wheel and holder states
    std::vector<double> wheels_omega, holders_theta;
    std::vector<int> directions;

    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      double wheel_vel = wheel_velocity_state_interfaces_[i].get().get_optional().value();
      double holder_pos = holder_position_state_interfaces_[i].get().get_optional().value();

      wheels_omega.push_back(wheel_vel);
      holders_theta.push_back(holder_pos);
      wheels_[i].set_current_angle(holder_pos);
      directions.push_back(wheels_[i].get_omega_direction());

      RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
        "Wheel %zu: holder_joint=%s, state_pos=%.6f, wheel_current_angle=%.6f, omega_dir=%d",
        i, holder_joint_names_[i].c_str(), holder_pos, wheels_[i].get_current_angle(), wheels_[i].get_omega_direction());
    }

    // Update odometry
    std::array<double,2> intersection_point = {0, 0};
    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "Calling odometry.update with holders_theta=[%.6f, %.6f], wheels_omega=[%.6f, %.6f], directions=[%d, %d]",
      holders_theta[0], holders_theta[1], wheels_omega[0], wheels_omega[1], directions[0], directions[1]);
    odometry_.update(wheels_omega, holders_theta, directions, time, &intersection_point);

    // Publish intersection point
    if (rt_avg_intersection_publisher_ && rt_avg_intersection_publisher_->trylock())
    {
      rt_avg_intersection_publisher_->msg_.x = intersection_point[0];
      rt_avg_intersection_publisher_->msg_.y = intersection_point[1];
      rt_avg_intersection_publisher_->unlockAndPublish();
    }

    // Publish odometry
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;

      // Create quaternion from yaw
      tf2::Quaternion quat;
      quat.setRPY(0.0, 0.0, odometry_.getHeading());
      geometry_msgs::msg::Quaternion orientation = tf2::toMsg(quat);

      // Publish odometry message
      if (rt_odom_publisher_ && rt_odom_publisher_->trylock())
      {
        rt_odom_publisher_->msg_.header.stamp = time;
        rt_odom_publisher_->msg_.pose.pose.position.x = odometry_.getX();
        rt_odom_publisher_->msg_.pose.pose.position.y = odometry_.getY();
        rt_odom_publisher_->msg_.pose.pose.orientation = orientation;
        rt_odom_publisher_->msg_.twist.twist.linear.x = odometry_.getLinearX();
        rt_odom_publisher_->msg_.twist.twist.linear.y = odometry_.getLinearY();
        rt_odom_publisher_->msg_.twist.twist.angular.z = odometry_.getAngular();
        rt_odom_publisher_->unlockAndPublish();
      }

      // Publish TF
      if (enable_odom_tf_ && rt_tf_odom_publisher_ && rt_tf_odom_publisher_->trylock())
      {
        geometry_msgs::msg::TransformStamped & odom_frame = rt_tf_odom_publisher_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        rt_tf_odom_publisher_->unlockAndPublish();
      }
    }

    // Get current velocity command
    utils::command current_cmd = *(commands_buffer_.readFromRT());

    // Limit velocities and accelerations
    const double cmd_dt = period.seconds();

    limiter_lin_x_.limit(current_cmd.x, last0_cmd_.x, last1_cmd_.x, cmd_dt);
    limiter_lin_y_.limit(current_cmd.y, last0_cmd_.y, last1_cmd_.y, cmd_dt);
    limiter_ang_.limit(current_cmd.w, last0_cmd_.w, last1_cmd_.w, cmd_dt);

    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "Cmd_vel: x=%.3f y=%.3f w=%.3f", current_cmd.x, current_cmd.y, current_cmd.w);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = current_cmd;

    // Compute wheel velocities and steering angles
    std::vector<double> desired_velocities, desired_positions;

    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      double wheel_vx = current_cmd.x - current_cmd.w * wheels_[i].position[1] -
                        wheels_[i].offset * cos(holder_position_state_interfaces_[i].get().get_optional().value());
      double wheel_vy = current_cmd.y + current_cmd.w * wheels_[i].position[0] +
                        wheels_[i].offset * sin(holder_position_state_interfaces_[i].get().get_optional().value());

      // Calculate required wheel speed and steering angle
      double w_w = sqrt(pow(wheel_vx, 2) + pow(wheel_vy, 2)) / wheels_[i].radius;
      double w_th = atan2(wheel_vy, wheel_vx);

      // Process command through wheel class
      double current_holder_pos = holder_position_state_interfaces_[i].get().get_optional().value();
      wheels_[i].set_current_angle(current_holder_pos);
      wheels_[i].set_command_velocity(w_w);
      wheels_[i].set_command_angle(w_th);

      // Get actual commands to apply
      double w_applied = wheels_[i].get_command_velocity();
      double th_applied = wheels_[i].get_command_angle();

      RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
        "Wheel %zu cmd: vx=%.3f vy=%.3f -> raw_angle=%.6f raw_vel=%.3f | current_pos=%.6f -> applied_angle=%.6f applied_vel=%.3f omega_dir=%d",
        i, wheel_vx, wheel_vy, w_th, w_w, current_holder_pos, th_applied, w_applied, wheels_[i].get_omega_direction());

      if (publish_wheel_joint_controller_state_)
      {
        desired_velocities.push_back(w_applied);
        desired_positions.push_back(th_applied);
      }

      // Set commands
      if (!wheel_velocity_command_interfaces_[i].get().set_value(w_applied))
      {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "Failed to set velocity command for wheel %zu", i);
      }
      if (!holder_position_command_interfaces_[i].get().set_value(th_applied))
      {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "Failed to set position command for holder %zu", i);
      }
    }

    publishWheelData(time, period, desired_velocities, desired_positions);

    time_previous_ = time;
    return controller_interface::return_type::OK;
  }

  void SwerveSteeringController::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr command)
  {
    if (!std::isfinite(command->angular.z) || !std::isfinite(command->linear.x) || !std::isfinite(command->linear.y))
    {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Received NaN in velocity command. Ignoring.");
      return;
    }

    cmd_.w = command->angular.z;
    cmd_.x = command->linear.x;
    cmd_.y = command->linear.y;
    cmd_.stamp = get_node()->now();
    commands_buffer_.writeFromNonRT(cmd_);
  }

  bool SwerveSteeringController::getWheelParams()
  {
    auto node = get_node();

    auto radii = node->get_parameter("radii").as_double_array();
    auto positions_param = node->get_parameter("positions").as_double_array();
    auto limitless = node->get_parameter("limitless").as_bool_array();
    auto limits_param = node->get_parameter("limits").as_double_array();
    auto offsets = node->get_parameter("offsets").as_double_array();

    if (radii.size() != wheel_joints_size_ ||
        limitless.size() != wheel_joints_size_ ||
        offsets.size() != wheel_joints_size_ ||
        positions_param.size() != wheel_joints_size_ * 2 ||
        limits_param.size() != wheel_joints_size_ * 2)
    {
      RCLCPP_ERROR(node->get_logger(), "Parameter array sizes don't match number of wheels. "
                   "Expected: radii=%zu, positions=%zu, limitless=%zu, offsets=%zu, limits=%zu. "
                   "Got: radii=%zu, positions=%zu, limitless=%zu, offsets=%zu, limits=%zu",
                   wheel_joints_size_, wheel_joints_size_ * 2, wheel_joints_size_,
                   wheel_joints_size_, wheel_joints_size_ * 2,
                   radii.size(), positions_param.size(), limitless.size(),
                   offsets.size(), limits_param.size());
      return false;
    }

    wheels_.resize(wheel_joints_size_);

    // Parse positions (2D array flattened)
    std::vector<std::array<double,2>> positions;
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      positions.push_back({positions_param[i*2], positions_param[i*2 + 1]});
    }

    // Set wheel parameters
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      wheels_[i].radius = radii[i];
      wheels_[i].set_position(positions[i]);
      wheels_[i].set_limitless(limitless[i]);
      wheels_[i].offset = offsets[i];

      if (!limitless[i])
      {
        std::array<double,2> limits = {limits_param[i*2], limits_param[i*2 + 1]};
        wheels_[i].set_rotation_limits(limits);
      }
    }

    odometry_.setWheelsParams(radii, positions);

    return true;
  }

  void SwerveSteeringController::setOdomPubFields()
  {
    auto node = get_node();

    auto pose_cov = node->get_parameter("pose_covariance_diagonal").as_double_array();
    auto twist_cov = node->get_parameter("twist_covariance_diagonal").as_double_array();

    // Create intersection point publisher
    avg_intersection_publisher_ = node->create_publisher<geometry_msgs::msg::Point>("~/avg_intersection", 10);
    rt_avg_intersection_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Point>>(
      avg_intersection_publisher_);

    // Create odometry publisher
    odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    rt_odom_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odom_publisher_);

    rt_odom_publisher_->msg_.header.frame_id = odom_frame_id_;
    rt_odom_publisher_->msg_.child_frame_id = base_frame_id_;
    rt_odom_publisher_->msg_.pose.pose.position.z = 0;

    rt_odom_publisher_->msg_.pose.covariance = {
      pose_cov[0], 0., 0., 0., 0., 0.,
      0., pose_cov[1], 0., 0., 0., 0.,
      0., 0., pose_cov[2], 0., 0., 0.,
      0., 0., 0., pose_cov[3], 0., 0.,
      0., 0., 0., 0., pose_cov[4], 0.,
      0., 0., 0., 0., 0., pose_cov[5]
    };

    rt_odom_publisher_->msg_.twist.twist.linear.y = 0;
    rt_odom_publisher_->msg_.twist.twist.linear.z = 0;
    rt_odom_publisher_->msg_.twist.twist.angular.x = 0;
    rt_odom_publisher_->msg_.twist.twist.angular.y = 0;

    rt_odom_publisher_->msg_.twist.covariance = {
      twist_cov[0], 0., 0., 0., 0., 0.,
      0., twist_cov[1], 0., 0., 0., 0.,
      0., 0., twist_cov[2], 0., 0., 0.,
      0., 0., 0., twist_cov[3], 0., 0.,
      0., 0., 0., 0., twist_cov[4], 0.,
      0., 0., 0., 0., 0., twist_cov[5]
    };

    // Create TF publisher
    tf_odom_publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    rt_tf_odom_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      tf_odom_publisher_);

    rt_tf_odom_publisher_->msg_.transforms.resize(1);
    rt_tf_odom_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
    rt_tf_odom_publisher_->msg_.transforms[0].child_frame_id = base_frame_id_;
    rt_tf_odom_publisher_->msg_.transforms[0].header.frame_id = odom_frame_id_;

    // Create controller state publisher if requested
    if (publish_wheel_joint_controller_state_)
    {
      controller_state_pub_ = node->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
        "~/wheel_joint_controller_state", 10);
      rt_controller_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>(
        controller_state_pub_);

      const size_t num_joints = wheel_joints_size_ * 2;
      rt_controller_state_pub_->msg_.joint_names.resize(num_joints);
      rt_controller_state_pub_->msg_.reference.positions.resize(num_joints);
      rt_controller_state_pub_->msg_.reference.velocities.resize(num_joints);
      rt_controller_state_pub_->msg_.reference.accelerations.resize(num_joints);
      rt_controller_state_pub_->msg_.feedback.positions.resize(num_joints);
      rt_controller_state_pub_->msg_.feedback.velocities.resize(num_joints);
      rt_controller_state_pub_->msg_.feedback.accelerations.resize(num_joints);
      rt_controller_state_pub_->msg_.error.positions.resize(num_joints);
      rt_controller_state_pub_->msg_.error.velocities.resize(num_joints);
      rt_controller_state_pub_->msg_.error.accelerations.resize(num_joints);
      rt_controller_state_pub_->msg_.output.positions.resize(num_joints);
      rt_controller_state_pub_->msg_.output.velocities.resize(num_joints);
      rt_controller_state_pub_->msg_.output.accelerations.resize(num_joints);

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        rt_controller_state_pub_->msg_.joint_names[i] = wheel_joint_names_[i];
        rt_controller_state_pub_->msg_.joint_names[i + wheel_joints_size_] = holder_joint_names_[i];
      }
    }
  }

  void SwerveSteeringController::set_to_initial_state()
  {
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      (void)wheel_velocity_command_interfaces_[i].get().set_value(0.0);
      (void)holder_position_command_interfaces_[i].get().set_value(0.0);
    }
  }

  void SwerveSteeringController::publishWheelData(
      const rclcpp::Time& time, const rclcpp::Duration& period,
      std::vector<double> wheels_desired_velocities,
      std::vector<double> holders_desired_positions)
  {
    if (publish_wheel_joint_controller_state_ && rt_controller_state_pub_ && rt_controller_state_pub_->trylock())
    {
      const double cmd_dt = period.seconds();

      rt_controller_state_pub_->msg_.header.stamp = time;
      const double control_duration = (time - time_previous_).seconds();

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        double holder_desired_velocity = (holders_desired_positions[i] - holders_desired_positions_previous_[i]) / cmd_dt;

        const double wheel_acc = (wheel_velocity_state_interfaces_[i].get().get_optional().value() - wheels_velocities_previous_[i]) / control_duration;
        const double holder_acc = (holder_velocity_state_interfaces_[i].get().get_optional().value() - holders_velocities_previous_[i]) / control_duration;

        // Feedback (actual state from sensors)
        rt_controller_state_pub_->msg_.feedback.positions[i] = wheel_position_state_interfaces_[i].get().get_optional().value();
        rt_controller_state_pub_->msg_.feedback.velocities[i] = wheel_velocity_state_interfaces_[i].get().get_optional().value();
        rt_controller_state_pub_->msg_.feedback.accelerations[i] = wheel_acc;

        rt_controller_state_pub_->msg_.feedback.positions[i + wheel_joints_size_] = holder_position_state_interfaces_[i].get().get_optional().value();
        rt_controller_state_pub_->msg_.feedback.velocities[i + wheel_joints_size_] = holder_velocity_state_interfaces_[i].get().get_optional().value();
        rt_controller_state_pub_->msg_.feedback.accelerations[i + wheel_joints_size_] = holder_acc;

        // Reference (desired state)
        rt_controller_state_pub_->msg_.reference.positions[i] += wheels_desired_velocities[i] * cmd_dt;
        rt_controller_state_pub_->msg_.reference.velocities[i] = wheels_desired_velocities[i];
        rt_controller_state_pub_->msg_.reference.accelerations[i] = (wheels_desired_velocities[i] - wheels_desired_velocities_previous_[i]) / cmd_dt;

        rt_controller_state_pub_->msg_.reference.positions[i + wheel_joints_size_] += holder_desired_velocity * cmd_dt;
        rt_controller_state_pub_->msg_.reference.velocities[i + wheel_joints_size_] = holder_desired_velocity;
        rt_controller_state_pub_->msg_.reference.accelerations[i + wheel_joints_size_] = (holder_desired_velocity - holders_desired_velocities_previous_[i]) / cmd_dt;

        // Output (commands actually sent to hardware)
        rt_controller_state_pub_->msg_.output.velocities[i] = wheels_desired_velocities[i];
        rt_controller_state_pub_->msg_.output.positions[i + wheel_joints_size_] = holders_desired_positions[i];

        // Error (reference - feedback)
        rt_controller_state_pub_->msg_.error.positions[i] =
          rt_controller_state_pub_->msg_.reference.positions[i] - rt_controller_state_pub_->msg_.feedback.positions[i];
        rt_controller_state_pub_->msg_.error.velocities[i] =
          rt_controller_state_pub_->msg_.reference.velocities[i] - rt_controller_state_pub_->msg_.feedback.velocities[i];
        rt_controller_state_pub_->msg_.error.accelerations[i] =
          rt_controller_state_pub_->msg_.reference.accelerations[i] - rt_controller_state_pub_->msg_.feedback.accelerations[i];

        rt_controller_state_pub_->msg_.error.positions[i + wheel_joints_size_] =
          rt_controller_state_pub_->msg_.reference.positions[i + wheel_joints_size_] - rt_controller_state_pub_->msg_.feedback.positions[i + wheel_joints_size_];
        rt_controller_state_pub_->msg_.error.velocities[i + wheel_joints_size_] =
          rt_controller_state_pub_->msg_.reference.velocities[i + wheel_joints_size_] - rt_controller_state_pub_->msg_.feedback.velocities[i + wheel_joints_size_];
        rt_controller_state_pub_->msg_.error.accelerations[i + wheel_joints_size_] =
          rt_controller_state_pub_->msg_.reference.accelerations[i + wheel_joints_size_] - rt_controller_state_pub_->msg_.feedback.accelerations[i + wheel_joints_size_];

        // Save previous values for next iteration
        wheels_velocities_previous_[i] = wheel_velocity_state_interfaces_[i].get().get_optional().value();
        wheels_desired_velocities_previous_[i] = wheels_desired_velocities[i];

        holders_velocities_previous_[i] = holder_velocity_state_interfaces_[i].get().get_optional().value();
        holders_desired_positions_previous_[i] = holders_desired_positions[i];
        holders_desired_velocities_previous_[i] = holder_desired_velocity;
      }

      rt_controller_state_pub_->unlockAndPublish();
    }
  }

} // namespace swerve_steering_controller

PLUGINLIB_EXPORT_CLASS(swerve_steering_controller::SwerveSteeringController, controller_interface::ControllerInterface)
