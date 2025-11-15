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

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/helpers.hpp>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <vector>
#include <cmath>
#include <memory>
#include <string>

#include <swerve_steering_controller/utils.h>
#include <swerve_steering_controller/odometry.h>
#include <swerve_steering_controller/wheel.h>
#include <swerve_steering_controller/speed_limiter.h>

namespace swerve_steering_controller
{
    class SwerveSteeringController : public controller_interface::ControllerInterface
  {
      public:
          SwerveSteeringController();

          controller_interface::InterfaceConfiguration command_interface_configuration() const override;

          controller_interface::InterfaceConfiguration state_interface_configuration() const override;

          controller_interface::CallbackReturn on_init() override;

          controller_interface::CallbackReturn on_configure(
              const rclcpp_lifecycle::State & previous_state) override;

          controller_interface::CallbackReturn on_activate(
              const rclcpp_lifecycle::State & previous_state) override;

          controller_interface::CallbackReturn on_deactivate(
              const rclcpp_lifecycle::State & previous_state) override;

          controller_interface::return_type update(
              const rclcpp::Time & time, const rclcpp::Duration & period) override;

      private:
          std::string base_frame_id_;
          std::string odom_frame_id_;

          rclcpp::Duration publish_period_{0, 0};
          rclcpp::Time last_state_publish_time_{0, 0, RCL_ROS_TIME};

          std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher_;
          std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_publisher_;

          std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tf_odom_publisher_;
          std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> rt_tf_odom_publisher_;

          std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Point>> avg_intersection_publisher_;
          std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Point>> rt_avg_intersection_publisher_;

          Odometry odometry_;

          double infinity_tol_;
          double intersection_tol_;

          /// Speed limiters:
          utils::command last1_cmd_;
          utils::command last0_cmd_;
          SpeedLimiter limiter_lin_x_;
          SpeedLimiter limiter_lin_y_;
          SpeedLimiter limiter_ang_;

          /// Previous time and velocities from the encoders:
          rclcpp::Time time_previous_{0, 0, RCL_ROS_TIME};
          std::vector<double> wheels_velocities_previous_;
          std::vector<double> holders_velocities_previous_;

          std::vector<double> wheels_desired_velocities_previous_;
          std::vector<double> holders_desired_velocities_previous_;
          std::vector<double> holders_desired_positions_previous_;

          bool enable_odom_tf_;
          bool publish_wheel_joint_controller_state_;

          size_t wheel_joints_size_;

          std::vector<wheel> wheels_;

          // Command and state interface names
          std::vector<std::string> wheel_joint_names_;
          std::vector<std::string> holder_joint_names_;

          // Command interfaces
          std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
              wheel_velocity_command_interfaces_;
          std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
              holder_position_command_interfaces_;

          // State interfaces
          std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
              wheel_velocity_state_interfaces_;
          std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
              wheel_position_state_interfaces_;
          std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
              holder_position_state_interfaces_;
          std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
              holder_velocity_state_interfaces_;

          utils::command cmd_;
          rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
          realtime_tools::RealtimeBuffer<utils::command> commands_buffer_;

          std::shared_ptr<rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>> controller_state_pub_;
          std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>> rt_controller_state_pub_;

          void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr command);

          bool getWheelParams();

          void setOdomPubFields();

          void set_to_initial_state();

          void publishWheelData(const rclcpp::Time& time, const rclcpp::Duration& period,
                                std::vector<double> wheels_desired_velocities,
                                std::vector<double> holders_desired_positions);

    };
}// namespace swerve_steering_controller
