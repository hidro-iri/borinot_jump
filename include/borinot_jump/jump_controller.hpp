////////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2020-2021 Pep Martí Saumell
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <chrono>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include "tf2_eigen/tf2_eigen.h"

#include <px4_msgs/msg/actuator_direct_control.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <px4_ros_com/frame_transforms.h>

#include "eagle_ros2_tf/state_pub_sub.hpp"

#include "eagle_ros2_control/controller_base.hpp"

using namespace eagle_ros2_tf;

namespace eagle_ros2_control
{

class JumpController : public StatePubSub, public ControllerAbstract
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit JumpController(const std::string& node_name);
    virtual ~JumpController();

  private:
    // initializers
    void initializeParameters();
    void initializeSubscribers();
    void initializePublishers();
    void initializeTimers();
    void initializeServices();

    // State subscribers callbacks
    virtual void callbackVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) override;
    virtual void callbackVehicleAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) override;
    virtual void callbackVehicleAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) override;
    virtual void callbackRobotState(const odri_ros2_msgs::msg::RobotState::SharedPtr msg) override;
    void         callbackLandingPosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // Controller service callbacks
    void callbackEnablePropellers(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                  std_srvs::srv::SetBool::Response::SharedPtr      response);
    void callbackEnableJump(const std_srvs::srv::SetBool::Request::SharedPtr request,
                            std_srvs::srv::SetBool::Response::SharedPtr      response);

    // State machine tranistions
    virtual bool transEnableCallback(std::string& message) override;
    virtual bool transStartCallback(std::string& message) override;
    virtual bool transDisableCallback(std::string& message) override;

    // Publishing methods
    virtual void computeControls() override;

  private:
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subs_landing_position_;

    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srvs_prop_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srvs_jump_;

    rclcpp::CallbackGroup::SharedPtr cb_group_srv_;

    struct JumpParams
    {
        std::string platform_name;

        bool write_px4_commands;

        // Propeller velocity
        double propeller_velocity_normalized;

        // Torque command
        std::vector<double> torque_commands;

        // Arm state
        std::vector<double> arm_landing;
        std::vector<double> arm_takeoff;

        // Landing phase parameters
        std::chrono::duration<double, std::milli> time_takeoff;
        std::vector<double>                       arm_kp;
        std::vector<double>                       arm_kd;
    } jump_params_;

    std::chrono::steady_clock::time_point time_takeoff_last_call_;

    bool is_propeller_enabled_{false};
    bool is_jumping_{false};
};
}  // namespace eagle_ros2_control
