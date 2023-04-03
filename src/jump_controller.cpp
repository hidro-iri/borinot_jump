////////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2020 Pep Martí Saumell
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

#include "borinot_jump/jump_controller.hpp"

#include <chrono>

#include "eagle_ros2_tf/utils/tools.hpp"

using namespace eagle_ros2_tf;

namespace eagle_ros2_control
{

JumpController::JumpController(const std::string &node_name)
    : Node{node_name}, StatePubSub{node_name}, ControllerAbstract{node_name, 6, state_params_.njoints_arm}
{
    // TODO: remove the hardcoded 6

    initializeParameters();
    initializeSubscribers();
    initializePublishers();
    initializeServices();
    initializeTimers();

    RCLCPP_INFO_STREAM(get_logger(), "Jump Controller node loaded");
    if (jump_params_.write_px4_commands)
    {
        RCLCPP_WARN_STREAM(get_logger(), "BE CAREFUL! Custom commands will be applied");
    }
}

JumpController::~JumpController() {}

void JumpController::initializeParameters()
{
    // Node parameter
    declare_parameter<std::string>("platform_name", "");

    declare_parameter<bool>("write_px4_commands", true);

    declare_parameter<double>("propeller_velocity_normalized", 0.0);

    declare_parameter<std::vector<double>>("torque_commands", {0, 0});

    declare_parameter<std::vector<double>>("arm_landing", {0, 0});
    declare_parameter<std::vector<double>>("arm_takeoff", {0, 0});

    declare_parameter<double>("time_takeoff", 0.0);
    declare_parameter<std::vector<double>>("arm_kp", {0, 0});
    declare_parameter<std::vector<double>>("arm_kd", {0, 0});

    // Dump parameters into struct.
    // Won't change during node existence and they'll be queried at mpc running time
    get_parameter("platform_name", jump_params_.platform_name);
    get_parameter("write_px4_commands", jump_params_.write_px4_commands);
    jump_params_.time_takeoff = std::chrono::duration<double, std::milli>(get_parameter("time_takeoff").as_double());
    get_parameter("propeller_velocity_normalized", jump_params_.propeller_velocity_normalized);
    get_parameter("torque_commands", jump_params_.torque_commands);
    get_parameter("arm_kp", jump_params_.arm_kp);
    get_parameter("arm_kd", jump_params_.arm_kd);
    get_parameter("arm_landing", jump_params_.arm_landing);
    get_parameter("arm_takeoff", jump_params_.arm_takeoff);
}

void JumpController::initializeSubscribers()
{
    rclcpp::SubscriptionOptions sub_opt_misc = rclcpp::SubscriptionOptions();

    subs_landing_position_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "landing_position", rclcpp::QoS(1),
        std::bind(&JumpController::callbackLandingPosition, this, std::placeholders::_1), sub_opt_misc);
}

void JumpController::initializePublishers() {}

void JumpController::initializeTimers() {}

void JumpController::initializeServices()
{
    cb_group_srv_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srvs_prop_    = create_service<std_srvs::srv::SetBool>(
        std::string(get_name()) + "/enable_propellers",
        std::bind(&JumpController::callbackEnablePropellers, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cb_group_srv_);
    srvs_jump_ = create_service<std_srvs::srv::SetBool>(
        std::string(get_name()) + "/enable_jump",
        std::bind(&JumpController::callbackEnableJump, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cb_group_srv_);
}

void JumpController::callbackVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    const std::lock_guard<std::recursive_mutex> lock{state_data_.mutex};
    StatePubSub::callbackVehicleLocalPosition(msg);
}

void JumpController::callbackVehicleAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    const std::lock_guard<std::recursive_mutex> lock{state_data_.mutex};
    StatePubSub::callbackVehicleAttitude(msg);
}

void JumpController::callbackVehicleAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
{
    const std::lock_guard<std::recursive_mutex> lock{state_data_.mutex};
    StatePubSub::callbackVehicleAngularVelocity(msg);
}

void JumpController::callbackRobotState(const odri_ros2_msgs::msg::RobotState::SharedPtr msg)
{
    const std::lock_guard<std::recursive_mutex> lock{state_data_.mutex};
    StatePubSub::callbackRobotState(msg);
}

void JumpController::callbackLandingPosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    for (std::size_t i = 0; i < jump_params_.arm_landing.size(); ++i)
    {
        jump_params_.arm_landing[i] = msg->data[i];
    }
}

void JumpController::callbackEnablePropellers(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                              std_srvs::srv::SetBool::Response::SharedPtr      response)
{
    is_propeller_enabled_ = request->data;
    response->success     = true;
    if (is_propeller_enabled_)
    {
        response->message = "propeller enabled";
    }
    else
    {
        response->message = "propeller disabled";
    }
}

void JumpController::callbackEnableJump(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                        std_srvs::srv::SetBool::Response::SharedPtr      response)
{
    is_jumping_         = request->data;
    response->success = true;
    if (is_jumping_)
    {
        time_takeoff_last_call_ = std::chrono::steady_clock::now();
        response->message = "jump enabled";
    }
    else
    {
        response->message = "jump disabled";
    }
}

void JumpController::computeControls()
{
    if (state_machine_->getStateActive() == "running")
    {
        if (is_propeller_enabled_)
        {
            controller_data_.cmd_platform_normalized.setConstant(jump_params_.propeller_velocity_normalized);
        }
        else
        {
            controller_data_.cmd_platform_normalized.setConstant(-1.0);
        }
        if (is_jumping_)  // during take-off phase
        {
            RCLCPP_INFO_STREAM_ONCE(get_logger(), "Inside Take-Off phase");
            bool is_time_to_change = true;
            for (std::size_t i = 0; i < jump_params_.torque_commands.size(); ++i)
            {
                controller_data_.cmd_arm_pos[i] = jump_params_.arm_landing[i];
                controller_data_.cmd_arm_vel[i] = 0;

                const std::lock_guard<std::recursive_mutex> lock{state_data_.mutex};
                // idea: direction_of_the_torque*(pos_takeoff-pos) > 0 
                if (sign(jump_params_.torque_commands[i])*(jump_params_.arm_takeoff[i] - state_data_.state[7 + i]) > 0)
                {
                    is_time_to_change = false;

                    controller_data_.cmd_arm_torque[i] = jump_params_.torque_commands[i];
                    controller_data_.cmd_arm_kp[i]     = 0;
                    controller_data_.cmd_arm_kd[i]     = 0;
                }
                else
                {
                    controller_data_.cmd_arm_torque[i] = 0;
                    controller_data_.cmd_arm_kp[i]     = jump_params_.arm_kp[i];
                    controller_data_.cmd_arm_kd[i]     = jump_params_.arm_kd[i];
                }
            }
            if (is_time_to_change ||  std::chrono::steady_clock::now() - time_takeoff_last_call_ > jump_params_.time_takeoff)
            {
                is_jumping_ = false;
            }
        }
        if(!is_jumping_)  // during parking phase
        {
            RCLCPP_INFO_STREAM_ONCE(get_logger(), "Inside Parking phase");
            for (std::size_t i = 0; i < jump_params_.torque_commands.size(); ++i)
            {
                controller_data_.cmd_arm_torque[i] = 0;
                controller_data_.cmd_arm_pos[i]    = jump_params_.arm_landing[i];
                controller_data_.cmd_arm_vel[i]    = 0;
                controller_data_.cmd_arm_kp[i]     = jump_params_.arm_kp[i];
                controller_data_.cmd_arm_kd[i]     = jump_params_.arm_kd[i];
            }
        }
    }
    else
    {
        controller_data_.cmd_platform_normalized.setConstant(-1.0);
    }
}

bool JumpController::transEnableCallback(std::string &message)
{
    // Check that MicroRTPS Agent is publishing
    if (sub_local_position_->get_publisher_count() == 0 || sub_attitude_->get_publisher_count() == 0 ||
        sub_angular_velocity_->get_publisher_count() == 0)
    {
        message = "Cannot enable Jump Controller. Check MicroRTPS Agent is running.";
        return false;
    }

    // Check PX4 is running
    if (!state_data_.rcv_local_position && !state_data_.rcv_attitude && !state_data_.rcv_ang_vel &&
        !controller_data_.rcv_ctrl_mode)
    {
        message = "Cannot enable Jump Controller. Check PX4 is publishing the platform state.";
        return false;
    }

    // Enable the arm (if needed)
    if (controller_params_.has_arm)
    {
        if (controller_data_.arm_state_machine_status_ != "enabled" && !armRequestTransitionCommand("enable"))
        {
            message = "Cannot enable controller. Cannot successfully call the service to enable the arm";
            return false;
        }

        // sanity check
        // I can do this because arm_state_machine_status updates in a different thread (cb_group_misc_).
        // This callback is executed by (cb_group_srv_)
        rclcpp::sleep_for(std::chrono::milliseconds(300));  // Wait for the status variable to change
        if (controller_data_.arm_state_machine_status_ != "enabled")
        {
            message = "Cannot enable controller. Arm has not transitioned to 'enabled' state.";
            return false;
        }
    }

    if (jump_params_.write_px4_commands)
    {
        platformArm();
    }

    message = "SM Enable transition accepted";

    return true;
}

bool JumpController::transStartCallback(std::string &message)
{
    if (controller_params_.has_arm)
    {
        if (controller_data_.arm_state_machine_status_ != "running" && !armRequestTransitionCommand("start"))
        {
            message = "Cannot start the controller. Cannot successfully call the service to start the arm";
            return false;
        }

        // sanity check
        rclcpp::sleep_for(std::chrono::milliseconds(300));  // Wait for the status variable to change
        if (controller_data_.arm_state_machine_status_ != "running")
        {
            message = "Cannot start controller. Arm has not transitioned to 'running' state.";
            return false;
        }
    }

    if (jump_params_.write_px4_commands)
    {
        // Change to DirectControl mode
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 10);
    }

    message = "SM Start transition accepted";

    return true;
}

bool JumpController::transDisableCallback(std::string &message)
{
    RCLCPP_INFO(get_logger(), "Disable MPC Controller called");

    // Pos Ctl
    // publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);

    if (controller_params_.has_arm)
    {
        if (controller_data_.arm_state_machine_status_ == "enabled")
        {
            armRequestTransitionCommand("disable");
        }

        // just in case
        if (controller_data_.arm_state_machine_status_ == "running")
        {
            armRequestTransitionCommand("stop");
        }
    }

    message = "SM Disable transition accepted";
    return true;
}

}  // namespace eagle_ros2_control
