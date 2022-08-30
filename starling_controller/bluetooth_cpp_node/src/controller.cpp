/*
 * Starling Template
 * Copyright (C) 2021 University of Bristol
 *
 * Author: Mickey Li <mickey.li@bristol.ac.uk> (University of Bristol)
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include "controller.hpp"
#include "main.hpp"

UserController::UserController(UAVController *node)
{
    // Set node so this controller can refer back to it later
    this->node = node;
    // Initialise publishers
    this->notify_angle_pub = this->node->create_publisher<bluetooth_msgs::msg::TargetAngle>("/monitor/notify_angle", 1);
    // request current vehicle id and total number
    this->vehicle_info_request_pub = this->node->create_publisher<std_msgs::msg::Empty>("/monitor/vehicle_info", 1);
    //reset initial takeoff point
    this->notify_vehicles_pub = this->node->create_publisher<bluetooth_msgs::msg::NotifyVehicles>("notify_vehicles", 1);



    // Initialise Subscribers
    this->sync_angle_sub = this->node->create_subscription<bluetooth_msgs::msg::TargetAngle>(
        "sync_angle", 10,
        std::bind(&UserController::handleTargetAngle, this, std::placeholders::_1));

    this->notify_vehicles_sub = this->node->create_subscription<bluetooth_msgs::msg::NotifyVehicles>(
        "notify_vehicles", 10,
        std::bind(&UserController::handleNotifyVehicles, this, std::placeholders::_1));

    this->bluetooth_dongle_sub = this->node->create_subscription<bluetooth_msgs::msg::BluetoothDongle>(
        "bluetooth_receive", 10,
        std::bind(&UserController::handleReceivedBtMsg, this, std::placeholders::_1));

    this->node->get_parameter_or("sync_angle_P", this->sync_angle_P, 0.001);

    // Variables
    this->origin = geometry_msgs::msg::Point();
}

void UserController::reset()
{
    this->system_vehicle_id = 0;
    this->received_circle_id = false;
}

bool UserController::smReady(const rclcpp::Time &stamp)
{
    if (!this->received_circle_id)
    {
        std_msgs::msg::Empty e;
        this->vehicle_info_request_pub->publish(e);
        RCLCPP_INFO(this->get_logger(), "Waiting for Notify Vehicle message from central monitor");
        return false;
    }
    return true;
}

bool UserController::smExecute(const rclcpp::Time &stamp, const rclcpp::Duration &time_elapsed)
{
    // Get Time Elapsed Since State Change
    double time_elapsed_sec = time_elapsed.seconds();

    // Current Vehicle Location
    std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pos = this->node->vehicle_local_position;

    double current_theta = atan2(current_pos->pose.position.y - this->origin.y, current_pos->pose.position.x - this->origin.x);
    bluetooth_msgs::msg::TargetAngle msg;
    msg.vehicle_id = this->node->vehicle_id;
    msg.time = stamp;
    msg.theta = current_theta;
    this->notify_angle_pub->publish(msg);

    // Get Angular (Theta) Velocity
    double angular_vel = this->vehicle_velocity / circle_radius;
    RCLCPP_INFO(this->get_logger(), "Vehicle angular velocity: %f", angular_vel);

    // Amount of theta vehicle should have moved w.r.t start location
    this->vehicle_setpoint_theta = fmod(this->vehicle_start_theta + time_elapsed_sec * angular_vel, 2 * M_PI);
    RCLCPP_INFO(this->get_logger(), "vehicle_setpoint_theta: %f", this->vehicle_setpoint_theta);

    // Convert theta to coordinate location
    double x = this->circle_radius * cos(this->vehicle_setpoint_theta) + this->origin.x;
    double y = this->circle_radius * sin(this->vehicle_setpoint_theta) + this->origin.y;
    double z = this->height + this->origin.z;
    double yaw = this->vehicle_setpoint_theta;

    // Tell Vehicle to go to coordinate location
    this->node->sendSetpointPositionCoordinate(stamp, x, y, z, yaw);

    // Log this using the following function
    RCLCPP_INFO(this->get_logger(), "Vehicle going to (%f, %f, %f), theta: %f", x, y, z, yaw);
    /*
     *
     * Implement Your Solution Here
     *
     */

    // State Machine never exists by giving false.
    return false;
}

void UserController::handleTargetAngle(const bluetooth_msgs::msg::TargetAngle::SharedPtr s)
{
    if (!isnan(s->theta))
    {
        double theta_diff = this->vehicle_setpoint_theta - s->theta;
        this->vehicle_velocity += this->sync_angle_P * theta_diff;
        RCLCPP_INFO(this->get_logger(), "Received target angle of %f with diff %f so velocity of %f", s->theta, theta_diff, this->vehicle_velocity);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Received target angle [%f] is nan", s->theta);
    }
}

void UserController::handleReceivedBtMsg(const bluetooth_msgs::msg::BluetoothDongle::SharedPtr s)
{
    uint8_t received_id = s->failed_id;
    double received_counter = s->test_num;
    /*

        able to receive all x y position and timestamp
    send a msg to handleNotifyVehicles and reset start_target_theta

    if(this->system_vehicle_id > received_id)
        this->system_vehicle_id--;
    s->total_vehicles--;
    */

    if(!this->found_failed_node){
        if (received_id != 0){
            RCLCPP_INFO(this->get_logger(), "bt msg received from %u with counter %f", received_id, received_counter);
            bluetooth_msgs::msg::NotifyVehicles NVs;
            NVs.vehicle_id = this->system_vehicle_id;
            if (this->system_vehicle_id > received_id)
                NVs.vehicle_id--;
            NVs.total_vehicles = this->system_total_vehicles - 1;
            this->notify_vehicles_pub->publish(NVs);
            this->circle_radius = 2.0;
            this->found_failed_node = true;
        }
    }
}

void UserController::handleNotifyVehicles(const bluetooth_msgs::msg::NotifyVehicles::SharedPtr s)
{
    RCLCPP_INFO(this->get_logger(), "update start_trajectory_location");
    // if (!this->node->start_trajectory_location)
    // {
        this->system_vehicle_id = s->vehicle_id;
        this->system_total_vehicles = s->total_vehicles;
        double start_target_theta = 2 * M_PI * s->vehicle_id / s->total_vehicles;
        double startx = this->circle_radius * cos(start_target_theta) + this->origin.x;
        double starty = this->circle_radius * sin(start_target_theta) + this->origin.y;
        double startz = this->height + this->origin.z;
        double startyaw = start_target_theta;

        auto loc = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
        std::vector<double> pos = {startx, starty, startz, startyaw};
        loc->positions = pos;
        this->node->start_trajectory_location = loc;

        this->vehicle_setpoint_theta = start_target_theta;
        this->vehicle_start_theta = start_target_theta;
        this->received_circle_id = true;
        // RCLCPP_INFO(this->get_logger(), "reset start point");
    // }
}

rclcpp::Logger UserController::get_logger() { return this->node->get_logger(); }
rclcpp::Time UserController::now() { return this->node->now(); }
