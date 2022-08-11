#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <cmath>

#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <libInterpolate/Interpolate.hpp>
#include <libInterpolate/AnyInterpolator.hpp>

// Include custom ros msgs from bluetooth_msgs
#include "bluetooth_msgs/msg/notify_vehicles.hpp"
#include "bluetooth_msgs/msg/target_angle.hpp"
#include "bluetooth_msgs/msg/bluetooth_dongle.hpp"

using namespace std;
using namespace rclcpp;

// Forward Declare Parent UAVController class to avoid cyclic include
class UAVController;

class UserController
{
public:
    UserController(UAVController *node);

    // Reset this controller
    void reset();

    // User Controller Checking If The Start Location is Registered
    bool smReady(const rclcpp::Time &stamp);

    // User Controller Execute One Control Loop
    bool smExecute(const rclcpp::Time &stamp, const rclcpp::Duration &time_elapsed);

private:
    // ROS Helper Functions
    rclcpp::Logger get_logger();
    rclcpp::Time now();

    // The ROSnode itself
    UAVController *node;

    // Publishers
    rclcpp::Publisher<bluetooth_msgs::msg::TargetAngle>::SharedPtr notify_angle_pub;

    // Subscriptions
    rclcpp::Subscription<bluetooth_msgs::msg::TargetAngle>::SharedPtr sync_angle_sub;
    rclcpp::Subscription<bluetooth_msgs::msg::NotifyVehicles>::SharedPtr notify_vehicles_sub;
    rclcpp::Subscription<bluetooth_msgs::msg::BluetoothDongle>::SharedPtr bluetooth_dongle_sub;

    //// Functionality

    // Handle Subscription Callbacks
    void handleTargetAngle(const bluetooth_msgs::msg::TargetAngle::SharedPtr s);
    void handleNotifyVehicles(const bluetooth_msgs::msg::NotifyVehicles::SharedPtr s);
    void handleReceivedBtMsg(const bluetooth_msgs::msg::BluetoothDongle::SharedPtr s);

    // Variables
    geometry_msgs::msg::Point origin;
    const double circle_radius = 1.5; // In meters
    const double height = 1.0;
    uint8_t system_vehicle_id;
    uint8_t system_total_vehicles;

    // Received initial positions
    bool received_circle_id = false;

    // Dynamic Variables
    double vehicle_start_theta;
    double vehicle_setpoint_theta;

    // Control on velocity
    double sync_angle_P;
    double vehicle_velocity = 0.5;
};

#endif