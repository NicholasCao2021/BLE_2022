#ifndef BTMSGTRANCIVER_H
#define BTMSGTRANCIVER_H
#include <chrono>
#include <memory>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <cmath>
#include <queue>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "bluetooth_msgs/msg/bluetooth_dongle.hpp"
#include "bluetooth_msgs/msg/notify_vehicles.hpp"



#include "sender.hpp"

using namespace std;
using namespace rclcpp;
using namespace std::chrono_literals;
using std::placeholders::_1;

// struct sequence
// {
//     int seq, state;
// };

// struct sequence_timeout
// {
//     uint8_t MSB_seq, LSB_seq, state, timeout;
// };
class Sender;

class BtMsgTranciver : public rclcpp::Node
{
public:
    BtMsgTranciver();
    void reset();
    int initialize();
    
    std::string bluetooth_address;
    uint8_t bt_id;
    uint8_t state_of_BLE;

    std::shared_ptr<geometry_msgs::msg::PoseStamped> vehicle_local_position;
    // uint8_t queue[];
    // std::string env_p = std::String(std::getenv("VEHICLE_MAVLINK_SYSID"));
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", env_p);
    // BtMsgTranciver() : Node("bluetooth_msgs_Tranciver"), count_(0);

private:
    std::shared_ptr<Sender> sender;
    
    void timer_callback();
    void vehicle_info_callback(const std_msgs::msg::Empty::SharedPtr s);
    void btMsgs_callback(const bluetooth_msgs::msg::BluetoothDongle::SharedPtr s);
    void bluetooth_stop_callback(const std_msgs::msg::Empty::SharedPtr s);
    void handleReceivedBtMsg(const bluetooth_msgs::msg::BluetoothDongle::SharedPtr s);
    void handleLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr s);
    // std::map<uint8_t, sequence_timeout> umap;


    std::map<uint8_t, double> umap2;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_dump_;
    rclcpp::TimerBase::SharedPtr timer_checker_;

    

    rclcpp::Publisher<bluetooth_msgs::msg::BluetoothDongle>::SharedPtr publisher_;
    rclcpp::Publisher<bluetooth_msgs::msg::NotifyVehicles>::SharedPtr notify_vehicles_pub;
     
    rclcpp::Subscription<bluetooth_msgs::msg::BluetoothDongle>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr vehicle_info_request_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub;

    // std::vector<uint8_t> p_data;
    double count_;
    uint8_t node_total_num;
};

#endif