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
#include "bluetooth_msgs/msg/bluetooth_dongle.hpp"

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

class BtMsgTranciver : public rclcpp::Node
{
public:
    BtMsgTranciver();
    void reset();
    int initialize();
    uint8_t bt_id;
    std::string bluetooth_address;

    // uint8_t queue[];
    // std::string env_p = std::String(std::getenv("VEHICLE_MAVLINK_SYSID"));
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", env_p);
    // BtMsgTranciver() : Node("bluetooth_msgs_Tranciver"), count_(0);

private:
    Sender BTsender;

    void timer_callback();
    void timer_callback2();
    void btMsgs_callback(const bluetooth_msgs::msg::BluetoothDongle::SharedPtr s);
    void handleReceivedBtMsg(const bluetooth_msgs::msg::BluetoothDongle::SharedPtr s);
    // std::map<uint8_t, sequence_timeout> umap;

    std::queue<uint8_t> uq;
    std::map<uint8_t, double> umap2;

    rclcpp::Subscription<bluetooth_msgs::msg::BluetoothDongle>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bluetooth_msgs::msg::BluetoothDongle>::SharedPtr publisher_;

    // std::vector<uint8_t> p_data;
    double count_;
    uint8_t node_total_num;
};

#endif