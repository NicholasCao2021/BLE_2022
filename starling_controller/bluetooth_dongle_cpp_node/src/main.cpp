#include "main.hpp"

BtMsgTranciver::BtMsgTranciver() : Node("bluetooth_msgs_tranciver")
{
    count_ = 0.0;
    node_total_num = 0;
    RCLCPP_INFO(this->get_logger(), "---------start here--------------");
    // this->declare_parameter("bluetooth_address", "/dev/ttyACM0");
    this->declare_parameter("bt_id", 1);
    // this->get_parameter("bluetooth_address", bluetooth_address);
    this->get_parameter("bt_id", bt_id);

    this->publisher_ = this->create_publisher<bluetooth_msgs::msg::BluetoothDongle>("bluetooth_receive", 1);
    this->notify_vehicles_pub = this->create_publisher<bluetooth_msgs::msg::NotifyVehicles>("notify_vehicles", 1);

    this->vehicle_info_request_sub = this->create_subscription<std_msgs::msg::Empty>("/monitor/vehicle_info", 10,std::bind(&BtMsgTranciver::vehicle_info_callback, this, _1));
    // this->subscription_ = this->create_subscription<bluetooth_msgs::msg::BluetoothDongle>("bluetooth_", 10,
    //                                                                                       std::bind(&BtMsgTranciver::btMsgs_callback, this, _1));
    this->sender = std::make_shared<Sender>(this);
    RCLCPP_INFO(this->get_logger(), "sender created");
    initialize();
}
// void BtMsgTranciver::reset()
// {
//     this->sender.Start_or_Stop_Atomic(0); // stop
//     this->sender.reset();
//     this->subscription_ = nullptr;
//     this->publisher_ = nullptr;
//     this->timer_ = nullptr;
// }
int BtMsgTranciver::initialize()
{
    int id = int(bt_id)-1;
    std::string s = "/dev/ttyACM"+std::to_string(id);
    RCLCPP_INFO(this->get_logger(), "connect to serial port: %s", s.c_str());
    RCLCPP_INFO(this->get_logger(), "serial port number: %d", bt_id);
    this->sender->init(bt_id, s);
    RCLCPP_INFO(this->get_logger(), "sender %u initialised",bt_id);
    this->timer_ = this->create_wall_timer(
        0.1ms, std::bind(&BtMsgTranciver::timer_callback, this));
    // this->timer_dump_ = this->create_wall_timer(
    //     1ms, std::bind(&BtMsgTranciver::timer_callback_dump, this));
    // this->timer_checker_ = this->create_wall_timer(
    //     10ms, std::bind(&BtMsgTranciver::timer_callback_dump, this));
    // this->sender.Start_or_Stop_Atomic(true); // start
    // RCLCPP_INFO(this->get_logger(), "Publishing STATUS: '%s'", this->sender.connection_status.c_str());
    // this->timer_callback();
    return 0;
}

void BtMsgTranciver::timer_callback()
{
    double nanosecs = this->now().nanoseconds();
    double secs = this->now().seconds();
    // RCLCPP_INFO(this->get_logger(), "nano: %f", nanosecs);
    // RCLCPP_INFO(this->get_logger(), "sec : %f", secs);
    double elps_time1 = this->now().seconds() - secs;
    // RCLCPP_INFO(this->get_logger(), "one flost process time : %f", elps_time1);
    this->sender->receive_single_packet(nanosecs);
    double elps_time = this->now().seconds() - secs;
    RCLCPP_INFO(this->get_logger(), "total process time : %f", elps_time);
    bluetooth_msgs::msg::BluetoothDongle msg;
    if (this->sender->lost_nodes.size() != 0)
    {
        // printf("lost node detected: %d", this->sender.lost_nodes.front());
        RCLCPP_INFO(this->get_logger(), "lost node detected: %d", this->sender->lost_nodes.front());
        msg.failed_id = this->sender->lost_nodes.front();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "no failed node");
        msg.failed_id = 0;
    }
    msg.test_num = this->now().nanoseconds();
    this->publisher_->publish(msg);
}

void BtMsgTranciver::vehicle_info_callback(const std_msgs::msg::Empty::SharedPtr s){
    (void)s;
    bluetooth_msgs::msg::NotifyVehicles NV;
    NV.vehicle_id = this->bt_id;
    NV.total_vehicles = 3;
    this->notify_vehicles_pub->publish(NV);
}

// void BtMsgTranciver::btMsgs_callback(const bluetooth_msgs::msg::BluetoothDongle::SharedPtr s)
// {
//     // RCLCPP_INFO(this->get_logger(), "vehicle msg from id: '%d'", s->bt_id);
//     // call decision function or publish something
// }

/*
蓝牙信号强度决定timeout阈值
概率丢包计算最短检测周期


*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BtMsgTranciver>()); // spin_some, spin_until_future_complete
    rclcpp::shutdown();
    return 0;
}