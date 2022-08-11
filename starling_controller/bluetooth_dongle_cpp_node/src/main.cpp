#include "main.hpp"

BtMsgTranciver::BtMsgTranciver() : Node("bluetooth_msgs_tranciver")
{
    count_ = 0.0;
    node_total_num = 0;
    RCLCPP_INFO(this->get_logger(), "---------start here--------------");
    this->declare_parameter("bluetooth_address", "/dev/ttyACM0");
    this->declare_parameter("bt_id", 1);
    this->get_parameter("bluetooth_address", bluetooth_address);
    this->get_parameter("bt_id", bt_id);

    this->publisher_ = this->create_publisher<bluetooth_msgs::msg::BluetoothDongle>("bluetooth_receive", 1);
    this->subscription_ = this->create_subscription<bluetooth_msgs::msg::BluetoothDongle>("bluetooth_", 10,
                                                                                          std::bind(&BtMsgTranciver::btMsgs_callback, this, _1));
    this->BTsender = Sender();
    RCLCPP_INFO(this->get_logger(), "BTsender created");
    initialize();
}
// void BtMsgTranciver::reset()
// {
//     this->BTsender.Start_or_Stop_Atomic(0); // stop
//     this->BTsender.reset();
//     this->subscription_ = nullptr;
//     this->publisher_ = nullptr;
//     this->timer_ = nullptr;
// }
int BtMsgTranciver::initialize()
{
    // this->BTsender.Start_or_Stop_Atomic(0);
    // RCLCPP_INFO(this->get_logger(), "BTsender stutus: '%s'", this->BTsender.connection_status.c_str());
    // this->BTsender.reset();
    RCLCPP_INFO(this->get_logger(), "connect to serial port: %s", bluetooth_address.c_str());
    RCLCPP_INFO(this->get_logger(), "serial port number: %d", ++bt_id);
    this->BTsender.init(bt_id, bluetooth_address);
    // umap.insert(pair<uint8_t, sequence_timeout>(bt_id, {0, 0}));
    // if (this->BTsender.connection_status == "EXIT_FAILURE")
    // {
    // RCLCPP_INFO(this->get_logger(), "BTsender EXIT_FAILURE with ret: %d", ret);
    // exit(EXIT_FAILURE);
    // }
    // RCLCPP_INFO(this->get_logger(), "BTsender stutus: '%s'", this->BTsender.connection_status.c_str());
    this->timer_ = this->create_wall_timer(
        300ms, std::bind(&BtMsgTranciver::timer_callback, this));
    // this->BTsender.Start_or_Stop_Atomic(true); // start
    // RCLCPP_INFO(this->get_logger(), "Publishing STATUS: '%s'", this->BTsender.connection_status.c_str());
    // this->timer_callback();
    return 0;
}

void BtMsgTranciver::timer_callback()
{
    double nanosecs = this->now().nanoseconds();
    double secs = this->now().seconds();
    printf("nano: %f \n", nanosecs);
    printf("sec : %f \n", secs);
    this->BTsender.receive_single_packet(nanosecs);
    if (this->BTsender.lost_nodes.size() != 0)
    {
        printf("lost node detected: %d", this->BTsender.lost_nodes.front());
    }
}

void BtMsgTranciver::btMsgs_callback(const bluetooth_msgs::msg::BluetoothDongle::SharedPtr s)
{
    // RCLCPP_INFO(this->get_logger(), "vehicle msg from id: '%d'", s->bt_id);
    // call decision function or publish something
}

/*
蓝牙信号强度决定timeout阈值
概率丢包计算最短检测周期


*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BtMsgTranciver>());
    rclcpp::shutdown();
    return 0;
}