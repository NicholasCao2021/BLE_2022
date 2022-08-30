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
    this->emergency_sub = this->create_subscription<std_msgs::msg::Empty>("emergency_stop_self", 1, std::bind(&BtMsgTranciver::bluetooth_stop_callback, this, _1));
    
    
    this->local_position_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", 10,
        std::bind(&BtMsgTranciver::handleLocalPosition, this, std::placeholders::_1));

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
    // this->vehicle_local_position->pose.position.x = 0;
    // this->vehicle_local_position->pose.position.y = 0;
    this->vehicle_local_position = nullptr;
    this->state_of_BLE = BOTHNODEANDBLE;
    bt_id--;
    std::string s = "/dev/ttyACM"+std::to_string(bt_id);
    RCLCPP_INFO(this->get_logger(), "connect to serial port: %s", s.c_str());
    try{
    this->sender->init(++bt_id, s);
    }
    catch (const std::exception &e)
    {
        string message = e.what();
        double ts = this->now().seconds();
        RCLCPP_ERROR(this->get_logger(), "%s at %lf", message.c_str(),ts);
    }
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
    try{
        double a[]= {nanosecs,0.0,0.0};
        uint8_t b[] = {this->state_of_BLE};
        if(this->vehicle_local_position != nullptr){
            a[2] = this->vehicle_local_position->pose.position.x;
            a[3] = this->vehicle_local_position->pose.position.y;
            RCLCPP_INFO(this->get_logger(), "position (%f,%f)", a[2],a[3]);
        }
        uint8_t *byteArray1 = reinterpret_cast<uint8_t *>(a);
        uint8_t *byteArray2 = reinterpret_cast<uint8_t *>(b);
        this->sender->receive_single_packet(byteArray1,byteArray2,sizeof(a),sizeof(b));
    }
    catch (const std::exception &e)
    {
        string message = e.what();
        double ts = this->now().seconds();
        RCLCPP_ERROR(this->get_logger(), "%s at %lf", message.c_str(),ts);
    }



    double elps_time = this->now().seconds() - secs;
    // RCLCPP_INFO(this->get_logger(), "total process time : %f", elps_time);
    bluetooth_msgs::msg::BluetoothDongle msg;
    if (this->sender->lost_nodes.size() != 0)
    {
        // printf("lost node detected: %d", this->sender.lost_nodes.front());
        RCLCPP_INFO(this->get_logger(), "lost node detected: %d", this->sender->lost_nodes.front());
        msg.failed_id = this->sender->lost_nodes.front();
    }
    else if(this->sender->resecure_nodes.size() != 0)
    {
        RCLCPP_INFO(this->get_logger(), "This Node: %d needs to rescure",bt_id);
        msg.failed_id = bt_id;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "no failed node");
        msg.failed_id = 0;
        // this->sender->umap.find(id_num)
    }
    
    msg.test_num = this->now().nanoseconds();
    // msg.node_id = int(bt_id)-1;
    // msg.position_x = this->vehicle_local_position->pose.position.x;
    // msg.position_y = this->vehicle_local_position->pose.position.y;
    this->publisher_->publish(msg);
}

void BtMsgTranciver::vehicle_info_callback(const std_msgs::msg::Empty::SharedPtr s){
    (void)s;
    bluetooth_msgs::msg::NotifyVehicles NV;
    NV.vehicle_id = this->bt_id;
    NV.total_vehicles = 3;
    this->notify_vehicles_pub->publish(NV);
}


void BtMsgTranciver::handleLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr s)
{
    this->vehicle_local_position = s;
    RCLCPP_INFO(this->get_logger(), "X:%lf Y:%lf", this->vehicle_local_position->pose.position.x,this->vehicle_local_position->pose.position.y);
}

void BtMsgTranciver::bluetooth_stop_callback(const std_msgs::msg::Empty::SharedPtr s)
{
    (void)s;
    this->state_of_BLE = ONLYBLUETOOTH;
    RCLCPP_INFO(this->get_logger(), "bluetooth_stop_callback detected");
    // call decision function or publish something
}

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