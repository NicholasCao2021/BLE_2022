#ifndef SENDER_H
#define SENDER_H
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <string>
#include <ctype.h>
#include <sys/time.h>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>

#include "slip.h"
#include "serialport.h"
#include "main.hpp"

/*----------------------------------------------------------------------------*/
#define VERSION "0.92"
#define DEFAULT_EXIT_TIME_SEC 1
#define DEFAULT_EXIT_TIME_FOREVER 0
#define bool unsigned char
#define true !0
#define false 0

// #define NUM_NODE 2
#define TIMEOUT_TREADHOLD 4
#define BYTE_NUM_OF_STATE 1
#define BYTE_NUM_OF_UINT8 1
#define BYTE_NUM_OF_INT 4
#define BYTE_NUM_OF_DOUBLE 8
#define BOTHNODEANDBLE 0x10
#define ONLYBLUETOOTH 0x11

/*----------------------------------------------------------------------------*/

using namespace std;

struct sequence_timeout
{
    uint8_t Timestamp[BYTE_NUM_OF_DOUBLE],position_x[BYTE_NUM_OF_DOUBLE],position_y[BYTE_NUM_OF_DOUBLE], state, timeout;
};

class BtMsgTranciver;

class Sender
{

public:
    Sender(BtMsgTranciver *Node);
    BtMsgTranciver *node;
    rclcpp::Logger get_logger();
    rclcpp::Time now();
    void reset();
    int init(uint8_t node_id, std::string addr);
    void receive_single_packet(uint8_t *data_byte1,uint8_t *data_byte2, uint8_t data_len1, uint8_t data_len2);
    void update_data_map(std::vector<uint8_t> p_data, uint8_t *data_byte1,uint8_t *data_byte2, uint8_t data_len1, uint8_t data_len2);
    void send_packet(uint8_t data_len, uint8_t *data);
    void stop_atomic();

    uint8_t msg_len;
    uint8_t rx_buf[SERIAL_MSG_LEN];
    serial_msg_t *rx_pkt; // = (serial_msg_t *)rx_buf

    // data map
    std::map<uint8_t, sequence_timeout> umap;
    std::vector<uint8_t> p_data;

    uint8_t node_id;
    std::string addr;
    int counter = 1;
    uint16_t period; // ms
    std::vector<int> lost_nodes;
    std::vector<int> resecure_nodes;
};
#endif