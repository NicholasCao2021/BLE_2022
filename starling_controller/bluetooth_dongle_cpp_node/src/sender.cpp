
#include "sender.hpp"

Sender::Sender(BtMsgTranciver *Node)
{
	this->node = Node;
	umap.clear();
	this->msg_len = 0;
	this->node_id = 0;
	this->addr = "";
	this->counter = 0;
	this->period = 30; // ms
					   // std::string connection_status;
}

int Sender::init(uint8_t id, std::string addr)
{
	// stop_atomic();
	// delay(100);
	rx_pkt = (serial_msg_t *)rx_buf;
	period = 30;
	// init()
	setbuf(stdout, NULL);

	int ret = -1;
	ret = serial_setup(addr.c_str());
	node_id = id;

	if (ret != 0)
	{
		return ret;
		// exit(EXIT_FAILURE);
	}

	// uint8_t counter = 1;
	// uint16_t period = 40; // ms

	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	   Set MAC configuration
	   Offset
		0   : MAC identification
		=00 Atomic
		1   : Config Item ID
		=00 Deployment Set A
			Offset
			2   : node-id of this node
			3   : node_id of timesync
			4   : Pattern Type
				=00 NONE
				=01 P2P
				=02 P2MP
				=03 MP2P
				=04 MP2MP
				=05 CONTROL
			5   : Max Message length
		 6..7   : Atomic Period in milliseconds (LSB in 6)
				   - set 0 for default value (1000ms)
				   - Valid range 1 to 5000
		8..15   : Future use - set to 00
		   16   : Number of sources
		 s..s   : array of bytes, length specified by offset 16, which are Source node IDs
		1+(16)  : Number of destination - (n) means content of offset n
		 d..d   : array of bytes, length specified by offset (1+(16)), which are destination node IDs
	*/

	uint8_t init_cmd[26] = {0x00, 0x00, node_id, 0x01, 0x02, 0x7f, period & 0xFF, (period >> 8) & 0xFF,
							// reserved
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							// sources
							0x04, 0x01, 0x02, 0x03,0x04,
							// destinations
							0x04, 0x01, 0x02, 0x03,0x04};

	// Initialise MAC configuration
	send_slip_packet(SERIAL_MSG_CMD_MAC_CFG_WRITE, sizeof(init_cmd), init_cmd);

	// Wait for OK
	do
	{
		rx_slip_packet(&msg_len, rx_buf);
	} while (rx_buf[0] != SERIAL_MSG_RSP_OK);

	// Start_or_Stop_Atomic()
	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	   Start or stop up either as a node or time sync
	   Offset
		0   : MAC identification
		=00 Atomic
		1   : start/stop
		=00 Stop
		=01 Start
	*/

	uint8_t start_cmd[2] = {0x00, 0x01};

	// Start up MAC
	send_slip_packet(SERIAL_MSG_CMD_MAC_SET_STATE, sizeof(start_cmd), start_cmd);

	// Wait for OK
	do
	{
		rx_slip_packet(&msg_len, rx_buf);
	} while (rx_pkt->msg_type != SERIAL_MSG_RSP_OK);

	return 0;
}
void Sender::reset()
{
	node_id = 0;
	addr = "";
	counter = 0;
	period = 30; // ms
	// connection_status = "Reset";
	stop_atomic();
}

void Sender::stop_atomic()
{
	uint8_t start_cmd[2] = {0x00, 0x00};

	// Start up MAC
	send_slip_packet(SERIAL_MSG_CMD_MAC_SET_STATE, sizeof(start_cmd), start_cmd);

	// Wait for OK
	do
	{
		rx_slip_packet(&msg_len, rx_buf);
	} while (rx_pkt->msg_type != SERIAL_MSG_RSP_OK);
}

void Sender::receive_single_packet(uint8_t *data_byte1,uint8_t *data_byte2, uint8_t data_len1, uint8_t data_len2)
{
	// Poll Atomic and wait for responses
	send_slip_packet(SERIAL_MSG_CMD_ATOMIC_POLL, 0, NULL);

	// Wait for incoming packets
	do
	{
		rx_slip_packet(&msg_len, rx_buf);
	} while (rx_pkt->msg_type != SERIAL_MSG_RSP_ATOMIC_DONE);

	// uint8_t num_packets = rx_pkt->payload[1];
	// printf("Got %d packets!\n", num_packets);

	// Receive all DATA_RX packets
	do
	{
		rx_slip_packet(&msg_len, rx_buf);

		/* Sends packet data TO the SLIP interface in the following format:
		   Byte
		   0x00 - SERIAL_MSG_RSP_ATMIC_DATA_RX (0xb1)
		   0x01 - fragment no. / total
		   0x02 - packet id LSB
		   0x03 - packet id MSB
		   0x04 - source ID
		   0x05 - Payload starts
		*/

		if (rx_pkt->msg_type == SERIAL_MSG_RSP_ATMIC_DATA_RX && rx_pkt->len >= 6)
		{
			// printf("sender :: rx_pkt->msg_type %d msg_len %d\n", rx_pkt->msg_type, rx_pkt->len);
			// printf("Got data len: %u from node %u!\n", rx_pkt->len, rx_pkt->payload[4]);
			// RCLCPP_INFO(this->get_logger(), "Got data from node %u!", rx_pkt->payload[4]);
			for(auto it=this->lost_nodes.begin();it!=this->lost_nodes.end();){
				if(*it==rx_pkt->payload[4]){
					// RCLCPP_INFO(this->get_logger(), "lost node %d reconnected!", *it);
					it = this->lost_nodes.erase(it);
					break;
				}else{
					++it;
				}
			}
			if (umap.find(rx_pkt->payload[4]) == umap.end())
			{
				
				umap.insert(pair<uint8_t, sequence_timeout>(rx_pkt->payload[4], {{0, 0, 0, 0, 0, 0, 0, 0}, 1, 0}));
			}
			// printf("umap element size: %d\n", umap.size());
			printf("received raw data with size %d: ",umap.size() * 2 + data_len1 + data_len2 + 1);
			for (auto i = 4; i < (umap.size() * 2 + data_len1 + data_len2 + 4 + 1); i++)
			{
				p_data.push_back(rx_pkt->payload[i]);
				printf("%u ", rx_pkt->payload[i]);
			}
			// printf("data end size:%ld umap.size():%u \n", p_data.size(), umap.size());
		}

	} while (rx_pkt->msg_type != SERIAL_MSG_RSP_OK);
	// RCLCPP_INFO(this->get_logger(), "");
	// RCLCPP_INFO(this->get_logger(), "============ update map ==========");
	double secs = this->now().seconds();
	update_data_map(p_data, data_byte1,data_byte2, data_len1,data_len2);
	double elps_time = this->now().seconds() - secs;
    // RCLCPP_INFO(this->get_logger(), "update process time : %f", elps_time);
	p_data.clear();
}

void Sender::update_data_map(std::vector<uint8_t> p_data, uint8_t *data_byte1,uint8_t *data_byte2, uint8_t data_len1, uint8_t data_len2)
{
	// printf("received data: ");
	// for (auto e : p_data)
	// printf("%u ", e);
	// printf("data end size:%ld umap.size():%u \n", p_data.size(), umap.size());
	// printf("packet id: %u %u\n", BTsender.rx_pkt->payload[3], BTsender.rx_pkt->payload[2]);
	double x, y;
	for (auto i = 0; i < p_data.size(); i += (umap.size() * 2 + data_len1 + data_len2 + 1))
	{
		uint8_t id_num = p_data[i]; // node_id
		// memcpy(timestamp, &p_data[i+1], sizeof(timestamp));

		// uint8_t id_LSB = p_data[i + 1]; // node_LSB
		// uint8_t id_MSB = p_data[i + 2]; // node_MSB
		// uint8_t id_MSB = p_data[i + 3]; // node_MSB
		// uint8_t id_MSB = p_data[i + 4]; // node_MSB
		// RCLCPP_INFO(this->get_logger(), "current node id: %u", node_id);
		// printf("msg from id: %u, seq:%u %u\n", id_num, id_MSB, id_LSB);
		// printf("msg type %u msg_len: %u msg_payload %u\n", rx_pkt->msg_type, rx_pkt->len, rx_pkt->payload[0]);
		for (auto it = umap.begin(); it != umap.end(); ++it)
		{
			if (it->first != id_num)
			{
				it->second.timeout++;
				// printf("node now: %u node id: %u timeout++ : %u\n", id_num, it->first, it->second.timeout);
			}
		}
		auto fd = umap.find(id_num);
		if (fd != umap.end())
		{
			memcpy(&fd->second, &p_data[i +  1], BYTE_NUM_OF_DOUBLE);
			memcpy(&fd->second.position_x, &p_data[i + BYTE_NUM_OF_DOUBLE + 1],     BYTE_NUM_OF_DOUBLE);
			memcpy(&fd->second.position_y, &p_data[i + 2 * BYTE_NUM_OF_DOUBLE + 1], BYTE_NUM_OF_DOUBLE);
			fd->second.state = p_data[i + data_len1 + 1];
			// RCLCPP_INFO(this->get_logger(), "receive msg BLE state position: %u state: %u", i + data_len1 + 1,fd->second.state);
			if(fd->second.state == ONLYBLUETOOTH){
				// if(this->resecure_nodes.back() != id_num){
					this->resecure_nodes.push_back(id_num);
					printf("\n node %u added",id_num);
				// }
					

			}
			x = *reinterpret_cast<double *>(&fd->second.position_x);
			y = *reinterpret_cast<double *>(&fd->second.position_y);
			// fd->second.Timestamp = timestamp;
			fd->second.timeout = 0;
		}
		// else if (id_num != 0)
		// {
		// 	umap.insert(pair<uint8_t, sequence_timeout>(id_num, {{0, 0, 0, 0, 0, 0, 0, 0}, 1, 0}));
		// 	umap.size()++;
		// }

		for (size_t j = data_len1 * BYTE_NUM_OF_DOUBLE + data_len2 * BYTE_NUM_OF_UINT8 + 1; j < (umap.size() * 2 + data_len1 * BYTE_NUM_OF_DOUBLE + data_len2 * BYTE_NUM_OF_UINT8 + 1); j += 2)
		{
			uint8_t node_id_other = p_data[j + i];
			uint8_t node_id_other_timeout = p_data[j + i + 1];
			// printf("node_id_other %u node_id_other_timeout %u\n", node_id_other, node_id_other_timeout);
			fd = umap.find(node_id_other);
			if (fd != umap.end() && node_id_other_timeout > TIMEOUT_TREADHOLD && fd->second.timeout > TIMEOUT_TREADHOLD)
			{
				// double dif = data_byte - *reinterpret_cast<double *>(fd->second.Timestamp);
				// printf("found failed node id: %u in %lf nanosecond\n", node_id_other, dif);
				// if(this->lost_nodes.back() != node_id_other)
					this->lost_nodes.push_back(node_id_other);
				fd->second.timeout = 30;
			}
		}
		RCLCPP_INFO(this->get_logger(), "receive msg from id: %u at position (%f,%f)", id_num,x,y);
	}
	p_data.clear();
	// for (auto e : umap)
	// {
	// 	printf("id=%u timeout ", e.first);
	// 	for (auto c : e.second.Timestamp)
	// 		printf("%u ", c);
	// 	printf("state=%u timeout=%u \n", e.second.state, e.second.timeout);
	// }

	/* Sends packet data TO other nodes in the following format:
	   Byte
	   0x00 - LSB self segment id from 0 to FFFF
	   0x01 - MSB self segment id from 0 to FFFF

	   // other node data from last receving
	   0x02 - other node id (eg. 2)
	   0x03 - recorded timeout value in umap

	   //other node 2 if exist
	   0x04 - (eg. 3)
	   0x05 - recorded timeout value in umap
	   ....

	*/
	// RCLCPP_INFO(this->get_logger(), "timestamp: %f", *reinterpret_cast<double *>(data_byte1));
	// char *byteArray = reinterpret_cast<char *>(&data_byte);
	// for(auto i = 0)
	// for (auto i = 0; i < BYTE_NUM_OF_DOUBLE; i++)
	// {
	// 	p_data.push_back(*byteArray);
	// 	byteArray++;
	// }
	printf("\ndata to transfer               : ");
	for(auto i = 0;i<data_len1;++i){
	    // char *byteArray = reinterpret_cast<char *>(&data_byte[i]);
			p_data.push_back(*data_byte1);
			// printf("%d ", *data_byte);
			data_byte1++;
	}
	for(auto i = 0;i<data_len2;++i){
	    // char *byteArray = reinterpret_cast<char *>(&data_byte[i]);
			p_data.push_back(*data_byte2);
			// printf("%d ", *data_byte);
			data_byte2++;
	}
	// printf("\n");
	// p_data.push_back(*byteArray++);
	for (auto it = umap.begin(); it != umap.end(); ++it)
	{
		if (it->first != node_id)
		{
			p_data.push_back(it->first);
			p_data.push_back(it->second.timeout);
			// p_data.push_back(it->second.MSB_seq);
			// printf("insert id:%u timeout:%u \n", it->first, it->second.timeout);
		}
	}
	//, 0x02, 0x55, 0x66
	// counter++;
	for (auto e : p_data)
		printf("%u ", e);
	printf("\n");
	// byteArray = reinterpret_cast<char *>(&data_byte);
	// double final = *reinterpret_cast<double *>(byteArray);
	// printf("%lf\n", final);
	send_packet(p_data.size(), &p_data[0]);
}

void Sender::send_packet(uint8_t data_len, uint8_t *data)
{
	/* Sends packet data FROM the SLIP interface in the following format:
	   Byte
	   0x00     - no. of packets
	   0x01     - length of each packet
	   0x02-END - packets
	 */

	uint8_t resp_pkt[data_len + 2] = {0x01, data_len};
	for (auto i = 2; i < data_len + 2; i++)
	{
		resp_pkt[i] = *data;
		data++;
	}

	// Send some data back
	send_slip_packet(SERIAL_MSG_CMD_ATOMIC_SEND, sizeof(resp_pkt), resp_pkt);

	// Wait for queue OK
	do
	{
		rx_slip_packet(&msg_len, rx_buf);
	} while (rx_pkt->msg_type != SERIAL_MSG_RSP_OK);
}

rclcpp::Logger Sender::get_logger() { return this->node->get_logger(); }
rclcpp::Time Sender::now() { return this->node->now(); }
// /* we should never reach here */
// return 100000;
