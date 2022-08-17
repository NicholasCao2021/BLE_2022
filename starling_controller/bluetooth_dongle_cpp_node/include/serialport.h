#pragma once

#define SERIAL_MSG_LEN 1024
#define SERIAL_SLIP_BUF_LEN (2 * SERIAL_MSG_LEN + 1)

#define SERIAL_MSG_CRC_LEN 2
#define SERIAL_MSG_OVERHEAD (sizeof(serial_msg_t) + SERIAL_MSG_CRC_LEN)

#define RSP_FLAG_OK 0x00000001
#define RSP_FLAG_ERROR 0x00000002
#define RSP_FLAG_PATTERN 0x00000004 // pattern is in pattern_buf

typedef enum
{
  // host -> device
  SERIAL_MSG_CMD_PING = 0x01,
  SERIAL_MSG_CMD_INFO = 0x02,
  SERIAL_MSG_CMD_RESET = 0x03,
  SERIAL_MSG_CMD_MAC_CFG_READ = 0x10,
  SERIAL_MSG_CMD_MAC_CFG_WRITE = 0x11,
  SERIAL_MSG_CMD_MAC_SET_STATE = 0x12,
  SERIAL_MSG_CMD_ATOMIC_SEND = 0x13,
  SERIAL_MSG_CMD_ATOMIC_POLL = 0x14,
  SERIAL_MSG_CMD_MAC_GET_STATE = 0x15,
  SERIAL_MSG_CMD_APPMESH_INFO = 0x20,
  SERIAL_MSG_CMD_APPMESH_CFG_READ = 0x21,
  SERIAL_MSG_CMD_APPMESH_CFG_WRITE = 0x22,
  SERIAL_MSG_CMD_APPMESH_PUBLISH = 0x23,
  SERIAL_MSG_CMD_GALLOP_CONFIG_INIT = 0x30,
  SERIAL_MSG_CMD_GALLOP_ADD_NBR_TBL_ROW = 0x31,
  SERIAL_MSG_CMD_GALLOP_DROP_SCHED = 0x32,
  SERIAL_MSG_CMD_GALLOP_QUEUE_DATA = 0x33,
  SERIAL_MSG_CMD_GALLOP_POLL_DATA = 0x34,
  SERIAL_MSG_CMD_GALLOP_INFO = 0x35,

  // device -> host
  SERIAL_MSG_RSP_PING = 0xa1,
  SERIAL_MSG_RSP_INFO = 0xa2,
  SERIAL_MSG_RSP_OK = 0xa3,
  SERIAL_MSG_RSP_DATA_QUEUED = 0xa4,
  SERIAL_MSG_RSP_ERROR = 0xfe,
  SERIAL_MSG_RSP_UNKNOWN_CMD = 0xff,
  SERIAL_MSG_RSP_MAC_CFG_READ = 0xb0,
  SERIAL_MSG_RSP_ATMIC_DATA_RX = 0xb1,
  SERIAL_MSG_RSP_ATOMIC_DONE = 0xb2,
  SERIAL_MSG_RSP_MAC_STATE = 0xb3,
  SERIAL_MSG_RSP_APPMESH_INFO = 0xD0,
  SERIAL_MSG_RSP_APPMESH_CFG_READ = 0xD1,
  SERIAL_MSG_RSP_APPMESH_RXMSG = 0xD3,
  SERIAL_MSG_REPLY_GALLOP_DATA_RX = 0xE0,
  SERIAL_MSG_REPLY_GALLOP_DONE = 0xE1,
  SERIAL_MSG_REPLY_GALLOP_INFO = 0xE2,
  SERIAL_MSG_REPLY_GALLOP_STATE = 0xE3,

} serial_cmd_t;

#define __packed_gcc __attribute__((packed))

// Serial link packets

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    uint8_t msg_type;
    uint8_t len;
    uint8_t payload[];
  } __packed_gcc serial_msg_t;

  typedef struct
  {
    uint8_t buf[SERIAL_MSG_LEN];
    uint16_t len;
    // TODO IN FUTURE    int16_t       at_index;  /* -ve will mean offset from end */
  } pattern_match_t;

  extern uint32_t rsp_flags;
  extern pattern_match_t pattern_match;

  int serial_setup(const char *portname);
  void serial_close();
  void send_slip_packet(uint8_t msg_type, uint8_t len, uint8_t *payload);
  void rx_slip_packet(uint8_t *msg_len, uint8_t *msg_payload);

#ifdef __cplusplus
}
#endif
