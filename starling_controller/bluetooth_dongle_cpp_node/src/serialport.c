#include <signal.h> // for catching exit signals
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#include "slip.h"
#include "serialport.h"

#define notVERBOSE_PRINT_CRC

#define PKT_FORMAT_MSGIDpLEN 0

static uint8_t serial_tx_buf[SERIAL_MSG_LEN];
static uint8_t serial_slip_tx_buf[SERIAL_SLIP_BUF_LEN];
static uint8_t serial_slip_rx_buf[SERIAL_SLIP_BUF_LEN];

uint32_t rsp_flags;
pattern_match_t pattern_match;

static int ser_fd;

slip_t slip_rx = {.p_buffer = serial_slip_rx_buf, .current_index = 0, .buffer_len = SERIAL_SLIP_BUF_LEN, .state = SLIP_STATE_DECODING};

#define SLIP_BYTE_END 0300

int set_interface_attribs(int fd, int speed, int parity)
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0)
  {
    printf("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK; // disable break processing
  tty.c_lflag = 0;        // no signaling chars, no echo,
                          // no canonical processing
  tty.c_oflag = 0;        // no remapping, no delays
  tty.c_cc[VMIN] = 1;     // read blocks
  tty.c_cc[VTIME] = 0;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
                                     // enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

  /* output modes - clear giving: no post processing such as NL to CR+NL */
  tty.c_oflag &= ~(OPOST);

  cfmakeraw(&tty);

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    printf("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

int serial_setup(const char *portname)
{
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    printf("error %d opening %s: %s\n", errno, portname, strerror(errno));
    return -1;
  }

  set_interface_attribs(fd, B115200, 0); // set speed to 115,200 bps, 8n1 (no parity)

  usleep(10000);
  tcflush(fd, TCIOFLUSH);
  usleep(10000);

  ser_fd = fd;

  return 0;
}

void serial_close()
{
  close(ser_fd);
}

unsigned short crc16(const unsigned char *data_p, unsigned char length)
{
  unsigned char x;
  unsigned short crc = 0xFFFF;

  while (length--)
  {
    x = crc >> 8 ^ *data_p++;
    x ^= x >> 4;
    crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x << 5)) ^ ((unsigned short)x);
  }
  return crc;
}

/*----------------------------------------------------------------------------*/
const char *msgid_to_string(uint8_t msg_id)
{
  switch (msg_id)
  {

  // host -> device                                  1234567890
  case SERIAL_MSG_CMD_PING:
    return "PING";
  case SERIAL_MSG_CMD_INFO:
    return "INFO";
  case SERIAL_MSG_CMD_RESET:
    return "RESET";
  case SERIAL_MSG_CMD_MAC_CFG_READ:
    return "MAC_CFG_RD";
  case SERIAL_MSG_CMD_MAC_CFG_WRITE:
    return "MAC_CFG_WR";
  case SERIAL_MSG_CMD_MAC_SET_STATE:
    return "MAC_STATE";
  case SERIAL_MSG_CMD_MAC_GET_STATE:
    return "MAC_STATE";
  case SERIAL_MSG_CMD_ATOMIC_SEND:
    return "ATMIC_SEND";
  case SERIAL_MSG_CMD_ATOMIC_POLL:
    return "ATMIC_POLL";
  case SERIAL_MSG_CMD_APPMESH_INFO:
    return "MESH_INFO";
  case SERIAL_MSG_CMD_APPMESH_CFG_READ:
    return "MSH_CFG_RD";
  case SERIAL_MSG_CMD_APPMESH_CFG_WRITE:
    return "MSH_CFG_WR";
  case SERIAL_MSG_CMD_APPMESH_PUBLISH:
    return "MESH_PUBL";
  case SERIAL_MSG_CMD_GALLOP_CONFIG_INIT:
    return "GLP_CFG";
  case SERIAL_MSG_CMD_GALLOP_ADD_NBR_TBL_ROW:
    return "GLP_ADDNBR";
  case SERIAL_MSG_CMD_GALLOP_DROP_SCHED:
    return "GLP_DRPSCH";
  case SERIAL_MSG_CMD_GALLOP_QUEUE_DATA:
    return "GLP_QUEDTA";
  case SERIAL_MSG_CMD_GALLOP_POLL_DATA:
    return "GLP_POLL";
  case SERIAL_MSG_CMD_GALLOP_INFO:
    return "GLP_INFO";

  // device -> host
  case SERIAL_MSG_RSP_PING:
    return "PING_REPLY";
  case SERIAL_MSG_RSP_INFO:
    return "INFO_REPLY";
  case SERIAL_MSG_RSP_OK:
    return "OK";
  case SERIAL_MSG_RSP_ERROR:
    return "ERROR";
  case SERIAL_MSG_RSP_DATA_QUEUED:
    return "DTA-QUEUED";
  case SERIAL_MSG_RSP_UNKNOWN_CMD:
    return "UNKNWN_CMD";
  case SERIAL_MSG_RSP_MAC_CFG_READ:
    return "MAC_CFG_RD";
  case SERIAL_MSG_RSP_ATMIC_DATA_RX:
    return "ATM_DTA_RX";
  case SERIAL_MSG_RSP_ATOMIC_DONE:
    return "ATMIC_DONE";
  case SERIAL_MSG_RSP_MAC_STATE:
    return "MAC_STATE";
  case SERIAL_MSG_RSP_APPMESH_INFO:
    return "MSH_INFO";
  case SERIAL_MSG_RSP_APPMESH_CFG_READ:
    return "MSH_CFG_RD";
  case SERIAL_MSG_RSP_APPMESH_RXMSG:
    return "MSH_RX_MSG";
  case SERIAL_MSG_REPLY_GALLOP_DATA_RX:
    return "GLP_RX_DTA";
  case SERIAL_MSG_REPLY_GALLOP_DONE:
    return "GLP_DONE";
  case SERIAL_MSG_REPLY_GALLOP_INFO:
    return "GLP_INFO";
  case SERIAL_MSG_REPLY_GALLOP_STATE:
    return "GLP_STATE";
  }
  return "??????????";
}

/*----------------------------------------------------------------------------*/
void print_packet(uint8_t format, char *p_format, uint8_t *pkt_data, uint16_t pkt_len)
{
  uint8_t msg_id;

  switch (format)
  {
  case 0:
    msg_id = pkt_data[0];
    printf(p_format, msgid_to_string(msg_id));
    if (pkt_len >= 4)
    {
      pkt_len -= 2;
      printf("%02x ", *pkt_data++);
      printf("%3u  ", *pkt_data++);
      pkt_len -= 2;
      for (int i = 0; i < pkt_len; i++)
      {
        printf("%02x", *pkt_data++);
      }
#ifdef VERBOSE_PRINT_CRC
      printf("  [crc:");
      for (int i = 0; i < 2; i++)
      {
        printf("%02x", *pkt_data++);
      }
      printf("]");
#else
      pkt_data += 2;
#endif
    }
    else
    {
      for (int i = 0; i < pkt_len; i++)
      {
        printf("%02x ", *pkt_data++);
      }
    }
    printf("\n");
    break; /* case 0 */
  }
}

/*----------------------------------------------------------------------------*/
void send_slip_packet(uint8_t msg_type, uint8_t len, uint8_t *payload)
{
  if (len > SERIAL_MSG_LEN - SERIAL_MSG_OVERHEAD)
  {
    return;
  }

  memset(serial_tx_buf, 0, SERIAL_MSG_LEN);
  memset(serial_slip_tx_buf, 0, SERIAL_SLIP_BUF_LEN);

  serial_msg_t *data_packet = (serial_msg_t *)serial_tx_buf;

  data_packet->msg_type = msg_type;
  data_packet->len = len;

  if (len)
  {
    memcpy(data_packet->payload, payload, len);
  }

  uint16_t offset = sizeof(serial_msg_t) + len;

  uint16_t crc = crc16((const unsigned char *)serial_tx_buf, offset);
  serial_tx_buf[offset] = (uint8_t)((crc >> 8) & 0xFF);
  serial_tx_buf[offset + 1] = (uint8_t)(crc & 0xFF);

  /* print message before it is slipped */
  // print_packet(PKT_FORMAT_MSGIDpLEN, ">TX : %-10s> ", serial_tx_buf, (offset + SERIAL_MSG_CRC_LEN));

  uint32_t slip_tx_len;
  ret_code_t ret = slip_encode(serial_slip_tx_buf, serial_tx_buf, SERIAL_MSG_OVERHEAD + len, &slip_tx_len);

  if (ret == NRF_SUCCESS)
  {
    // reset the SLIP state machine first
    uint8_t esc = SLIP_BYTE_END;
    write(ser_fd, &esc, 1);

    write(ser_fd, serial_slip_tx_buf, slip_tx_len);
  }
}

/*----------------------------------------------------------------------------*/
int16_t /* will be -ve if not found, otherwise index of where found */
find_pattern_match(uint8_t *pkt_buf, uint16_t pkt_len, pattern_match_t *p_match)
{
  uint16_t start_idx = 0;
  uint16_t end_idx = 0xFFFF;

  if ((p_match->len == 0) || (pkt_len == 0))
  {
    return -1;
  }
  for (uint16_t i = start_idx; i <= end_idx; i++)
  {
    if ((i + p_match->len) > pkt_len)
    {
      return -1;
    }
    if (memcmp(&pkt_buf[i], p_match->buf, p_match->len) == 0)
    {
      /* found a match at index i */
      return i;
    }
  }
  return -1;
}

/*----------------------------------------------------------------------------*/
void on_slip_packet(uint8_t *pkt_buf, uint16_t pkt_len)
{
  if (rsp_flags == 0)
  {
    return;
  }

  if (rsp_flags & RSP_FLAG_OK)
  {
    if (pkt_buf[0] == SERIAL_MSG_RSP_OK)
    {
      exit(EXIT_SUCCESS);
    }
  }

  if (rsp_flags & RSP_FLAG_ERROR)
  {
    if (pkt_buf[0] == SERIAL_MSG_RSP_ERROR)
    {
      exit(EXIT_SUCCESS);
    }
  }

  if (rsp_flags & RSP_FLAG_PATTERN)
  {
    if (find_pattern_match(pkt_buf, pkt_len, &pattern_match))
    {
      exit(EXIT_SUCCESS);
    }
  }
}

/*----------------------------------------------------------------------------*/
void rx_slip_packet(uint8_t *msg_len, uint8_t *msg_payload)
{
  uint8_t rx_byte;
  while (read(ser_fd, &rx_byte, sizeof(uint8_t)))
  {
    ret_code_t ret = slip_decode_add_byte(&slip_rx, rx_byte);
    // printf("ret_code_t: %u\n", ret);
    if (ret == NRF_SUCCESS)
    {

      uint8_t rx_len = slip_rx.current_index;
      if (rx_len > SERIAL_MSG_LEN - SERIAL_MSG_OVERHEAD)
      {
        return;
      }

      /* display the packet */
      // print_packet(PKT_FORMAT_MSGIDpLEN, "<RX : %-10s< ", slip_rx.p_buffer, slip_rx.current_index);

      /* Check if the packet is of the type expected to exit the app */
      on_slip_packet(slip_rx.p_buffer, slip_rx.current_index);

      *msg_len = slip_rx.current_index;
      memcpy(msg_payload, slip_rx.p_buffer, slip_rx.current_index);

      slip_rx.state = SLIP_STATE_DECODING;
      slip_rx.current_index = 0;

      uint16_t crc = (msg_payload[rx_len - 2] << 8) | msg_payload[rx_len - 1];
      uint16_t crc_calc = crc16((const unsigned char *)msg_payload, rx_len - SERIAL_MSG_CRC_LEN);
      if (crc != crc_calc)
      {
        printf("     ::CRC-Fail %04x %04x\n", crc, crc_calc);
        return;
      }

      return;
    }
  }
}
