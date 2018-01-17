#ifndef _LOCK_H
#define _LOCK_H

#include "stm32l0xx.h"

#define LOCK_HEADER 0xF5

#define LOCK_CMD_IDLE       0x00  // self define
#define ADD_FINGER 					0x02
#define DEL_FINGER					0x03

#define	LOCK_CMD_READ_REV		0x16
#define LOCK_CMD_READ_TIME	0x17
#define LOCK_CMD_CAL_TIME		0x18
#define	LOCK_CMD_ADD_USER		0x19
#define LOCK_CMD_ADD_PW			0x1A
#define LOCK_CMD_DEL_USER		0x1B
#define LOCK_CMD_DEL_PW			0x1C
#define LOCK_CMD_READ_LOG		0x1D
#define LOCK_CMD_MODE_TRANS	0x1E
#define LOCK_CMD_INIT				0x1F
#define LOCK_CMD_REGISTER		0x20
#define LOCK_CMD_USER_SEARCH					0x21
#define LOCK_CMD_PW_SEARCH						0x22
#define LOCK_CMD_STATUS								0x23
#define LOCK_CMD_TEMP_PW							0x24
#define LOCK_CMD_CHANGE_USER_INFO			0x25
#define LOCK_CMD_CHANGE_PW						0x26
#define LOCK_CMD_ACCESS_LOCAL_ADD_PW	0x27
#define LOCK_CMD_LOCK_CON							0xB1
#define LOCK_CMD_REMOTE_LOCK_CON			0xB2
#define	LOCK_CMD_MATCH								0xB3

#define LOCK_UP_OP_LOG								0xE0
#define LOCK_UP_CONF_NOTIFY						0xE1
#define LOCK_UP_STA_NOTIFY						0xE2
#define LOCK_READ_REV_ID							0xE3
#define LOCK_READ_REMOTER_NUM					0xE4
#define LOCK_DEL_REMOTER_NUM_ID				0xE5

enum RX_STATE{ 
	RX_BUSY, 
	RX_IDLE
};

typedef struct {
	uint8_t cmd_type;
	uint32_t wait_time;
	enum RX_STATE  rx_state;
}LOCK_COM_STRUCT;

void usart2_rx_enbale_int(void);
void send_cmd_to_lock(uint8_t cmd, uint8_t* data, uint8_t data_len);

#endif
