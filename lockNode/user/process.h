#ifndef _PROCESS_H
#define _PROCESS_H

#include "stm32l0xx.h"
#include "finger.h"

#define TO_SERVER_HEADER_1		0x5A
#define TO_SERVER_HEADER_2		0x5A
#define RX_FRAME_HEADER_1			0xA5
#define RX_FRAME_HEADER_2 		0xA5

#define NO_LORA_CMD					  0x00
#define UPDATE_TIME_CMD				0x02		
#define UPDATE_CONTENT_CMD		0x13
#define SET_SLEEP_CMD					0x23
#define SET_MANUFACTURE_CMD		0x04
#define ACK_CMD								0xEE
#define FAILED_ACK						0x55

#define UPLOG_TYPE_CARD				0x02
#define UPLOG_TYPE_PW					0x01
#define UPLOG_TYPE_FINGER			0x03

#define UPDATE_TIME_REQ				0x01
#define UPDATE_TIME_SUCCESS		0x02
#define UPDATE_TIME_FAILED		0x03

#define MAX_RECORD_NUM		10
typedef struct{
	uint8_t updateFlag;		//0 表示无开锁记录，03表示有开锁记录，正好利用03表示开锁类型
	uint16_t fingerAddress;
	uint8_t openDoorTime[5];//BCD code
	uint8_t batterInfo[5];	
}Record;

extern Record record[MAX_RECORD_NUM];

uint8_t isEmptyRecord(void);
// 指纹地址2字节+开门时间5字节+电压信息5字节
void addRecord(uint8_t* data);
void delFirstRecord(void);
void delAllRecord(void);
	
void finger_OpSta(void);
void serverUpdateNode(uint8_t*data, uint8_t data_len);
void process_fingerRxBuffer(void);
void process_serverRxBuffer(void);
void process_lockUnitRxBuffer(void);
void finger_callBack(enum FINGER_FSM fsm, uint8_t* data);
uint8_t lock_cmd_timeout_callback(void);

#endif
