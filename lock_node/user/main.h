#ifndef MAIN_H
#define MAIN_H

#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "GlobalVar.h"

#define LED_RUNNING_PERIOD			3000
#define SYS_IDLE_TIME						10000
#define FINGER_UART_RX_TIMEOUT  20
#define LOCK_UART_RX_TIMEOUT		30
#define LOCK_ACK_TIME						500  //发送的时候有110的唤醒，然后是数据发送
#define FINGER_NO_TOUCH_TIME		500

enum SYS_FSM{ 
	SYS_IDLE,  
	SLEEP_MODE
};

typedef struct {
	enum SYS_FSM sys_fsm;
	uint32_t sysIdle_timeTick;
	uint8_t sys_time[6]; //YY-MM-DD-HH-MM-SS
}SYS_STRUCT;

typedef struct{
	uint8_t user_name[4];
	uint8_t user_pw[10];
	uint8_t user_card[8];
	uint16_t user_finger;
	uint8_t opType;	
}USER_MAG;

// -------   exported variables ----------
extern SYS_STRUCT sys_para;
extern USER_MAG user_mag;
extern uint8_t bell_update;
extern uint8_t alarm_valid;

// ------  FUCNTOIN DECLARATION    -------
void init_lora_com(void);
void sendToServer(uint8_t* data, uint16_t data_len, uint8_t cmd, uint8_t ack);
void update_sysIdle_tick(void);
void copy_data_to_lora_txBuf(uint8_t*buf, uint8_t buf_len, uint8_t cmd, uint8_t pkt_no, uint8_t ack);
#endif
