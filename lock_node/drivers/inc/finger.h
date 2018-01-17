#ifndef _FINGER_H
#define _FINGER_H

#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "GlobalVar.h"

#define CHARACTER_BUF_SIZE 		150
#define MAX_FINGER_BUFFERSIZE 780

#define FINGER_BUF_1_ID 0x01
#define FINGER_BUF_2_ID 0x02

#define FINGER_LIB_START_POSITION 0x0000
#define SEARCH_PAGES_NUM 					150

#define PACKET_HEADER 0xEF01
#define FINGER_ADDR		0xFFFFFFFF

typedef struct {
	uint8_t up_download_fingerModelBuff[MAX_FINGER_BUFFERSIZE];
	uint16_t buf_size;
	uint8_t receive_data_flag;
}UPDOWNLOADBUF;

enum FINGER_FSM{
	FSM_IDLE,
	REGISTER_LOCAL, //采集两次，并将模板存在本地
	REGISTER_UPPER,//采集两次，将模板上传到上位机
	MATCH_LOCAL,	 //采集一次，本地库中比对
	REGISTER_DOWN,  //将指纹模块下传到指模库
	DELETE_CHAR
};

enum Finger_Register{
	STA_IDLE,
	GETIMG1_STA,
	GENCHAR1_STA,
	GETIMG2_STA,
	GENCHAR2_STA,
	REGMODEL_STA,
	STORECHAR_STA,
	UPCHAR_STA,
	DOWNCHAR_STA,
	DELCHAR_STA,
	SEARCH_STA,
	EMPTY_STA,
	MATCH_STA,
	LOADCHAR_STA,
	TEMPLETENUM_STA
};

enum FINGER_CMD{
	NO_CMD = 0,
	GETIMG = 0x01, 
	GENCHAR = 0x02, 
	MATCH_CMD = 0x03,
	SEARCH_CMD = 0x04,
	REGMODEL = 0x05,
	STORE = 0x06,
	LOADCHAR = 0x07,
	UPCHAR = 0x08, 
	DOWNCHAR	= 0x09,
	DELETCHAR = 0x0C,
	EMPTY = 0x0D,
	SETSYSPARA = 0x0E,
	READSYSPARA = 0x0F,
	WRITENOTEPAD = 0x18,
	READNOTEPAD = 0x19,
	READTEMPLETENUM = 0x1D,
	READFPFLASH = 0x1F,
	READINFO = 0x47
};

enum FINGER_PID{
	PID_COMMAND = 0x01, 
	PID_DATA 		= 0x02, 
	PID_ACK 		= 0x07, 
	PID_END 		= 0x08
};

typedef struct {
	enum FINGER_FSM fsm;
	enum Finger_Register sta;
}FINGER_FSM_STRUCT;

typedef struct {
	uint8_t character_buf[CHARACTER_BUF_SIZE];
	uint8_t character_size;
	enum FINGER_CMD command;
	enum FINGER_PID pid;
	uint8_t pkt_num ;
}FINGER_PARA;

typedef struct {
	uint16_t pageID;
	//todo
}FINGER_LIB;

typedef struct {
	uint8_t cmd_retry_times;
	uint32_t response_ack_tickTime;
}FINGER_COMMUNICATION;


extern FINGER_PARA finger;
extern FINGER_FSM_STRUCT finger_fsm;
extern FINGER_COMMUNICATION finger_com;
extern FINGER_LIB finger_lib;

uint8_t finger_cmd_wait_timeOut(enum Finger_Register sta);
void finger_cmd_wait_time_update(void);
void init_finger_touch_int(void);
//BOOL finger_online(void);
uint8_t send_to_finger(enum FINGER_PID pid, uint8_t* data, uint16_t data_len);
uint16_t fingerSearchFrame(uint8_t* data, uint16_t len);
void sendCmd_getImg(void);
void sendCmd_genChar(uint8_t bufID);
void sendCmd_regModel(void);
void sendCmd_store(uint8_t bufID, uint16_t pageID);
void sendCmd_upChar(uint8_t bufID);
void sendCmd_templeteNum(void);
void sendCmd_downChar(uint8_t bufID);
void sendCmd_deletChar(uint16_t pageID, uint16_t deletNum);
void sendCmd_empty(void);
void sendCmd_match(void);
void sendCmd_search(uint8_t bufID, uint16_t startPage, uint16_t pageNum);
void sendCmd_readInfo(uint8_t notePageNum);
void sendCmd_readSysPara(void);
void sendCmd_loadChar(uint8_t bufID, uint16_t pageID);
void sendData_downchar(uint8_t* buf, uint16_t buf_len);

void finger_processCMD(void);
void finger_processACK(void);
void finger_processEND(void);
void finger_processDATA(void);

#endif
