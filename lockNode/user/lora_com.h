#ifndef _LORA_COM_H
#define _LORA_COM_H

#include "stm32l0xx.h"

#define MAX_TXDATALEN   100
#define MAX_QUEUESIZE		 5

#define MAX_RXDATALEN   100
#define LORA_RECNUM_MAX 256

#define LORA_TX_TIMEOUT 		LORA_SEND_MAX_TIMEOUT
#define LORA_WAIT_ACK_TIME	LORA_SEND_MAX_TIMEOUT*4
#define MAX_RESEND_TIMES 3

enum LORA_FSM{ 
	LORA_IDLE_FSM, 
	LORA_TXDONE_FSM,
	LORA_TX_FSM,
	LORA_WAIT_ACK_FSM
};

typedef struct{
	uint8_t data[MAX_TXDATALEN];
	uint8_t data_len;
	uint8_t ack_enabled;	
} LORA_DATABUF;

typedef struct {
		LORA_DATABUF node[MAX_QUEUESIZE];
		uint8_t tx_done_flag;
		uint8_t resend_times_count;
		uint32_t tickTime;
}LORATXBUF;

//typedef struct {
//	LORA_DATABUF data[MAX_QUEUESIZE];
//	uint8_t ack;
//}LORARXBUF;

typedef struct {
	uint8_t lora_recv_data_buf[LORA_RECNUM_MAX];
	uint8_t lora_recv_data_buf_len;
	uint8_t lora_data_arrived ;
	uint8_t ack_flag;
}LORA_DATA_STRUCT;

extern LORA_DATA_STRUCT lora_data;
extern LORATXBUF lora_txBuf;
//extern LORARXBUF lora_rxBuf;

void init_lora_txDataBuf(LORATXBUF* buf);

uint8_t is_txBuf_empty(LORATXBUF buf);
uint8_t write_datas_to_txBuf(LORATXBUF* buf, uint8_t*datas, uint8_t datas_len, uint8_t ack);
uint8_t remove_datas_from_txBuf(LORATXBUF* buf);

//uint8_t is_rxBuf_empty(LORARXBUF buf);
//uint8_t write_datas_to_rxBuf(LORARXBUF* buf, uint8_t*datas, uint8_t datas_len);
//uint8_t remove_datas_from_rxBuf(LORARXBUF* buf);

void set_tx_done_flag(void);
uint8_t Lora_Send(uint8_t *p_send_buf, uint16_t len);
uint8_t lora_com_fsm(uint8_t param);

void set_ack_flag(void);

#endif
