#include "lora_com.h"
#include "string.h"
#include "main.h"
#include "lock.h"
#include "sx1276.h"
#include "GlobalVar.h"
#include "process.h"
enum LORA_FSM lora_fsm;
LORATXBUF lora_txBuf;
LORA_DATA_STRUCT lora_data;
//LORARXBUF lora_rxBuf;
// 初始化 lora通信参数
void init_lora_txDataBuf(LORATXBUF* buf){
	
	for(uint8_t i = 0 ; i < MAX_QUEUESIZE; i++){
		memset(buf->node[i].data, 0, MAX_TXDATALEN);
		buf->node[i].data_len  = 0;
		buf->node[i].ack_enabled = 0;
	}
	buf->tx_done_flag = 0;
	buf->resend_times_count = 0;
	buf->tickTime  = 0;
}

uint8_t is_txBuf_empty(LORATXBUF buf){

	for(uint8_t i = 0 ; i < MAX_QUEUESIZE; i++){
		if(buf.node[i].data_len != 0)
			return 0;
	}

	return 1;
}

uint8_t write_datas_to_txBuf(LORATXBUF* buf, uint8_t*datas, uint8_t datas_len, uint8_t ack){	
	
	if(datas_len > MAX_TXDATALEN)
		return 0 ;
		
	for ( uint8_t i = 0; i < MAX_QUEUESIZE; i++){
		if( buf->node[i].data_len == 0){
			memcpy( buf->node[i].data, datas, datas_len);
			buf->node[i].data_len = datas_len;
			buf->node[i].ack_enabled = ack;
			return i;
		}
	}
	return 0;
}

uint8_t remove_datas_from_txBuf(LORATXBUF* buf){
	
	memmove( buf->node[0].data, buf->node[1].data, (MAX_QUEUESIZE - 1) * (MAX_TXDATALEN + 2));	
	return 0;
}

//uint8_t write_datas_to_rxBuf(LORARXBUF* buf, uint8_t*datas, uint8_t datas_len){
//	if(datas_len > MAX_RXDATALEN)
//		return 0 ;
//		
//	for ( uint8_t i = 0; i < MAX_QUEUESIZE; i++){
//		if( buf->data[i].data_len == 0){
//			memcpy( buf->data[i].data, datas, datas_len);
//			buf->data[i].data_len = datas_len;
//			return 1;
//		}
//	}
//	return 0;
//	
//}

//uint8_t is_rxBuf_empty(LORARXBUF buf){
//	for(uint8_t i = 0 ; i < MAX_QUEUESIZE; i++){
//		if(buf.data[i].data_len != 0)
//			return 1;
//	}

//	return 0;
//}

//uint8_t remove_datas_from_rxBuf(LORARXBUF* buf){
//	
//	memmove( buf->data[0].data, buf->data[1].data, (MAX_QUEUESIZE - 1) * (MAX_TXDATALEN+1));

//	return 0;
//}

//void set_flag(uint8_t* flag){
//	*flag = 1;
//}

//void clear_flag(uint8_t* flag){
//	*flag = 0;
//}

//uint8_t get_flag(uint8_t flag){
//	return flag;
//}
void set_tx_done_flag(void){
	lora_txBuf.tx_done_flag = 1;
}

void clear_tx_done_flag(void){
	lora_txBuf.tx_done_flag  = 0;
}

uint8_t tx_done_flag(void){
	return lora_txBuf.tx_done_flag;
}

uint8_t Lora_Send(uint8_t *p_send_buf, uint16_t len){
	//指纹采集模块的touch_output在LORA发送的时候有变化
	HAL_NVIC_DisableIRQ((IRQn_Type)EXTI0_1_IRQn); 
	FUN_RF_SENDPACKET(p_send_buf, len);	
	clear_tx_done_flag();
	lora_txBuf.resend_times_count++;
	lora_txBuf.tickTime = local_ticktime();
	return 0;
}

void set_ack_flag(void){	
	lora_data.ack_flag = 1;
}

void clear_ack_flag(void){
	lora_data.ack_flag = 0;
}

uint8_t ack_flag(void){
	return lora_data.ack_flag ;
}

uint8_t lora_com_fsm(uint8_t param){
	
	if(tx_done_flag()){
		lora_fsm =  LORA_TXDONE_FSM;
		clear_tx_done_flag();
	}
	// LORA Rx
	if(lora_data.lora_data_arrived == 1){
		process_serverRxBuffer();
		lora_data.lora_data_arrived = 0;
	}	
	switch(lora_fsm){
		case LORA_IDLE_FSM:
			if( !is_txBuf_empty(lora_txBuf)){
				Lora_Send(lora_txBuf.node[0].data, lora_txBuf.node[0].data_len);
				lora_fsm = LORA_TX_FSM;
			}			
			break;
		case LORA_TX_FSM:
			if( timeout( lora_txBuf.tickTime, LORA_TX_TIMEOUT )){
				HAL_NVIC_EnableIRQ((IRQn_Type)EXTI0_1_IRQn);
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
				lora_fsm =  LORA_TXDONE_FSM;
				clear_tx_done_flag();				
			}
			break;
		case LORA_TXDONE_FSM:
			if(lora_txBuf.node[0].ack_enabled == 1){
				lora_fsm = LORA_WAIT_ACK_FSM;
			}else{
				lora_fsm = LORA_IDLE_FSM;
				remove_datas_from_txBuf( &lora_txBuf );
			}			
			clear_ack_flag();	
			lora_txBuf.tickTime = local_ticktime();
			break;		
		case LORA_WAIT_ACK_FSM:
			if( timeout( lora_txBuf.tickTime, LORA_WAIT_ACK_TIME ) ){				
				if( lora_txBuf.resend_times_count >= MAX_RESEND_TIMES ){
					lora_txBuf.resend_times_count = 0;
					remove_datas_from_txBuf( &lora_txBuf );
				}
				lora_fsm = LORA_IDLE_FSM;			
			}			
			if( ack_flag() ){
				clear_ack_flag();
				remove_datas_from_txBuf( &lora_txBuf );
				lora_fsm = LORA_IDLE_FSM;
			}
			break;
		default:
			break;
	}
	
	return 0;
}
// end
