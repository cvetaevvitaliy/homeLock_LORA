#include "temp.h"
#include "process.h"
#include "lock.h"

extern void sendToServer(uint8_t* data, uint16_t data_len, uint8_t cmd, uint8_t ack);

void test_lora_send(void){
	uint8_t data[250] = {1,2,3,4,5,6};
	sendToServer(data, 6, UPDATE_CONTENT_CMD, 1);
}

// no manager default password
// 开启门铃功能
void lockCMDTest(void){
	uint8_t temp_data[2] = {S_SET_FAILED, 0};

	send_cmd_to_lock(LOCK_CMD_VOICE_BROADCAST, temp_data, 1 );	
}

void enable_doorBell(void){
	uint8_t temp_data[20] = {0x00};
	temp_data[3] = 0x01;
	temp_data[7] = 0x01;
	send_cmd_to_lock(LOCK_CMD_REGISTER, temp_data, 8 );	
}


// ---   end  ---
