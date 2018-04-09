#ifndef _LOCK_H
#define _LOCK_H

#include "stm32l0xx.h"

#define MAX_SEND_LEN									30
#define LOCK_HEADER 									0xF5


#define LOCK_CMD_IDLE       					0xFE  // self define
#define ADD_FINGER 										0x02
#define DEL_FINGER										0x03

#define LOCK_CMD_ACK									0x00
#define	LOCK_CMD_READ_REV							0x16
#define LOCK_CMD_READ_TIME						0x17
#define LOCK_CMD_CAL_TIME							0x18
#define	LOCK_CMD_ADD_USER							0x19
#define LOCK_CMD_ADD_PW								0x1A
#define LOCK_CMD_DEL_USER							0x1B
#define LOCK_CMD_DEL_PW								0x1C
#define LOCK_CMD_READ_LOG							0x1D
#define LOCK_CMD_MODE_TRANS						0x1E
#define LOCK_CMD_INIT									0x1F
#define LOCK_CMD_REGISTER							0x20
#define LOCK_CMD_USER_SEARCH					0x21
#define LOCK_CMD_PW_SEARCH						0x22
#define LOCK_CMD_STATUS								0x23
#define LOCK_CMD_TEMP_PW							0x24
#define LOCK_CMD_CHANGE_USER_INFO			0x25
#define LOCK_CMD_CHANGE_PW						0x26
#define LOCK_CMD_ACCESS_LOCAL_ADD_PW	0x27
#define LOCK_CMD_VOICE_BROADCAST			0x52
#define LOCK_CMD_LOCK_CON							0xB1
#define LOCK_CMD_REMOTE_LOCK_CON			0xB2
#define	LOCK_CMD_MATCH								0xB3

#define LOCK_UP_OP_LOG								0xE0
#define LOCK_UP_CONF_NOTIFY						0xE1
#define LOCK_UP_STA_NOTIFY						0xE2
#define LOCK_READ_REV_ID							0xE3
#define LOCK_READ_REMOTER_NUM					0xE4
#define LOCK_DEL_REMOTER_NUM_ID				0xE5

#define LOCK_POWER_ON   0x01
#define LOCK_DOOR_BELL	0x12

#define LOCK_MAX_RETRY_TIMES  3

//	---   ��������	----
#define S_KEY1                          0x01  //  1. ������
#define S_KEY2                          0x02  //  2. ��������ˮ������
#define S_KEY3                          0x03  //  3. ��������ˮ������
#define S_POWER_UP                      0x04  //  4. �������ϵ� �����ɹ� ��ʾ��
#define S_WARING                        0x05  //  5. ����
#define S_DOORBELL                      0x06  //  6. ����
#define S_PLS_SET_TIME                  0x14  //  20. ������ʱ��
#define S_PWD_INVALID                   0x15  //  21. ������Ч
#define S_PWD_FPT_INPUT                 0x16  //  22. ��������������ָ��
#define S_SET_SUCCEED                   0x17  //  23. ���óɹ�
#define S_UNLOCKSETING                  0x18  //  24. �ѽ��볣��ģʽ
#define S_LOCKSETING                    0x19  //  25. ��ȡ������ģʽ
#define S_INITIALIZING                  0x1A  //  26. ���ڳ�ʼ��
#define S_INIT_SUCCEED                  0x1B  //  27. ��ʼ���ɹ�
#define S_DEL_SUCCEED                   0x1C  //  28. ɾ���ɹ�
#define S_PWD_ERROR                     0x1D  //  29. �������
#define S_PWD_EXIST                     0x1E  //  30. �����Ѵ���
#define S_KEY_FULL                      0x1F  //  31. ����������
#define S_PWD_INPUT_NEW                 0x20  //  32. ����������	
#define S_PWD_INPUT_NEW_AGAIN           0x21  //  33. ���ٴ�����������
#define S_ADD_PWD_SUCCEED               0x22  //  34. �������óɹ�
#define S_CARD_EXIST                    0x23  //  35. �˿��Ѵ���
#define S_CARD_FULL                     0x24  //  36. ��������
#define S_ADD_CARD_SUCCEED              0x25  //  37. �˿����óɹ�
#define S_FPT_ERROR                     0x26  //  38. ָ�ƴ���
#define S_FPT_INVALID                   0x27  //  39. ָ����Ч
#define S_CARD_INVALID                  0x28  //  40. �˿���Ч
#define S_PLS_COMEIN                    0x29  //  41. �������
#define S_PLS_INPUT_FPT                 0x2A  //  42. ������ָ��
#define S_PLS_INPUT_FPT_ANGIN           0x2B  //  43. ���ٴ�����ָ��
#define S_ADD_FPT_SUCCEED               0x2C  //  44. ָ�����óɹ�
#define S_PLS_INPUT_CARD                0x2D  //  45. �����뿨
#define S_FPT_EXIST                     0x2E  //  46. ָ���Ѵ���
#define S_ADD_FPT_FAILED                0x2F  //  47. ָ������ʧ��
#define S_PLS_INPUT_KEY_SECOND          0x30  //  48. ������ڶ���Կ
#define S_KEY_INVALID                   0x31  //  49. ������Կ��Ч
#define S_LOCK_DEAD_BOLT                0x32  //  50. �ڲ��ѷ���
#define S_NIHAO_COMEIN                  0x33  //  51. ���ã����
#define S_PLS_INPUT_ADKEY               0x34  //  52. �������������
#define S_EXIT_MENU                     0x36  //  54. �˵����˳�
#define S_CONNECT_SUCCESSFULLY          0x37  //  55. ���ӳɹ�����ǰ���������ǣ�
#define S_CONNECT_FAILED                0x38  //  56. ����ʧ��
#define S_RELEASE_FINGER                0x39  //  57. ��ſ���ָ
#define S_NOTE_DOOR_NOT_CLOSED          0x3A  //  58. ��ע����û�غ�
#define S_UPGRADE_SUCCESSFULLY          0x3B  //  59. �����ɹ���������4λ�꣩
#define S_UPGRADE_FAILED                0x3C  //  60. ����ʧ��
#define S_NO_PERMISSION_TO_OPERATE      0x3D  //  61. ��Ȩ����
#define S_WAITING_NETWORK_AUTHOR        0x3E  //  62. ���ڵȴ�������Ȩ(��������λʱ��
#define S_CONNECTING_NETWORK            0x3F  //  63. ��������(��������λ�֣�
#define S_UPGRADING                     0x40  //  64. �����������꣩
#define S_SET_FAILED                    0x46  //  70. ����ʧ��
#define S_CONFIRM_ENTER                 0x47  //  71. ȷ���밴#�ż�
#define S_OPEN                          0x5F  //  95. �ѿ���
#define S_POWER_LOW                     0x60  //  96. ���û����
#define S_FPT_FULL                      0x61  //  97. ָ��������
#define S_DEL_FAILED                    0x62  //  98. ɾ��ʧ��


typedef struct {
	uint8_t cmd_type;
	uint8_t retry_times;	
	uint8_t last_data[MAX_SEND_LEN];
	uint8_t last_data_len;
	uint32_t timeoutTick;
}LOCK_COM_STRUCT;


//batteryLevel 
// 1 : 75% - 100%
// 2 : 50% - 75%
// 3 : 25% - 50%
// 4 : 1%  - 25%
typedef struct {
	uint32_t nodeID;
	uint8_t batteryLevel;//
}LOCKUNIT;
/*  -------------------    Exported Variables -------------*/
extern LOCK_COM_STRUCT lock_com;
extern volatile LOCKUNIT lockUnit;

/*  -------------------    Functin Declaration -------------*/
void lock_init(void);
void init_lock_com(void);
void usart2_rx_enbale_int(void);
void send_cmd_to_lock(uint8_t cmd, uint8_t* data, uint8_t data_len );

#endif
