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

//	---   语音播报	----
#define S_KEY1                          0x01  //  1. 按键音
#define S_KEY2                          0x02  //  2. 按键音（水滴声）
#define S_KEY3                          0x03  //  3. 按键音（水滴声）
#define S_POWER_UP                      0x04  //  4. 触摸）上电 操作成功 提示音
#define S_WARING                        0x05  //  5. 警报
#define S_DOORBELL                      0x06  //  6. 门铃
#define S_PLS_SET_TIME                  0x14  //  20. 请设置时间
#define S_PWD_INVALID                   0x15  //  21. 密码无效
#define S_PWD_FPT_INPUT                 0x16  //  22. 请输入管理密码或指纹
#define S_SET_SUCCEED                   0x17  //  23. 设置成功
#define S_UNLOCKSETING                  0x18  //  24. 已进入常开模式
#define S_LOCKSETING                    0x19  //  25. 已取消常开模式
#define S_INITIALIZING                  0x1A  //  26. 正在初始化
#define S_INIT_SUCCEED                  0x1B  //  27. 初始化成功
#define S_DEL_SUCCEED                   0x1C  //  28. 删除成功
#define S_PWD_ERROR                     0x1D  //  29. 密码错误
#define S_PWD_EXIST                     0x1E  //  30. 密码已存在
#define S_KEY_FULL                      0x1F  //  31. 密码已裴满
#define S_PWD_INPUT_NEW                 0x20  //  32. 请输入密码	
#define S_PWD_INPUT_NEW_AGAIN           0x21  //  33. 请再次输入新密码
#define S_ADD_PWD_SUCCEED               0x22  //  34. 密码配置成功
#define S_CARD_EXIST                    0x23  //  35. 此卡已存在
#define S_CARD_FULL                     0x24  //  36. 卡已配满
#define S_ADD_CARD_SUCCEED              0x25  //  37. 此卡配置成功
#define S_FPT_ERROR                     0x26  //  38. 指纹错误
#define S_FPT_INVALID                   0x27  //  39. 指纹无效
#define S_CARD_INVALID                  0x28  //  40. 此卡无效
#define S_PLS_COMEIN                    0x29  //  41. 您好请进
#define S_PLS_INPUT_FPT                 0x2A  //  42. 请输入指纹
#define S_PLS_INPUT_FPT_ANGIN           0x2B  //  43. 请再次输入指纹
#define S_ADD_FPT_SUCCEED               0x2C  //  44. 指纹配置成功
#define S_PLS_INPUT_CARD                0x2D  //  45. 请输入卡
#define S_FPT_EXIST                     0x2E  //  46. 指纹已存在
#define S_ADD_FPT_FAILED                0x2F  //  47. 指纹配置失败
#define S_PLS_INPUT_KEY_SECOND          0x30  //  48. 请输入第二密钥
#define S_KEY_INVALID                   0x31  //  49. 输入密钥无效
#define S_LOCK_DEAD_BOLT                0x32  //  50. 内部已反锁
#define S_NIHAO_COMEIN                  0x33  //  51. 您好，请进
#define S_PLS_INPUT_ADKEY               0x34  //  52. 请输入管理密码
#define S_EXIT_MENU                     0x36  //  54. 菜单已退出
#define S_CONNECT_SUCCESSFULLY          0x37  //  55. 连接成功（当前分配的序号是）
#define S_CONNECT_FAILED                0x38  //  56. 连接失败
#define S_RELEASE_FINGER                0x39  //  57. 请放开手指
#define S_NOTE_DOOR_NOT_CLOSED          0x3A  //  58. 请注意门没关好
#define S_UPGRADE_SUCCESSFULLY          0x3B  //  59. 升级成功（请输入4位年）
#define S_UPGRADE_FAILED                0x3C  //  60. 升级失败
#define S_NO_PERMISSION_TO_OPERATE      0x3D  //  61. 无权操作
#define S_WAITING_NETWORK_AUTHOR        0x3E  //  62. 正在等待网络授权(请输入两位时）
#define S_CONNECTING_NETWORK            0x3F  //  63. 正在联网(请输入两位分）
#define S_UPGRADING                     0x40  //  64. 正在升级（年）
#define S_SET_FAILED                    0x46  //  70. 设置失败
#define S_CONFIRM_ENTER                 0x47  //  71. 确认请按#号键
#define S_OPEN                          0x5F  //  95. 已开锁
#define S_POWER_LOW                     0x60  //  96. 电池没电了
#define S_FPT_FULL                      0x61  //  97. 指纹已配满
#define S_DEL_FAILED                    0x62  //  98. 删除失败


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
