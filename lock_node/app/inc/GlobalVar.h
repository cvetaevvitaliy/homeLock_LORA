#ifndef __GLOBAL_VAR_H__
#define __GLOBAL_VAR_H__
#include "stm32l0xx.h"

#define clrbit(reg,bit)   reg &= ~(bit)	/*清寄存器的某1比特位*/
#define bitclr(reg,bit)   reg &= ~(bit)	/*清寄存器的某1比特位*/

#define setbit(reg,bit)   reg |=  (bit)	/*设置寄存器的某1比特位*/
#define bitset(reg,bit)   reg |=  (bit)	/*设置寄存器的某1比特位*/
#define cplbit(reg,bit)   reg ^=  (bit)	/*对寄存器的某1比特位取反*/
#define bitcpl(reg,bit)   reg ^=  (bit)	/*对寄存器的某1比特位取反*/

#define testbit(reg,bit) (reg&(bit))
#define bittest(reg,bit) (reg&(bit))

#define regcpl(reg)		 reg = ~reg
#define regclr(reg)		 reg = 0


#define ARRAY_SIZE(array, type) (sizeof(array)/sizeof(type))
#define STR_EQ(a,b) (strcmp((a),(b))==0)

typedef enum { False=0, True=1 }BOOL;
/*
  @brief 获取当前系统启动到当前的毫秒
*/
uint32_t local_ticktime(void);

/*
  @brief 计算给定的时间是否超时
*/
uint8_t timeout(uint32_t last_time, uint32_t ms);

/*
 @brief 延时
*/
void delay_us(uint32_t nus);

/*
 @brief 喂狗
*/
void watchdog(void);


#endif	/* __GLOBAL_VAR_H__ */

