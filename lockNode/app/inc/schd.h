#ifndef __SCHD_H__
#define __SCHD_H__

union SchdParameter {
	void* ptr;
	unsigned char data;
	int intvalue;
	float floatvalue;
	char chars[32];
};

typedef void (*SchdCallback)(union SchdParameter);

/*
 @brief 在指定毫秒后调用回调函数，同时携带注册时的入参
 @param microsecond 延时毫秒数
 @param p 回调时的参数
 @return 1=add one item, 0=no item added
 */
int Schd_After(int microsecond, SchdCallback callback, union SchdParameter p);

/*
 @brief 在指定毫秒后调用回调函数，同时携带注册时的入参
 @param microsecond 延时毫秒数
 @param value 回调时的参数，整数值
 @return 1=add one item, 0=no item added
 */
int Schd_After_Int(int microsecond, SchdCallback callback, unsigned char value);

/*
 @brief 运行调度器
 @param current_microsecond 当前毫秒数
 */
void Schd_Run(int current_microsecond);

#endif
