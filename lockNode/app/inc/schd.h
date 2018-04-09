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
 @brief ��ָ���������ûص�������ͬʱЯ��ע��ʱ�����
 @param microsecond ��ʱ������
 @param p �ص�ʱ�Ĳ���
 @return 1=add one item, 0=no item added
 */
int Schd_After(int microsecond, SchdCallback callback, union SchdParameter p);

/*
 @brief ��ָ���������ûص�������ͬʱЯ��ע��ʱ�����
 @param microsecond ��ʱ������
 @param value �ص�ʱ�Ĳ���������ֵ
 @return 1=add one item, 0=no item added
 */
int Schd_After_Int(int microsecond, SchdCallback callback, unsigned char value);

/*
 @brief ���е�����
 @param current_microsecond ��ǰ������
 */
void Schd_Run(int current_microsecond);

#endif