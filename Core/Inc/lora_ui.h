#ifndef __LORA_UI_H__
#define __LORA_UI_H__	 
#include "sys.h"

/************************************************
 ALIENTEK ������STM32F7������
 ATK-LORA-01ģ��UI�͵ײ㴦������
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com  
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

void lora_at_response(u8 mode);	
u8* lora_check_cmd(u8 *str);
u8 lora_send_cmd(u8 *cmd,u8 *ack,u16 waittime);

void Menu_ui(void);
void Menu_cfg(u8 num);
void Process_ui(void);
void Dire_Set(void);

#endif


