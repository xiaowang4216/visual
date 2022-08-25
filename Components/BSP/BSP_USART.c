/**
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  * @file       BSP_USART.c/h
  * @brief      将HAL库串口函数进行二次封装，并在串口中断中接收数据
  * @note      	
  * @history
  *  Version    Date            Author          Modification
  *  V3.0.0     2020.7.14     	              	1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  */
#include "BSP_USART.h"


struct __FILE 
{ 
	int handle; 
}; 
  FILE __stdout;     
int fputc(int ch, FILE *f)
{ 	
	while((USART2->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART2->TDR=(u8)ch;      
	return ch;
}




/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

