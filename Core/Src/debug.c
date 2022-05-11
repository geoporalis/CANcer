/**
 * @file    debug.c
 *
 * @date    05.11.2020
 * @author  Georg Poller
 *
 * @brief Function declarations for Debug output e.g. via UART/Console directly in the IDE
 */

#include "main.h"
#include "global.h"
#include "debug.h"
//#ifdef PROTOTYPE
#include "usart.h"

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the LPUART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&hlpuart1,(uint8_t *)&ch , 1, 0xFFFF);
//  LL_USART_TransmitData8(USART1, ch);	//
//		usart_write((Uint8 *)&ch, 1);
//	ITM_SendChar(ch);	// not tested

  return ch;
}

//#endif
#ifdef DEBUG
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
const char * pathToFileName(const char * path)
{
    size_t i = 0;
    size_t pos = 0;
    char * p = (char *)path;
    while(*p){
        i++;
        if(*p == '/' || *p == '\\'){
            pos = i;
        }
        p++;
    }
    return path+pos;
}
#endif
