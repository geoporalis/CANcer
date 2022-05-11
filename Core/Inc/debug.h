#ifndef DEBUG_H_
#define DEBUG_H_
/**
 * @file    debug.h
 *
 * @date    05.11.2020
 * @author  Georg Poller
 *
 * @brief Function declarations for Debug output e.g. via UART/Console directly in the IDE
 *
 * By default the UART is adjusted to 115.2kBaud
 */

#include "global.h"


#ifdef USE_DEBUG_PRINT
# include <stdio.h>
 const char * pathToFileName(const char * path);
# define DEBUG_PRINT(...) printf(__VA_ARGS__);
# define DEBUG_PRINT_FL(...) {printf(__VA_ARGS__);fflush(stdout);}
# define DEBUG_PRINT_LN(...) {printf(__VA_ARGS__);printf("\r\n");}
# define DEBUG_PRINT_FILE(...) {printf("[%s:%u] %s():",pathToFileName(__FILE__), __LINE__, __FUNCTION__);printf(__VA_ARGS__);printf("\r\n");}
#else
# define DEBUG_PRINT(...)
# define DEBUG_PRINT_LN(...)
# define DEBUG_PRINT_FILE(...)
#endif





// soory DOT0 is used as FAN_PWM1
//#define SET_DOT_0_1 	(DIT_00_GPIO_Port->ODR |=  DIT_00_Pin)
//#define SET_DOT_0_0		(DIT_00_GPIO_Port->ODR &= ~DIT_00_Pin)
//#define TOG_DOT_0		(DIT_00_GPIO_Port->ODR ^=  DIT_00_Pin)
//#define SET_DOT_0(x)	(x==0 ? SET_DOT_0_0 : SET_DOT_0_1)

#define SET_DOT_1_1 	(DIT_01_GPIO_Port->ODR |=  DIT_01_Pin)
#define SET_DOT_1_0		(DIT_01_GPIO_Port->ODR &= ~DIT_01_Pin)
#define TOG_DOT_1		(DIT_01_GPIO_Port->ODR ^=  DIT_01_Pin)
#define SET_DOT_1(x)	(x==0 ? SET_DOT_1_0 : SET_DOT_1_1)

#define SET_DOT_2_1 	(DIT_02_GPIO_Port->ODR |=  DIT_02_Pin)
#define SET_DOT_2_0		(DIT_02_GPIO_Port->ODR &= ~DIT_02_Pin)
#define TOG_DOT_2		(DIT_02_GPIO_Port->ODR ^=  DIT_02_Pin)
#define SET_DOT_2(x)	(x==0 ? SET_DOT_2_0 : SET_DOT_2_1)

#define SET_DOT_3_1 	(DIT_03_GPIO_Port->ODR |=  DIT_03_Pin)
#define SET_DOT_3_0		(DIT_03_GPIO_Port->ODR &= ~DIT_03_Pin)
#define TOG_DOT_3		(DIT_03_GPIO_Port->ODR ^=  DIT_03_Pin)
#define SET_DOT_3(x)	(x==0 ? SET_DOT_3_0 : SET_DOT_3_1)

// to set as variable takes 116ns longer then to set as fixed value
// since the value of the variable is not know at the time of compiling
#define SET_DOTS(bits) \
		(bits & 0x01 ? SET_DOT_0_1 : SET_DOT_0_0), \
		(bits & 0x02 ? SET_DOT_1_1 : SET_DOT_1_0), \
		(bits & 0x04 ? SET_DOT_2_1 : SET_DOT_2_0), \
		(bits & 0x08 ? SET_DOT_3_1 : SET_DOT_3_0)



/// to printf a byte in binary format
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')



//usage: 	printf("my binary value" BYTE_TO_BINARY_PATTERN "\r\n", BYTE_TO_BINARY(byte_in_binary));
//			printf("my multi-byte binary value" BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN "\r\n", BYTE_TO_BINARY(2nd_byte_to_binary), BYTE_TO_BINARY(1st_byte_to_binary));

#endif /* DEBUG_H_ */
