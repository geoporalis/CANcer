#ifndef GLOBAL_H_
#define GLOBAL_H_

#ifdef __cplusplus
#define __EXTERN_C extern "C"
#else
#define __EXTERN_C
#endif

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <limits.h>

//#include "typedefines.h"
//#include <config_general.h>
//#include "defines.h"

#ifdef DEBUG
#define USE_DEBUG_PRINT
#include "debug.h"
#endif





#define US_2_TICKS(x)	(x*170)
#define MS_2_TICKS(x)	(x*170*1000)


/// checks if the previous error code was a success and only executes the given statement if so
#define RET_CHECK(x) if(ret == StdError::SUCCESS_VALUE){ ret = x;}

#define ArraySize(arr) (sizeof arr / sizeof *arr)

#define NR_TEMP_SENSOR 13

#endif /* GLOBAL_H_ */
