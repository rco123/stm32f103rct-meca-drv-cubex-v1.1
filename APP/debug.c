#include "main.h"
#include "debug.h"

#include <stdio.h>
#include <stdbool.h>


extern UART_HandleTypeDef huart1;


int _write(int32_t file, uint8_t *ptr, int32_t len) {

	if( HAL_UART_Transmit(&huart1, ptr, len, len) == HAL_OK ) return len;
	    else return 0;
}


void yb_debug_msg(char *fmt, char *file, const char *func, int line, ...)
{
    char buffer[128] = {0};
    va_list arg_list;
    va_start(arg_list, line);
    vsnprintf(buffer, sizeof(buffer), fmt, arg_list);
    printf("[Debug]: %s (file: %s, func: %s, line: %d)\n", buffer, file, func, line);
    va_end(arg_list);
}

