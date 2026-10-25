#include "tensorflow/lite/micro/debug_log.h"
#include <stdarg.h>
#include <stdio.h>

extern "C" {
    #include "mik32_hal_usart.h"
}

extern USART_HandleTypeDef husart0;

extern "C" void DebugLog(const char* format, va_list args) {
    char buffer[128];
    int len = vsnprintf(buffer, sizeof(buffer), format, args);

    if (len > 0) {
        if (len > static_cast<int>(sizeof(buffer)) - 1) {
            len = sizeof(buffer) - 1;
        }
        
        for (int i = 0; i < len; ++i) {
            HAL_USART_Transmit(&husart0, buffer[i], USART_TIMEOUT_DEFAULT);
        }
    }
}

extern "C" int DebugVsnprintf(char* buffer, size_t buf_size,
                              const char* format, va_list vlist) {
    return vsnprintf(buffer, buf_size, format, vlist);
}
