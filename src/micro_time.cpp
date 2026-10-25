#include "tensorflow/lite/micro/micro_time.h"
#include <stdint.h>

extern "C" {
    #include "mik32_hal_scr1_timer.h"
}

namespace tflite {
    uint32_t ticks_per_second() {
        return 1000000;
    }

    uint32_t GetCurrentTimeTicks() {
        return static_cast<uint32_t>(HAL_Time_SCR1TIM_Micros());
    }
}