#include "tensorflow/lite/micro/system_setup.h"

extern "C" {
    #include "mik32_hal_scr1_timer.h"
}

namespace tflite {
    void InitializeTarget() {
        HAL_Time_SCR1TIM_Init();
    }
}
