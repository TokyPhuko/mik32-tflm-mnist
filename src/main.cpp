#include "model_data_f8.h"
#include "ST7735.h"
#include "crest.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include <cstring>

extern "C" {
    #include "mik32_hal_pcc.h"
    #include "mik32_hal_gpio.h"
    #include "mik32_hal_usart.h"
    #include "mik32_hal_scr1_timer.h"
}

enum Button { BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_A, BTN_B, BTN_COUNT };

struct BtnPin {
    GPIO_TypeDef* port;
    HAL_PinsTypeDef pin;
};

namespace {
    const BtnPin kButtons[BTN_COUNT] = {
        {GPIO_1, GPIO_PIN_6},
        {GPIO_2, GPIO_PIN_6},
        {GPIO_2, GPIO_PIN_7},
        {GPIO_1, GPIO_PIN_7},
        {GPIO_1, GPIO_PIN_10},
        {GPIO_1, GPIO_PIN_11}, 
    };

    bool btn_pressed_high[BTN_COUNT] = {false, false, false, false, false, false};

    constexpr int kCanvasSize = 96;           // холст 96x96 бит
    constexpr int kCanvasX = 4;               // левый край холста на экране
    constexpr int kCanvasY = 16;              // верхний край
    constexpr int kCell = 6;
    constexpr int kBrushR = 4;                // радиус кисти в пикселях холста
    constexpr int kCursorR = 4;               // радиус кольца курсора
    constexpr int kMoveStep = 2;              // шаг курсора за тик
    constexpr int kMoveRepeatMs = 45;         // автоповтор при удержании крестовины
    constexpr int kPollMs = 10;               // период опроса кнопок
    constexpr uint32_t kGuessHoldMs = 400;    // удержание A

    constexpr int kCardX = 108;
    constexpr int kCardY = 12;
    constexpr int kCardW = 52;
    constexpr int kCardH = 104;

    uint8_t canvas_bits[kCanvasSize * kCanvasSize / 8];

    constexpr int kTensorArenaSize = 6 * 1024;
    uint8_t tensor_arena[kTensorArenaSize];

    constexpr int kModelInputSize = 16 * 16;
    constexpr int kModelOutputSize = 10;

    constexpr HAL_PinsTypeDef kLedPin = GPIO_PIN_12;
}

namespace {
    bool BtnDown(Button b) {
        bool high = (HAL_GPIO_ReadPin(kButtons[b].port, kButtons[b].pin) == GPIO_PIN_HIGH);
        return btn_pressed_high[b] ? high : !high;
    }

    inline void Canvas_SetBit(int x, int y) {
        if (x < 0 || x >= kCanvasSize || y < 0 || y >= kCanvasSize) return;
        canvas_bits[(y * kCanvasSize + x) / 8] |= static_cast<uint8_t>(1 << (x % 8));
    }

    inline bool Canvas_GetBit(int x, int y) {
        if (x < 0 || x >= kCanvasSize || y < 0 || y >= kCanvasSize) return false;
        return (canvas_bits[(y * kCanvasSize + x) / 8] >> (x % 8)) & 1;
    }

    inline bool Canvas_IsEmpty() {
        for (size_t i = 0; i < sizeof(canvas_bits); ++i) {
            if (canvas_bits[i]) return false;
        }
        return true;
    }
}

void SystemClock_Config(void) {
    PCC_InitTypeDef PCC_OscInit = {0};
    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 8;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}

void GPIO_Init(void) {
    __HAL_PCC_GPIO_1_CLK_ENABLE();
    __HAL_PCC_GPIO_2_CLK_ENABLE();

    GPIO_InitTypeDef out = {};
    out.Pin = kLedPin;
    out.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    out.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_1, &out);
    HAL_GPIO_WritePin(GPIO_1, kLedPin, GPIO_PIN_LOW);

    //P1
    GPIO_InitTypeDef inp = {};
    inp.Pin = static_cast<HAL_PinsTypeDef>(GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_11);
    inp.Mode = HAL_GPIO_MODE_GPIO_INPUT;
    inp.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_1, &inp);
    //P2
    inp.Pin = static_cast<HAL_PinsTypeDef>(GPIO_PIN_6 | GPIO_PIN_7);
    inp.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_2, &inp);
}

USART_HandleTypeDef husart0;

void USART_Init(void) {
    husart0.Instance = UART_0;
    husart0.transmitting = Enable;
    husart0.receiving = Disable;
    husart0.frame = Frame_8bit;
    husart0.parity_bit = Disable;
    husart0.parity_bit_inversion = Disable;
    husart0.bit_direction = LSB_First;
    husart0.data_inversion = Disable;
    husart0.tx_inversion = Disable;
    husart0.rx_inversion = Disable;
    husart0.swap = Disable;
    husart0.lbm = Disable;
    husart0.stop_bit = StopBit_1;
    husart0.mode = Asynchronous_Mode;
    husart0.xck_mode = XCK_Mode3;
    husart0.last_byte_clock = Disable;
    husart0.overwrite = Disable;
    husart0.rts_mode = AlwaysEnable_mode;
    husart0.dma_tx_request = Disable;
    husart0.dma_rx_request = Disable;
    husart0.channel_mode = Duplex_Mode;
    husart0.tx_break_mode = Disable;
    husart0.Interrupt.ctsie = Disable;
    husart0.Interrupt.eie = Disable;
    husart0.Interrupt.idleie = Disable;
    husart0.Interrupt.lbdie = Disable;
    husart0.Interrupt.peie = Disable;
    husart0.Interrupt.rxneie = Disable;
    husart0.Interrupt.tcie = Disable;
    husart0.Interrupt.txeie = Disable;
    husart0.Modem.rts = Disable;
    husart0.Modem.cts = Disable;
    husart0.Modem.dtr = Disable;
    husart0.Modem.dcd = Disable;
    husart0.Modem.dsr = Disable;
    husart0.Modem.ri = Disable;
    husart0.Modem.ddis = Disable;
    husart0.baudrate = 9600;
    HAL_USART_Init(&husart0);
}

// управление
static bool WaitMsOrButton(uint32_t ms) {
    uint32_t t0 = HAL_Time_SCR1TIM_Millis();
    while (HAL_Time_SCR1TIM_Millis() - t0 < ms) {
        for (int b = 0; b < BTN_COUNT; ++b) {
            if (BtnDown(static_cast<Button>(b))) return true;
        }
    }
    return false;
}

// управление
static void WaitAllReleased() {
    for (;;) {
        bool any = false;
        for (int b = 0; b < BTN_COUNT; ++b) {
            if (BtnDown(static_cast<Button>(b))) {
                any = true;
                break;
            }
        }
        if (!any) return;
    }
}

// управление
static void WaitAnyButton() {
    for (;;) {
        for (int b = 0; b < BTN_COUNT; ++b) {
            if (BtnDown(static_cast<Button>(b))) return;
        }
    }
}

// интерфейс
static void Canvas_PaintBlob(int cx, int cy) {
    for (int dy = -kBrushR; dy <= kBrushR; ++dy) {
        for (int dx = -kBrushR; dx <= kBrushR; ++dx) {
            if (dx * dx + dy * dy > kBrushR * kBrushR) continue;
            int x = cx + dx, y = cy + dy;
            if (x < 0 || x >= kCanvasSize || y < 0 || y >= kCanvasSize) continue;
            Canvas_SetBit(x, y);
            LCD_DrawPixel(kCanvasX + x, kCanvasY + y, POLY_SOFT);
        }
    }
}

// интерфейс
static void Canvas_PaintLine(int x0, int y0, int x1, int y1) {
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int steps = (dx > dy) ? dx : dy;
    if (steps == 0) {
        Canvas_PaintBlob(x0, y0);
        return;
    }
    for (int i = 0; i <= steps; i += 2) {
        int x = x0 + (x1 - x0) * i / steps;
        int y = y0 + (y1 - y0) * i / steps;
        Canvas_PaintBlob(x, y);
    }
    Canvas_PaintBlob(x1, y1);
}

// интерфейс
static void Canvas_RedrawBox(int x0, int y0, int w, int h) {
    if (x0 < 0) {
        w += x0;
        x0 = 0;
    }
    if (y0 < 0) {
        h += y0;
        y0 = 0;
    }
    if (x0 + w > kCanvasSize) w = kCanvasSize - x0;
    if (y0 + h > kCanvasSize) h = kCanvasSize - y0;
    if (w <= 0 || h <= 0) return;

    uint16_t row[96];
    for (int y = 0; y < h; ++y) {
        int n = (w > 96) ? 96 : w;
        for (int x = 0; x < n; ++x) {
            row[x] = Canvas_GetBit(x0 + x, y0 + y) ? POLY_SOFT : BLACK;
        }
        LCD_PushRow(kCanvasX + x0, kCanvasY + y0 + y, n, row);
    }
}

// интерфейс
static void Cursor_Draw(int cx, int cy) {
    LCD_DrawCircle(kCanvasX + cx, kCanvasY + cy, kCursorR, POLY_ACCENT);
    LCD_DrawPixel(kCanvasX + cx, kCanvasY + cy, POLY_ACCENT);
}

// интерфейс
static void Cursor_Erase(int cx, int cy) {\
    Canvas_RedrawBox(cx - kCursorR - 1, cy - kCursorR - 1, 2 * (kCursorR + 1) + 1,
                     2 * (kCursorR + 1) + 1);
}

// интерфейс
static void LCD_Blit1(int x, int y, int w, int h, const uint8_t* bits, uint16_t fg, uint16_t bg) {
    const int rb = (w + 7) / 8;
    uint16_t row[w];
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            bool bit = (bits[j * rb + i / 8] >> (7 - (i % 8))) & 1;
            row[i] = bit ? fg : bg;
        }
        LCD_PushRow(x, y + j, w, row);
    }
}
// интерфейс
static void DrawCrest() {
    LCD_Blit1((LCD_W - LOGO72_W) / 2, 4, LOGO72_W, LOGO72_H, LOGO72, POLY_INK, POLY_BG);
}
// интерфейс
static void DrawBootPlate() {
    LCD_FillScreen(POLY_BG);
    LCD_Blit1((LCD_W - LOGO48_W) / 2, 30, LOGO48_W, LOGO48_H, LOGO48, POLY_INK, POLY_BG);
    LCD_TextCentered(88, "ПОЛИТЕХ", POLY_INK, POLY_BG, 2);
}
// интерфейс
static void DrawMiniCrest() {
    LCD_Blit1(120, 86, LOGO28_W, LOGO28_H, LOGO28, POLY_CARD_TXT, POLY_CARD);
}
// интерфейс
static void DrawScreenFrame() {
    LCD_FillScreen(POLY_BG);
    LCD_FillRect(0, 0, LCD_W, 12, POLY_ACCENT);
    LCD_TextCentered(2, "РИСУЙ ЦИФРУ", POLY_CARD_TXT, POLY_ACCENT, 1);
    LCD_FillRect(kCanvasX - 4, kCanvasY - 4, kCanvasSize + 8, kCanvasSize + 8, POLY_ACCENT);
    LCD_FillRect(kCanvasX - 2, kCanvasY - 2, kCanvasSize + 4, kCanvasSize + 4, POLY_INK);
    LCD_FillRect(kCanvasX, kCanvasY, kCanvasSize, kCanvasSize, BLACK);
    LCD_FillRect(kCardX, kCardY, kCardW, kCardH, POLY_CARD);
    DrawMiniCrest();
    LCD_FillRect(0, LCD_H - 10, LCD_W, 10, POLY_ACCENT);
    LCD_TextCentered(LCD_H - 9, "ПОЛИТЕХ", POLY_CARD_TXT, POLY_ACCENT, 1);
}
// интерфейс
static void DrawStatus(bool pen_down, int digit, int conf_pct) {
    LCD_FillRect(kCardX, kCardY, kCardW, 72, POLY_CARD);
    LCD_Text(kCardX + 4, 16, "ПЕРО", POLY_CARD_TXT, POLY_CARD, 1);
    LCD_Text(kCardX + 4, 26, pen_down ? "ОПУЩЕНО" : "ПОДНЯТО", pen_down ? POLY_HI : POLY_CARD_TXT,
             POLY_CARD, 1);
    LCD_Text(kCardX + 4, 42, "ПОСЛ", POLY_CARD_TXT, POLY_CARD, 1);
    if (digit < 0) {
        LCD_Text(kCardX + 4, 54, "--", POLY_CARD_TXT, POLY_CARD, 2);
    } else {
        char d[2];
        d[0] = static_cast<char>('0' + digit);
        d[1] = '\0';
        LCD_Text(kCardX + 4, 54, d, POLY_HI, POLY_CARD, 2);
        char pct[6];
        int n = 0;
        if (conf_pct >= 10) {
            pct[n++] = static_cast<char>('0' + conf_pct / 10);
        }
        pct[n++] = static_cast<char>('0' + conf_pct % 10);
        pct[n++] = '%';
        pct[n] = '\0';
        LCD_Text(kCardX + 4, 72, pct, POLY_CARD_TXT, POLY_CARD, 1);
    }
}
//интерфейс
static void ShowSplash() {
    LCD_FillScreen(POLY_BG);
    DrawCrest();
    LCD_TextCentered(82, "ПОЛИТЕХ", POLY_INK, POLY_BG, 2);
    LCD_TextCentered(102, "ПЕТРА ВЕЛИКОГО", POLY_ACCENT, POLY_BG, 1);
    LCD_TextCentered(114, "РИСУЙ ЦИФРУ", POLY_INK, POLY_BG, 1);
    WaitMsOrButton(1500);
}
// модель
static void DownsampleToInput(TfLiteTensor* input) {
    const float scale = input->params.scale;
    const int32_t zp = input->params.zero_point;

    for (int i = 0; i < 256; ++i) {
        input->data.int8[i] = static_cast<int8_t>(zp);
    }

    int x0 = kCanvasSize, y0 = kCanvasSize, x1 = -1, y1 = -1;
    for (int y = 0; y < kCanvasSize; ++y) {
        for (int x = 0; x < kCanvasSize; ++x) {
            if (Canvas_GetBit(x, y)) {
                if (x < x0) x0 = x;
                if (x > x1) x1 = x;
                if (y < y0) y0 = y;
                if (y > y1) y1 = y;
            }
        }
    }
    if (x1 < x0) return;

    // Поля 3px, обрезка по холсту.
    const int kMargin = 3;
    if (x0 > kMargin) x0 -= kMargin; else x0 = 0;
    if (y0 > kMargin) y0 -= kMargin; else y0 = 0;
    if (x1 + kMargin < kCanvasSize) x1 += kMargin; else x1 = kCanvasSize - 1;
    if (y1 + kMargin < kCanvasSize) y1 += kMargin; else y1 = kCanvasSize - 1;
    const int bw = x1 - x0 + 1;
    const int bh = y1 - y0 + 1;

    // Вписать длинной стороной в 12px, пропорции сохранить.
    const int kFit = 12;
    const int long_side = (bw > bh) ? bw : bh;
    int ow = (bw * kFit + long_side / 2) / long_side;
    int oh = (bh * kFit + long_side / 2) / long_side;
    if (ow < 1) ow = 1;
    if (oh < 1) oh = 1;
    if (ow > 16) ow = 16;
    if (oh > 16) oh = 16;
    const int ox = (16 - ow) / 2;
    const int oy = (16 - oh) / 2;

    for (int ty = 0; ty < oh; ++ty) {
        int sy0 = y0 + (ty * bh) / oh;
        int sy1 = y0 + ((ty + 1) * bh) / oh;
        if (sy1 <= sy0) sy1 = sy0 + 1;
        if (sy1 > y1 + 1) sy1 = y1 + 1;
        for (int tx = 0; tx < ow; ++tx) {
            int sx0 = x0 + (tx * bw) / ow;
            int sx1 = x0 + ((tx + 1) * bw) / ow;
            if (sx1 <= sx0) sx1 = sx0 + 1;
            if (sx1 > x1 + 1) sx1 = x1 + 1;
            int sum = 0;
            for (int y = sy0; y < sy1; ++y) {
                for (int x = sx0; x < sx1; ++x) {
                    if (Canvas_GetBit(x, y)) ++sum;
                }
            }
            const int area = (sx1 - sx0) * (sy1 - sy0);
            float v = (area > 0) ? (static_cast<float>(sum) / static_cast<float>(area)) : 0.0f;
            int q = static_cast<int>(v / scale + static_cast<float>(zp));
            if (q > 127) q = 127;
            if (q < -128) q = -128;
            input->data.int8[(oy + ty) * 16 + (ox + tx)] = static_cast<int8_t>(q);
        }
    }
}

static int RunInference(int* conf_pct) {
    tflite::MicroMutableOpResolver<8> resolver; // подключение всех необходимых слоев
    if (resolver.AddConv2D() != kTfLiteOk) return -1;
    if (resolver.AddMaxPool2D() != kTfLiteOk) return -1;
    if (resolver.AddShape() != kTfLiteOk) return -1;
    if (resolver.AddStridedSlice() != kTfLiteOk) return -1;
    if (resolver.AddPack() != kTfLiteOk) return -1;
    if (resolver.AddReshape() != kTfLiteOk) return -1;
    if (resolver.AddFullyConnected() != kTfLiteOk) return -1;
    if (resolver.AddSoftmax() != kTfLiteOk) return -1;

    const tflite::Model* model = tflite::GetModel(model_16x16_int8_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) { // проверка сохранности самой модели
        MicroPrintf("Model schema version mismatch!");
        return -1;
    }

    tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, kTensorArenaSize); // подключение исполнителя
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        MicroPrintf("AllocateTensors() failed");
        return -1;
    }

    TfLiteTensor* input = interpreter.input(0);
    DownsampleToInput(input);

    static int8_t saved[kModelInputSize];
    std::memcpy(saved, input->data.int8, kModelInputSize);

    uint64_t sum1 = 0;
    std::memcpy(input->data.int8, saved, kModelInputSize);
    const uint64_t t0 = HAL_Time_SCR1TIM_Micros();
    const TfLiteStatus st = interpreter.Invoke();
    sum1 += HAL_Time_SCR1TIM_Micros() - t0;
    if (st != kTfLiteOk) {
        MicroPrintf("Invoke() failed");
        return -1;
    }

    TfLiteTensor* output = interpreter.output(0);
    const float out_scale = output->params.scale;
    const int32_t out_zp = output->params.zero_point;
    int best = 0;
    float best_v = -1e30f;
    float sum = 0.0f;
    for (int i = 0; i < kModelOutputSize; ++i) {
        float v = (static_cast<float>(output->data.int8[i]) - static_cast<float>(out_zp)) * out_scale;
        sum += v;
        if (v > best_v) {
            best_v = v;
            best = i;
        }
    }
    if (sum < 1e-6f) sum = 1e-6f;
    int pct = static_cast<int>(best_v / sum * 100.0f);
    if (pct < 0) pct = 0;
    if (pct > 99) pct = 99;
    *conf_pct = pct;
    MicroPrintf("Recognized digit: %d (%d%%)", best, pct);
    return best;
}

// интерфейс
static void SweepAnimation() {
    for (int y = 0; y < kCanvasSize; y += 12) {
        LCD_FillRect(kCanvasX, kCanvasY + y, kCanvasSize, 10, POLY_ACCENT);
        HAL_DelayMs(28);
        Canvas_RedrawBox(0, y, kCanvasSize, 10);
    }
    HAL_DelayMs(80);
}

// интерфейс
static void ShowResult(int digit, int conf_pct) {
    LCD_FillScreen(POLY_CARD);
    LCD_TextCentered(4, "РЕЗУЛЬТАТ", POLY_CARD_TXT, POLY_CARD, 1);
    LCD_SevenSeg((LCD_W - 34) / 2, 16, digit, 20, 7, POLY_HI, POLY_CARD);

    char pct[6];
    int n = 0;
    if (conf_pct >= 10) {
        pct[n++] = static_cast<char>('0' + conf_pct / 10);
    }
    pct[n++] = static_cast<char>('0' + conf_pct % 10);
    pct[n++] = '%';
    pct[n] = '\0';
    LCD_TextCentered(82, pct, POLY_CARD_TXT, POLY_CARD, 2);

    const int barX = 20, barY = 102, barW = LCD_W - 40, barH = 8;
    LCD_FillRect(barX - 2, barY - 2, barW + 4, barH + 4, POLY_CARD_TXT);
    LCD_FillRect(barX, barY, barW, barH, POLY_BG);
    const int target = (barW * conf_pct) / 100;
    for (int w = 0; w <= target; w += 12) {
        LCD_FillRect(barX, barY, w, barH, POLY_HI);
        HAL_DelayMs(25);
    }
    LCD_FillRect(barX, barY, target, barH, POLY_HI);
    LCD_TextCentered(116, "НАЖМИ КНОПКУ", POLY_CARD_TXT, POLY_CARD, 1);
}
// интерфейс
static void RedrawCanvas() {
    for (int y = 0; y < kCanvasSize; ++y) {
        uint16_t row[96];
        for (int x = 0; x < kCanvasSize; ++x) {
            row[x] = Canvas_GetBit(x, y) ? POLY_SOFT : BLACK;
        }
        LCD_PushRow(kCanvasX, kCanvasY + y, kCanvasSize, row);
    }
}

int main() {
    SystemClock_Config(); // 32 МГц
    GPIO_Init();
    USART_Init(); // 9600
    tflite::InitializeTarget();
    LCD_Init();

    MicroPrintf("MikBoy digits 160x128: UI start");
    HAL_GPIO_WritePin(GPIO_1, kLedPin, GPIO_PIN_HIGH);

    DrawBootPlate(); // 1я заставка
    ShowSplash();// 2я заставка

    std::memset(canvas_bits, 0, sizeof(canvas_bits));
    DrawScreenFrame();

    int cx = kCanvasSize / 2, cy = kCanvasSize / 2;
    bool pen_down = true;
    int last_digit = -1, last_conf = 0;
    DrawStatus(pen_down, last_digit, last_conf);
    Cursor_Draw(cx, cy);

    bool was_down[BTN_COUNT] = {};
    bool deb_a = false, deb_b = false;
    uint8_t deb_a_cnt = 0, deb_b_cnt = 0;
    bool was_moving = false;
    uint32_t last_move = 0;
    uint32_t a_press_at = 0;
    bool a_hold_fired = false;
    uint32_t last_poll = HAL_Time_SCR1TIM_Millis();
    uint32_t led_at = HAL_Time_SCR1TIM_Millis();
    bool led_on = true;

    while (true) {
        uint32_t now = HAL_Time_SCR1TIM_Millis();
        if (now - last_poll < kPollMs) {
            continue;
        }
        last_poll = now;

        if (now - led_at > 500) {
            led_at = now;
            led_on = !led_on;
            HAL_GPIO_WritePin(GPIO_1, kLedPin, led_on ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        }

        // --- Крестовина: движение (+ рисование, если перо опущено) ---
        int dx = 0, dy = 0;
        if (BtnDown(BTN_LEFT)) dx -= 1;
        if (BtnDown(BTN_RIGHT)) dx += 1;
        if (BtnDown(BTN_UP)) dy -= 1;
        if (BtnDown(BTN_DOWN)) dy += 1;
        bool moving = (dx != 0 || dy != 0);

        // Первый тик движения — шаг сразу; дальше автоповтор каждые kMoveRepeatMs.
        if (moving && (!was_moving || (now - last_move >= kMoveRepeatMs))) {
            last_move = now;
            Cursor_Erase(cx, cy);
            int nx = cx + dx * kMoveStep;
            int ny = cy + dy * kMoveStep;
            if (nx < 0) nx = 0;
            if (nx >= kCanvasSize) nx = kCanvasSize - 1;
            if (ny < 0) ny = 0;
            if (ny >= kCanvasSize) ny = kCanvasSize - 1;
            if (pen_down) {
                Canvas_PaintLine(cx, cy, nx, ny);
            }
            cx = nx;
            cy = ny;
            Cursor_Draw(cx, cy);
        }
        was_moving = moving;

        bool a_raw = BtnDown(BTN_A);
        if (a_raw == deb_a) {
            deb_a_cnt = 0;
        } else if (++deb_a_cnt >= 2) {
            deb_a = a_raw;
            deb_a_cnt = 0;
        }
        bool b_raw = BtnDown(BTN_B);
        if (b_raw == deb_b) {
            deb_b_cnt = 0;
        } else if (++deb_b_cnt >= 2) {
            deb_b = b_raw;
            deb_b_cnt = 0;
        }
        bool a_now = deb_a;
        if (a_now && !was_down[BTN_A]) {
            a_press_at = now;
            a_hold_fired = false;
        }

        if (a_now && !a_hold_fired && (now - a_press_at >= kGuessHoldMs)) {
            a_hold_fired = true;
            Cursor_Erase(cx, cy);
            if (Canvas_IsEmpty()) {
                LCD_Text(kCanvasX + (kCanvasSize - 89) / 2, kCanvasY + 44, "СНАЧАЛА НАРИСУЙ",
                         POLY_ACCENT, BLACK, 1);
                HAL_DelayMs(700);
                RedrawCanvas();
            } else {
                SweepAnimation();
                HAL_GPIO_WritePin(GPIO_1, kLedPin, GPIO_PIN_HIGH);
                int conf = 0;
                int digit = RunInference(&conf);
                if (digit >= 0) {
                    last_digit = digit;
                    last_conf = conf;
                    ShowResult(digit, conf);
                    WaitAnyButton();
                    WaitAllReleased();
                    for (int b = 0; b < BTN_COUNT; ++b) was_down[b] = false;
                    a_hold_fired = true;
                    DrawScreenFrame();
                    RedrawCanvas();
                    DrawStatus(pen_down, last_digit, last_conf);
                } else {
                    LCD_Text(kCanvasX + (kCanvasSize - 71) / 2, kCanvasY + 40, "ОШИБКА",
                             POLY_ACCENT, BLACK, 2);
                    HAL_DelayMs(700);
                    RedrawCanvas();
                }
            }
            Cursor_Draw(cx, cy);
        }

        if (!a_now && was_down[BTN_A] && !a_hold_fired) {
            pen_down = !pen_down;
            DrawStatus(pen_down, last_digit, last_conf);
        }
        was_down[BTN_A] = a_now;

        bool b_now = deb_b;
        if (b_now && !was_down[BTN_B]) {
            Cursor_Erase(cx, cy);
            for (int y = 0; y < kCanvasSize; y += 16) {
                LCD_FillRect(kCanvasX, kCanvasY + y, kCanvasSize, 12, POLY_ACCENT);
                HAL_DelayMs(25);
            }
            std::memset(canvas_bits, 0, sizeof(canvas_bits));
            RedrawCanvas();
            Cursor_Draw(cx, cy);
        }
        was_down[BTN_B] = b_now;
    }

    return 0;
}
