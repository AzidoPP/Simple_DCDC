#include "input.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "shared_data.h"
#include "stdbool.h"

#define NUM_KEYS 5
#define DEBOUNCE_TICKS 20
#define REPEAT_PERIOD   10
#define REPEAT_START    500

typedef struct {
    bool pressed;
    uint32_t press_tick;
    uint32_t last_evt_tick;
    uint32_t last_repeat_tick;
} KeyState_t;

static KeyState_t keys[NUM_KEYS] = {0};
static GPIO_TypeDef* key_ports[NUM_KEYS] = { BUTTON_UP_GPIO_Port,
                                             BUTTON_DOWN_GPIO_Port,
                                             BUTTON_LEFT_GPIO_Port,
                                             BUTTON_RIGHT_GPIO_Port,
                                             BUTTON_MID_GPIO_Port };
static uint16_t key_pins[NUM_KEYS] = { BUTTON_UP_Pin,
                                       BUTTON_DOWN_Pin,
                                       BUTTON_LEFT_Pin,
                                       BUTTON_RIGHT_Pin,
                                       BUTTON_MID_Pin };

void Input_Init(void)
{
    // 如果需要初始化可以在这里加
}

void Input_HandleEvent(BtnEvent_t *evt)
{
    if (evt->key_id >= NUM_KEYS) return;
    KeyState_t *k = &keys[evt->key_id];
    if ((evt->tick - k->last_evt_tick) < DEBOUNCE_TICKS) return;

    k->last_evt_tick = evt->tick;
    if (evt->type == BTN_EVENT_PRESS && !k->pressed) {
        k->pressed = true;
        k->press_tick = evt->tick;
        k->last_repeat_tick = evt->tick;

        // LEFT/RIGHT 翻页
        if (evt->key_id == 2 || evt->key_id == 3) {
            uint8_t old_page = g_display_page;
            g_display_page = (g_display_page + (evt->key_id == 3 ? 1 : DISPLAY_NUM_PAGES - 1)) % DISPLAY_NUM_PAGES;
            if (old_page != g_display_page && sysEventFlagsHandle)
                osEventFlagsSet(sysEventFlagsHandle, EVT_PAGE_CHANGED);
        }
        // UP/DOWN 调整当前页设置
        else if (evt->key_id == 0 || evt->key_id == 1) {
            if (osMutexAcquire(ADJSetMutexHandle, 0) == osOK) {
                float step = 0.1f;
                if (g_display_page == 0) {
                    g_adj_target.adj1_set_v += (evt->key_id == 0 ? step : -step);
                    if (g_adj_target.adj1_set_v > 24.0f) g_adj_target.adj1_set_v = 24.0f;
                    if (g_adj_target.adj1_set_v < 0.0f) g_adj_target.adj1_set_v = 0.0f;
                } else if (g_display_page == 1) {
                    g_adj_target.adj2_set_v += (evt->key_id == 0 ? step : -step);
                    if (g_adj_target.adj2_set_v > 24.0f) g_adj_target.adj2_set_v = 24.0f;
                    if (g_adj_target.adj2_set_v < 0.0f) g_adj_target.adj2_set_v = 0.0f;
                }
                osMutexRelease(ADJSetMutexHandle);
                if (sysEventFlagsHandle) osEventFlagsSet(sysEventFlagsHandle, EVT_ADJ_TARGET_UPDATED);
            }
        }
    } else if (evt->type == BTN_EVENT_RELEASE) {
        k->pressed = false;
        k->last_repeat_tick = 0;
    }
}

void Input_PeriodicTask(void)
{
    uint32_t now = xTaskGetTickCount();
    for (int i = 0; i < NUM_KEYS; i++) {
        KeyState_t *k = &keys[i];
        if (!k->pressed) continue;
        if (HAL_GPIO_ReadPin(key_ports[i], key_pins[i]) != GPIO_PIN_RESET) { k->pressed = false; continue; }
        if (now - k->press_tick >= REPEAT_START && now - k->last_repeat_tick >= REPEAT_PERIOD) {
            // 重复处理逻辑和短按一样
            BtnEvent_t evt = { .key_id = i, .type = BTN_EVENT_PRESS, .tick = now };
            Input_HandleEvent(&evt);
            k->last_repeat_tick = now;
        }
    }
}
