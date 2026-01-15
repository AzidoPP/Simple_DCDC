/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "power_switch.h"
#include "shared_data.h"
#include "adc.h"
#include "tim.h"
#include <string.h>
#include "LED.h"
#include "u8g2.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ====== 数控电源参数（按你描述）====== */
/* 输出级：Vout = 8 * Vref_equiv；Vref_equiv 来自 PWM 经 RC 滤波形成的等效 DAC 电压 */
#define PSU_GAIN_OUT              (8.0f)

/* 反馈分压：Vfb = Vout * (R_BOT / (R_TOP + R_BOT)) */
#define FB_R_TOP_OHM              (30000.0f)
#define FB_R_BOT_OHM              (1800.0f)

/* ADC 量化：按你说“满量程 4096”，这里用 4096.0f（STM32 12bit 实际满量程码为 4095） */
#define ADC_FULL_SCALE_F          (4096.0f)

/* PA3：2.5V 标定电压芯片（用于推算 VDDA/3.3V） */
#define VREF_CAL_VOLT             (2.500f)

/* 输出电压设定范围（按常见 0~24V，若你硬件不同可改） */
#define VSET_MIN_V                (0.0f)
#define VSET_MAX_V                (24.0f)

/* 控制周期（ControlTask） */
#define CTRL_PERIOD_MS            (10U)
#define CTRL_DT_S                 (0.010f)

/* 积分参数（前馈 + 慢积分补偿）
   error 单位: V，积分输出单位: duty (0..1)
   integ += Ki * error * dt
*/
#define CTRL_KI                   (0.25f)   /* 可按实际响应调：小则更稳但更慢 */
#define INTEG_LIMIT               (0.25f)   /* 积分限幅，防 windup */

/* Duty 限幅 */
#define DUTY_MIN                  (0.0f)
#define DUTY_MAX                  (1.0f)

/* ADC 低通滤波（EMA）alpha：越小越慢、越稳 */
#define ADC_EMA_ALPHA             (0.05f)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ====== 共享：设定值 / 测量值 / duty（用 mutex 保护）====== */
static float s_vset_v = 12.0f;      /* 设定输出电压 */
static float s_vout_v = 0.0f;       /* 测得输出电压 */
static float s_vfb_v  = 0.0f;       /* 测得反馈点电压（PA4） */
static float s_vdda_v = 3.30f;      /* 推算的 VDDA（ADC 参考 & PWM 高电平） */
static float s_duty_cmd = 0.0f;     /* 最终输出占空比（写 TIM1 CH1） */

/* EMA 状态（内部用，无需 mutex：仅 ADCTask 写，Control/UI 读时用 mutex 拷贝即可） */
static float s_ema_v25_adc = 0.0f;  /* PA3 ADC 原码 EMA */
static float s_ema_vfb_adc = 0.0f;  /* PA4 ADC 原码 EMA */

/* 控制积分项 */
static float s_integ = 0.0f;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
  .name = "ADCTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ButtonTask */
osThreadId_t ButtonTaskHandle;
const osThreadAttr_t ButtonTask_attributes = {
  .name = "ButtonTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for buttonQueue */
osMessageQueueId_t buttonQueueHandle;
const osMessageQueueAttr_t buttonQueue_attributes = {
  .name = "buttonQueue"
};
/* Definitions for ADJSetMutex */
osMutexId_t ADJSetMutexHandle;
const osMutexAttr_t ADJSetMutex_attributes = {
  .name = "ADJSetMutex"
};
/* Definitions for adcDmaSem */
osSemaphoreId_t adcDmaSemHandle;
const osSemaphoreAttr_t adcDmaSem_attributes = {
  .name = "adcDmaSem"
};
/* Definitions for i2cTxSem */
osSemaphoreId_t i2cTxSemHandle;
const osSemaphoreAttr_t i2cTxSem_attributes = {
  .name = "i2cTxSem"
};
/* Definitions for sysEventFlags */
osEventFlagsId_t sysEventFlagsHandle;
const osEventFlagsAttr_t sysEventFlags_attributes = {
  .name = "sysEventFlags"
};

/* USER CODE BEGIN Variables_Extra */
/* （可选）如你想把测量/设定拆开 mutex，也可以新增；这里复用 ADJSetMutexHandle 统一保护 */
/* USER CODE END Variables_Extra */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float ema_update(float prev, float x, float a)
{
  return prev + a * (x - prev);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartADCTask(void *argument);
void StartControlTask(void *argument);
void StartButtonTask(void *argument);
void StartDisplayTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* creation of ADJSetMutex */
  ADJSetMutexHandle = osMutexNew(&ADJSetMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of adcDmaSem */
  adcDmaSemHandle = osSemaphoreNew(1, 0, &adcDmaSem_attributes);

  /* creation of i2cTxSem */
  i2cTxSemHandle = osSemaphoreNew(1, 0, &i2cTxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of buttonQueue */
  buttonQueueHandle = osMessageQueueNew (8, 8, &buttonQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ADCTask */
  ADCTaskHandle = osThreadNew(StartADCTask, NULL, &ADCTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of ButtonTask */
  ButtonTaskHandle = osThreadNew(StartButtonTask, NULL, &ButtonTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of sysEventFlags */
  sysEventFlagsHandle = osEventFlagsNew(&sysEventFlags_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
* @brief Function implementing the ADCTask thread.
*        - 启动 ADC1 DMA 循环采样
*        - 使用 PA3(2.5V) 推算 VDDA
*        - 使用 PA4(FB) 计算 Vout
*        - 对 ADC 原码做 EMA 滤波
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCTask */
void StartADCTask(void *argument)
{
  /* USER CODE BEGIN StartADCTask */

  /* adc.c 里定义的 DMA 缓冲区（7通道） */
  extern uint16_t adc_buffer[7];
  extern ADC_HandleTypeDef hadc1;

  /* ADC 校准（F1 支持） */
  (void)HAL_ADCEx_Calibration_Start(&hadc1);

  /* 启动 DMA */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 7) != HAL_OK) {
    /* 如果启动失败，仍然让任务跑着，避免系统卡死 */
  }

  /* 初始化 EMA 为当前值，避免启动时跳变 */
  osDelay(50);
  {
    float a25 = (float)adc_buffer[3]; /* Rank4: ADC_CHANNEL_3 -> PA3 -> V2_5 */
    float afb = (float)adc_buffer[4]; /* Rank5: ADC_CHANNEL_4 -> PA4 -> Feedback */
    s_ema_v25_adc = a25;
    s_ema_vfb_adc = afb;
  }

  for (;;)
  {
    /* 等待 DMA half/full 完成回调释放信号量（你在 adc.c 的 ConvCpltCallback 里 release 了） */
    if (osSemaphoreAcquire(adcDmaSemHandle, osWaitForever) != osOK) {
      continue;
    }

    /* 取本次 DMA buffer 的目标通道 */
    float adc_v25 = (float)adc_buffer[3];  /* PA3 */
    float adc_vfb = (float)adc_buffer[4];  /* PA4 */

    /* EMA 滤波（对 ADC 原码做） */
    s_ema_v25_adc = ema_update(s_ema_v25_adc, adc_v25, ADC_EMA_ALPHA);
    s_ema_vfb_adc = ema_update(s_ema_vfb_adc, adc_vfb, ADC_EMA_ALPHA);

    /* 用 2.5V 推算 VDDA（ADC参考/供电） */
    float vdda = s_vdda_v;
    if (s_ema_v25_adc > 1.0f) {
      vdda = VREF_CAL_VOLT * (ADC_FULL_SCALE_F / s_ema_v25_adc);
    }
    /* 合理限幅（防止异常时算飞） */
    vdda = clampf(vdda, 2.8f, 3.6f);

    /* 计算 Vfb / Vout */
    float vfb = (s_ema_vfb_adc / ADC_FULL_SCALE_F) * vdda;
    float vout = vfb * ((FB_R_TOP_OHM + FB_R_BOT_OHM) / FB_R_BOT_OHM);

    /* 写共享数据 */
    if (osMutexAcquire(ADJSetMutexHandle, 5) == osOK) {
      s_vdda_v = vdda;
      s_vfb_v  = vfb;
      s_vout_v = vout;
      osMutexRelease(ADJSetMutexHandle);
    } else {
      /* 兜底：尽量不发生 */
      s_vdda_v = vdda;
      s_vfb_v  = vfb;
      s_vout_v = vout;
    }
  }

  /* USER CODE END StartADCTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
*        - 输出 PA8(TIM1_CH1) PWM 到 RC 形成等效 DAC
*        - 控制策略：前馈 + 慢积分
*          duty_ff = (Vset/8) / VDDA
*          duty = duty_ff + integ
*          integ += Ki * (Vset - Vout_meas) * dt
*        - 写 TIM1 CCR1
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */

  /* 启动 TIM1 CH1 及互补输出 CH1N（如果你只用 PA8，可保留也可删 CH1N） */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  if (arr == 0) arr = 1;

  for (;;)
  {
    float vset, vout, vdda;

    /* 读取共享测量/设定 */
    if (osMutexAcquire(ADJSetMutexHandle, 5) == osOK) {
      vset = s_vset_v;
      vout = s_vout_v;
      vdda = s_vdda_v;
      osMutexRelease(ADJSetMutexHandle);
    } else {
      vset = s_vset_v;
      vout = s_vout_v;
      vdda = s_vdda_v;
    }

    /* 前馈：Vref_equiv = Vset / 8；Vref_equiv = duty * VDDA */
    float duty_ff = 0.0f;
    if (vdda > 0.1f) {
      duty_ff = (vset / PSU_GAIN_OUT) / vdda;
    }
    duty_ff = clampf(duty_ff, DUTY_MIN, DUTY_MAX);

    /* 误差（V） */
    float err = vset - vout;

    /* 慢积分补偿 */
    s_integ += (CTRL_KI * err * CTRL_DT_S);
    s_integ = clampf(s_integ, -INTEG_LIMIT, INTEG_LIMIT);

    /* 合成 duty 并限幅 */
    float duty = duty_ff + s_integ;
    duty = clampf(duty, DUTY_MIN, DUTY_MAX);

    /* 写 PWM */
    uint32_t cmp = (uint32_t)(duty * (float)(arr + 1u));
    if (cmp > arr) cmp = arr;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);

    /* 写共享给 UI/调试 */
    if (osMutexAcquire(ADJSetMutexHandle, 2) == osOK) {
      s_duty_cmd = duty;
      osMutexRelease(ADJSetMutexHandle);
    } else {
      s_duty_cmd = duty;
    }

    /* 你原来使用的 shared_data 示例保留：用于外部查看 compare */
    g_pwm_cmd.pwm1 = (uint16_t)cmp;

    osDelay(CTRL_PERIOD_MS);
  }

  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
* @brief Function implementing the ButtonTask thread.
*        - 用 UP/DOWN 调整“输出电压设定值 Vset”
*        - 短按：小步进；按住：连发大步进
*        - 其余按键保留结构，后续可扩展（例如 MID 切换输出使能等）
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
  #define NUM_KEYS 5
  #define DEBOUNCE_TICKS 20   // 20 ms
  #define REPEAT_PERIOD   80  // 80 ms repeat period
  #define REPEAT_START    400 // 400 ms after press starts repeat

  typedef struct {
      bool     pressed;
      uint32_t press_tick;
      uint32_t last_evt_tick;
      uint32_t last_repeat_tick;
  } KeyState_t;

  static KeyState_t keys[NUM_KEYS] = {0};

  GPIO_TypeDef* key_ports[NUM_KEYS] = { BUTTON_UP_GPIO_Port,
                                        BUTTON_DOWN_GPIO_Port,
                                        BUTTON_LEFT_GPIO_Port,
                                        BUTTON_RIGHT_GPIO_Port,
                                        BUTTON_MID_GPIO_Port };
  uint16_t key_pins[NUM_KEYS] = { BUTTON_UP_Pin,
                                  BUTTON_DOWN_Pin,
                                  BUTTON_LEFT_Pin,
                                  BUTTON_RIGHT_Pin,
                                  BUTTON_MID_Pin };

  BtnEvent_t evt;
  osStatus_t status;
  uint32_t now;

  /* 电压步进：可按你想要的 UI 手感改 */
  const float step_short_v = 0.10f;  /* 短按 0.10V */
  const float step_long_v  = 0.50f;  /* 连发 0.50V */

  for (;;)
  {
    bool any_pressed = false;
    for (int i = 0; i < NUM_KEYS; i++) {
      if (keys[i].pressed) { any_pressed = true; break; }
    }

    uint32_t timeout = any_pressed ? REPEAT_PERIOD : osWaitForever;
    status = osMessageQueueGet(buttonQueueHandle, &evt, NULL, timeout);
    now = xTaskGetTickCount();

    /* 处理来自 ISR 的按键事件（消抖 + 立即动作） */
    if (status == osOK) {
      if (evt.key_id < NUM_KEYS) {
        KeyState_t *k = &keys[evt.key_id];

        if ((evt.tick - k->last_evt_tick) >= DEBOUNCE_TICKS) {
          k->last_evt_tick = evt.tick;

          if (evt.type == BTN_EVENT_PRESS) {
            if (!k->pressed) {
              k->pressed = true;
              k->press_tick = evt.tick;
              k->last_repeat_tick = evt.tick;

              /* 立即动作：UP/DOWN 调 Vset */
              if (evt.key_id == 0 || evt.key_id == 1) {
                if (osMutexAcquire(ADJSetMutexHandle, 10) == osOK) {
                  if (evt.key_id == 0) s_vset_v += step_short_v;  /* UP */
                  else                 s_vset_v -= step_short_v;  /* DOWN */
                  s_vset_v = clampf(s_vset_v, VSET_MIN_V, VSET_MAX_V);
                  osMutexRelease(ADJSetMutexHandle);
                } else {
                  if (evt.key_id == 0) s_vset_v += step_short_v;
                  else                 s_vset_v -= step_short_v;
                  s_vset_v = clampf(s_vset_v, VSET_MIN_V, VSET_MAX_V);
                }
              }

              /* 你后续可以扩展：
                 - LEFT/RIGHT 改变步进大小
                 - MID 切换输出 enable / 清积分等
              */
            }
          } else { /* RELEASE */
            if (k->pressed) {
              k->pressed = false;
              k->last_repeat_tick = 0;
            }
          }
        }
      }
    }

    /* 按住连发 + 物理确认避免卡死 */
    for (int i = 0; i < NUM_KEYS; i++) {
      KeyState_t *k = &keys[i];
      if (!k->pressed) continue;

      /* 物理确认：松开则清状态（GPIO 上拉，按下为 RESET） */
      if (HAL_GPIO_ReadPin(key_ports[i], key_pins[i]) != GPIO_PIN_RESET) {
        k->pressed = false;
        continue;
      }

      uint32_t held = now - k->press_tick;

      if ((i == 0 || i == 1) && held >= REPEAT_START) {
        if ((now - k->last_repeat_tick) >= REPEAT_PERIOD) {
          if (osMutexAcquire(ADJSetMutexHandle, 10) == osOK) {
            if (i == 0) s_vset_v += step_long_v;
            else        s_vset_v -= step_long_v;
            s_vset_v = clampf(s_vset_v, VSET_MIN_V, VSET_MAX_V);
            osMutexRelease(ADJSetMutexHandle);
          } else {
            if (i == 0) s_vset_v += step_long_v;
            else        s_vset_v -= step_long_v;
            s_vset_v = clampf(s_vset_v, VSET_MIN_V, VSET_MAX_V);
          }
          k->last_repeat_tick = now;
        }
      }
    }
  }
  /* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
*        u8g2 UI：显示
*          - 设定电压 Vset
*          - 实测电压 Vout
*          - VDDA(由2.5V标定推算)
*          - PWM duty
*        并提供简单进度条
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
/* ===== 手动格式化辅助函数（放在 freertos.c 顶部 USER CODE BEGIN FunctionPrototypes 附近也行） ===== */
static void fmt_u32_zeropad(char *dst, uint32_t val, uint8_t width)
{
  /* 把 val 以 width 位十进制、左侧补 0 写到 dst，写完不加 '\0' */
  for (int i = (int)width - 1; i >= 0; --i) {
    dst[i] = (char)('0' + (val % 10u));
    val /= 10u;
  }
}

static void fmt_v_x100(char *out, const char *prefix, float v) /* 0.01V */
{
  /* 输出形如: "SET: 12.34 V" / "OUT:  0.05 V" （整数部分最少2位，左侧空格） */
  int32_t x = (int32_t)(v * 100.0f + (v >= 0 ? 0.5f : -0.5f));
  if (x < 0) x = 0; /* 这里按你的电源不需要负电压，防御 */

  uint32_t ip = (uint32_t)(x / 100);
  uint32_t fp = (uint32_t)(x % 100);

  /* prefix 拷贝 */
  char *p = out;
  while (*prefix) *p++ = *prefix++;

  /* 整数部分：至少2位，左侧空格（比如 " 0" ~ "99"），更大就自然扩展 */
  if (ip < 10) {
    *p++ = ' ';
    *p++ = (char)('0' + ip);
  } else if (ip < 100) {
    *p++ = (char)('0' + (ip / 10));
    *p++ = (char)('0' + (ip % 10));
  } else {
    /* >=100：不补空格，直接写（最多到 9999 都行） */
    /* 简单起见写 4 位上限，你可按需求扩 */
    uint32_t tmp = ip;
    char buf[10];
    int n = 0;
    do { buf[n++] = (char)('0' + (tmp % 10u)); tmp /= 10u; } while (tmp && n < (int)sizeof(buf));
    for (int i = n - 1; i >= 0; --i) *p++ = buf[i];
  }

  *p++ = '.';
  fmt_u32_zeropad(p, fp, 2); p += 2;

  *p++ = ' ';
  *p++ = 'V';
  *p++ = '\0';
}

static void fmt_vdda_and_duty(char *out, float vdda, float duty)
{
  /* 输出形如: "VDDA:3.300  Duty: 75%" */
  int32_t mv = (int32_t)(vdda * 1000.0f + (vdda >= 0 ? 0.5f : -0.5f)); /* mV */
  if (mv < 0) mv = 0;
  uint32_t ip = (uint32_t)(mv / 1000);
  uint32_t fp = (uint32_t)(mv % 1000);

  int32_t pct = (int32_t)(duty * 100.0f + 0.5f);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  char *p = out;

  /* "VDDA:" */
  *p++='V';*p++='D';*p++='D';*p++='A';*p++=':';

  /* ip（1位够用：3） */
  *p++ = (char)('0' + (ip % 10u));

  *p++='.';
  fmt_u32_zeropad(p, fp, 3); p += 3;

  *p++=' '; *p++=' '; /* 两个空格 */

  /* "Duty:" */
  *p++='D';*p++='u';*p++='t';*p++='y';*p++=':';

  /* 百分比宽度3，左侧空格（"  0".."100"） */
  if (pct < 10) {
    *p++=' '; *p++=' ';
    *p++=(char)('0'+pct);
  } else if (pct < 100) {
    *p++=' ';
    *p++=(char)('0'+(pct/10));
    *p++=(char)('0'+(pct%10));
  } else { /* 100 */
    *p++='1'; *p++='0'; *p++='0';
  }

  *p++='%';
  *p++='\0';
}

void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  extern uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
  extern uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

  u8g2_t u8g2;
  u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0,
                                       u8x8_byte_stm32_hw_i2c,
                                       u8x8_gpio_and_delay_stm32);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);

  /* 本 UI 目标：突出“设定电压”居中大字，其余信息小字放角落 */
  char line[32];

  for (;;)
  {
    float vset, vout, vdda, duty;

    if (osMutexAcquire(ADJSetMutexHandle, 5) == osOK) {
      vset = s_vset_v;
      vout = s_vout_v;
      vdda = s_vdda_v;
      duty = s_duty_cmd;
      osMutexRelease(ADJSetMutexHandle);
    } else {
      vset = s_vset_v;
      vout = s_vout_v;
      vdda = s_vdda_v;
      duty = s_duty_cmd;
    }

    /* 清屏 */
    u8g2_ClearBuffer(&u8g2);

    /* ===== 中间大字：设定电压 =====
       用 0.01V 分辨率，格式类似 "12.34V"
       选一个较大的字体（u8g2 常见大字库），如你固件里没编进该字体，可换成其它 *_tf 字体
    */
    {
      /* 组装 "xx.xxV"（手动格式化，避免 snprintf 浮点） */
      int32_t x = (int32_t)(vset * 100.0f + (vset >= 0 ? 0.5f : -0.5f));
      if (x < 0) x = 0;
      uint32_t ip = (uint32_t)(x / 100);
      uint32_t fp = (uint32_t)(x % 100);

      char big[16];
      char *p = big;

      /* 整数部分：最多 3 位显示（0~999），更大可自行扩展 */
      if (ip >= 100) {
        *p++ = (char)('0' + (ip / 100));
        *p++ = (char)('0' + ((ip / 10) % 10));
        *p++ = (char)('0' + (ip % 10));
      } else if (ip >= 10) {
        *p++ = (char)('0' + (ip / 10));
        *p++ = (char)('0' + (ip % 10));
      } else {
        *p++ = (char)('0' + ip);
      }

      *p++ = '.';
      *p++ = (char)('0' + (fp / 10));
      *p++ = (char)('0' + (fp % 10));
      *p++ = 'V';
      *p++ = '\0';

      /* 大字字体 + 居中绘制 */
      u8g2_SetFont(&u8g2, u8g2_font_logisoso24_tf); /* 大字：约 24px 高 */
      int w = u8g2_GetStrWidth(&u8g2, big);
      int x0 = (128 - w) / 2;
      if (x0 < 0) x0 = 0;

      /* 垂直位置：让大字主体在屏幕中间偏下更好看 */
      int baseline_y = 44; /* 经验值：64 高度屏，大字 24 左右时 42~46 比较合适 */
      u8g2_DrawStr(&u8g2, x0, baseline_y, big);
    }

    /* ===== 小字信息：左上 OUT，右上 VDDA，右下 DUTY ===== */
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);

    /* 左上：OUT:xx.xxV */
    {
      /* 复用你已有的 fmt_v_x100()，它输出如 "OUT: 12.34 V" */
      /* 为了更紧凑，prefix 给 "OUT:"，并把末尾空格+V 改成紧凑版（这里直接手动组更短） */
      int32_t x = (int32_t)(vout * 100.0f + (vout >= 0 ? 0.5f : -0.5f));
      if (x < 0) x = 0;
      uint32_t ip = (uint32_t)(x / 100);
      uint32_t fp = (uint32_t)(x % 100);

      char s[16];
      char *p = s;
      *p++='O';*p++='U';*p++='T';*p++=':';

      if (ip >= 100) {
        *p++ = (char)('0' + (ip / 100));
        *p++ = (char)('0' + ((ip / 10) % 10));
        *p++ = (char)('0' + (ip % 10));
      } else if (ip >= 10) {
        *p++ = (char)('0' + (ip / 10));
        *p++ = (char)('0' + (ip % 10));
      } else {
        *p++ = (char)('0' + ip);
      }
      *p++='.';
      *p++ = (char)('0' + (fp / 10));
      *p++ = (char)('0' + (fp % 10));
      *p++='V';
      *p++='\0';

      u8g2_DrawStr(&u8g2, 0, 12, s);
    }

    /* 右上：VDD: x.xxx */
    {
      int32_t mv = (int32_t)(vdda * 1000.0f + (vdda >= 0 ? 0.5f : -0.5f));
      if (mv < 0) mv = 0;
      uint32_t ip = (uint32_t)(mv / 1000);
      uint32_t fp = (uint32_t)(mv % 1000);

      char s[16];
      char *p = s;
      *p++='V';*p++='D';*p++='D';*p++=':';
      *p++ = (char)('0' + (ip % 10u));
      *p++='.';
      /* 3 位小数补零 */
      p[0] = (char)('0' + (fp / 100));
      p[1] = (char)('0' + ((fp / 10) % 10));
      p[2] = (char)('0' + (fp % 10));
      p += 3;
      *p++='\0';

      int w = u8g2_GetStrWidth(&u8g2, s);
      int x0 = 128 - w;
      if (x0 < 0) x0 = 0;
      u8g2_DrawStr(&u8g2, x0, 12, s);
    }

    /* 右下：DUTY:xxx% */
    {
      int32_t pct = (int32_t)(duty * 100.0f + 0.5f);
      if (pct < 0) pct = 0;
      if (pct > 100) pct = 100;

      char s[16];
      char *p = s;
      *p++='D';*p++=':';

      /* 宽度 3，左侧空格 */
      if (pct < 10) {
        *p++=' ';*p++=' ';
        *p++=(char)('0'+pct);
      } else if (pct < 100) {
        *p++=' ';
        *p++=(char)('0'+(pct/10));
        *p++=(char)('0'+(pct%10));
      } else {
        *p++='1';*p++='0';*p++='0';
      }
      *p++='%';
      *p++='\0';

      int w = u8g2_GetStrWidth(&u8g2, s);
      int x0 = 128 - w;
      if (x0 < 0) x0 = 0;
      u8g2_DrawStr(&u8g2, x0, 63, s); /* 63 接近底部 */
    }

    /* 刷新 */
    u8g2_SendBuffer(&u8g2);

    /* 不加 osDelay：你要求 displayTask 最低优先级，靠调度自然限帧 */
  }
  /* USER CODE END StartDisplayTask */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */
