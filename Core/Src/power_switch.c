#include "power_switch.h"
#include "main.h"
#include "stm32f1xx_hal.h"

/* ================= 内部状态寄存器定义 ================= */

/* MOS 内部寄存器 */
typedef struct {
    MosState_t mos1;
    MosState_t mos2;
    MosState_t mos3;
} MosReg_t;

/* 半桥内部寄存器 */
typedef struct {
    PushPullState_t hb1;
    PushPullState_t hb2;
    PushPullState_t hb3;
} PushPullReg_t;

/* ================= 实例 ================= */

static MosReg_t mos_reg = {
    .mos1 = MOS_OFF,
    .mos2 = MOS_OFF,
    .mos3 = MOS_OFF,
};

static PushPullReg_t pp_reg = {
    .hb1 = PP_HIZ,
    .hb2 = PP_HIZ,
    .hb3 = PP_HIZ,
};

static void MOS_Write(GPIO_TypeDef *port, uint16_t pin, MosState_t state)
{
    if (state == MOS_ON) {
        // MOSFET gate pull up, MOSFET on
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    } else {
        // MOSFET gate pull down, MOSFET close
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    }
}
void MOS1_Set(MosState_t state)
{
    mos_reg.mos1 = state;
    MOS_Write(GPIOB, GPIO_PIN_10, state);
}

void MOS2_Set(MosState_t state)
{
    mos_reg.mos2 = state;
    MOS_Write(GPIOB, GPIO_PIN_11, state);
}

void MOS3_Set(MosState_t state)
{
    mos_reg.mos3 = state;
    MOS_Write(GPIOB, GPIO_PIN_12, state);
}

void MOS1_Toggle(void) { MOS1_Set(!mos_reg.mos1); }
void MOS2_Toggle(void) { MOS2_Set(!mos_reg.mos2); }
void MOS3_Toggle(void) { MOS3_Set(!mos_reg.mos3); }

MosState_t MOS1_Get(void) { return mos_reg.mos1; }
MosState_t MOS2_Get(void) { return mos_reg.mos2; }
MosState_t MOS3_Get(void) { return mos_reg.mos3; }


static void HB_Write(GPIO_TypeDef *ho_port, uint16_t ho_pin,
                     GPIO_TypeDef *en_port, uint16_t en_pin,
                     PushPullState_t state)
{
    switch (state) {
    case PP_HIZ:
        HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET);
        break;

    case PP_LOW:
        HAL_GPIO_WritePin(ho_port, ho_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET);
        break;

    case PP_HIGH:
        HAL_GPIO_WritePin(ho_port, ho_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET);
        break;
    }
}

void HB1_Set(PushPullState_t state)
{
    pp_reg.hb1 = state;
    HB_Write(GPIOA, GPIO_PIN_8, GPIOB, GPIO_PIN_13, state);
}

PushPullState_t HB1_Get(void)
{
    return pp_reg.hb1;
}

void HB2_Set(PushPullState_t state)
{
    pp_reg.hb2 = state;
    HB_Write(GPIOA, GPIO_PIN_9, GPIOB, GPIO_PIN_14, state);
}

PushPullState_t HB2_Get(void)
{
    return pp_reg.hb2;
}

void HB3_Set(PushPullState_t state)
{
    pp_reg.hb3 = state;
    HB_Write(GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_15, state);
}

PushPullState_t HB3_Get(void)
{
    return pp_reg.hb3;
}
