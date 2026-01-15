#ifndef POWER_SWITCH_H
#define POWER_SWITCH_H

#include <stdint.h>

/* ================= MOS ================= */

typedef enum {
    MOS_OFF = 0,   // 高阻
    MOS_ON  = 1    // 下拉导通
} MosState_t;

/* MOS1 ~ MOS3 */
void MOS1_Set(MosState_t state);
void MOS2_Set(MosState_t state);
void MOS3_Set(MosState_t state);

void MOS1_Toggle(void);
void MOS2_Toggle(void);
void MOS3_Toggle(void);

MosState_t MOS1_Get(void);
MosState_t MOS2_Get(void);
MosState_t MOS3_Get(void);


/* ================= 半桥（推挽） ================= */

typedef enum {
    PP_HIZ = 0,
    PP_LOW,
    PP_HIGH
} PushPullState_t;

/* Half-Bridge 1 ~ 3 */
void HB1_Set(PushPullState_t state);
void HB2_Set(PushPullState_t state);
void HB3_Set(PushPullState_t state);

PushPullState_t HB1_Get(void);
PushPullState_t HB2_Get(void);
PushPullState_t HB3_Get(void);

#endif /* POWER_SWITCH_H */
