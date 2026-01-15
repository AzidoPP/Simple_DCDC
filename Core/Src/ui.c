#include "ui.h"
#include "u8g2.h"
#include "shared_data.h"
#include <string.h>

static u8g2_t u8g2;

void UI_Init(void)
{
    extern uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t*, uint8_t, uint8_t, void*);
    extern uint8_t u8x8_gpio_and_delay_stm32(u8x8_t*, uint8_t, uint8_t, void*);
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_stm32_hw_i2c, u8x8_gpio_and_delay_stm32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
}

static void fmt_f01(char *buf, float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 99.9f) v = 99.9f;
    int iv = (int)(v*10 + 0.5f);
    int ip = iv / 10;
    int fp = iv % 10;
    buf[0] = (ip>=10)?'0'+ip/10:' ';
    buf[1] = '0'+(ip%10);
    buf[2] = '.';
    buf[3] = '0'+fp;
    buf[4] = '\0';
}

void UI_Refresh(void)
{
    char s_buf[6], f_buf[6], line[25];
    float vs1, vs2, vf1, vf2;
    if (osMutexAcquire(ADJSetMutexHandle, 10) == osOK) {
        vs1 = g_adj_target.adj1_set_v;
        vs2 = g_adj_target.adj2_set_v;
        osMutexRelease(ADJSetMutexHandle);
    } else { vs1 = g_adj_target.adj1_set_v; vs2 = g_adj_target.adj2_set_v; }
    vf1 = g_adc_status.adj1_fb;
    vf2 = g_adc_status.adj2_fb;

    u8g2_ClearBuffer(&u8g2);
    uint8_t page = g_display_page % DISPLAY_NUM_PAGES;

    u8g2_DrawRFrame(&u8g2,0,0,128,64,3);
    u8g2_DrawHLine(&u8g2,0,15,128);

    if(page==0){
        u8g2_SetFont(&u8g2,u8g2_font_6x12_tf);
        u8g2_DrawStr(&u8g2,10,12,"MONITOR: ADJ1");
        fmt_f01(s_buf,vs1);
        fmt_f01(f_buf,vf1);
        u8g2_SetFont(&u8g2,u8g2_font_9x15_tf);
        strcpy(line,"Set : "); strcat(line,s_buf); strcat(line," V"); u8g2_DrawStr(&u8g2,12,32,line);
        strcpy(line,"Feed: "); strcat(line,f_buf); strcat(line," V"); u8g2_DrawStr(&u8g2,12,48,line);
        u8g2_DrawFrame(&u8g2,12,52,104,5);
        u8g2_DrawBox(&u8g2,12,52,(uint8_t)(vs1*4.3f),5);
    }else if(page==1){
        u8g2_SetFont(&u8g2,u8g2_font_6x12_tf);
        u8g2_DrawStr(&u8g2,10,12,"MONITOR: ADJ2");
        fmt_f01(s_buf,vs2);
        fmt_f01(f_buf,vf2);
        u8g2_SetFont(&u8g2,u8g2_font_9x15_tf);
        strcpy(line,"Set : "); strcat(line,s_buf); strcat(line," V"); u8g2_DrawStr(&u8g2,12,32,line);
        strcpy(line,"Feed: "); strcat(line,f_buf); strcat(line," V"); u8g2_DrawStr(&u8g2,12,48,line);
        u8g2_DrawFrame(&u8g2,12,52,104,5);
        u8g2_DrawBox(&u8g2,12,52,(uint8_t)(vs2*4.3f),5);
    }

    strcpy(line,"PAGE "); line[5]='1'+page; line[6]='\0';
    u8g2_SetFont(&u8g2,u8g2_font_6x12_tf); u8g2_DrawStr(&u8g2,85,12,line);

    u8g2_SendBuffer(&u8g2);
}
