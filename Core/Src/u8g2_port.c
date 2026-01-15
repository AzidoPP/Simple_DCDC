#include "u8g2.h"
#include "i2c.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;
extern osSemaphoreId_t i2cTxSemHandle;

// 定义一个静态缓冲区，用于 DMA 传输
// 128x64 屏幕一行的最大数据量 + 1字节控制码，132 足够
static uint8_t u8g2_i2c_buffer[132];
static uint8_t u8g2_buffer_ptr = 0;

uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_SEND: {
            // copy data to DMA buffer
            uint8_t *data = (uint8_t *)arg_ptr;
            while (arg_int > 0) {
                u8g2_i2c_buffer[u8g2_buffer_ptr++] = *data++;
                arg_int--;
            }
            break;
        }
        case U8X8_MSG_BYTE_INIT:
            // I2C already inited outside
            break;
        case U8X8_MSG_BYTE_SET_DC:
            // I2C mode, this is unused
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            u8g2_buffer_ptr = 0;
            break;
        case U8X8_MSG_BYTE_END_TRANSFER: {
            // Use HAL DMA to send
            if (HAL_I2C_Master_Transmit_DMA(
                    &hi2c1,
                    u8x8_GetI2CAddress(u8x8),
                    u8g2_i2c_buffer,
                    u8g2_buffer_ptr) != HAL_OK)
            {
                return 0;
            }
            // wait for DMA finish
            osSemaphoreAcquire(i2cTxSemHandle, osWaitForever);

            break;
        }
        default:
            return 0;
    }
    return 1;
}

uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_DELAY_MILLI:
            osDelay(arg_int);
            break;
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;
        default:
            return 0;
    }
    return 1;
}

