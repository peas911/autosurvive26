 #ifndef BSP_RC_H
#define BSP_RC_H

#include "stm32f4xx_hal.h"

// SBUS Rx buffer size (DBUS is 18 bytes, we leave some margin)
#define SBUS_RX_BUF_NUM 36
#define RC_FRAME_LENGTH 18

typedef struct
{
    struct
    {
        int16_t ch[5]; // rx, ry, lx, ly, wheel
        char s[2];     // switches
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct
    {
        uint16_t v;
    } key;
} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;
extern volatile uint32_t rc_last_update_ms;
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

#endif