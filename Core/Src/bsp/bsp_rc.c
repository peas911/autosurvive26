#include "bsp/bsp_rc.h"
#include "usart.h" // We assume usart3 or usart1 is used. Modify accordingly.

extern UART_HandleTypeDef huart3; // Usually UART3 is used for RC DBUS
RC_ctrl_t rc_ctrl;

void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL) { return; }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;    //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff; //!< Channel 2
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //!< Wheel

    rc_ctrl->rc.ch[0] -= 1024;
    rc_ctrl->rc.ch[1] -= 1024;
    rc_ctrl->rc.ch[2] -= 1024;
    rc_ctrl->rc.ch[3] -= 1024;
    rc_ctrl->rc.ch[4] -= 1024;

    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003); //!< Switch right
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C)         >> 2; //!< Switch left
}

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // enable the DMA transfer for the receiver request
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    // enable idle interrupt
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    // disable DMA
    __HAL_DMA_DISABLE(huart3.hdmarx);
    while(huart3.hdmarx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(huart3.hdmarx);
    }

    __HAL_DMA_CLEAR_FLAG(huart3.hdmarx, DMA_LISR_TCIF1);
    __HAL_DMA_CLEAR_FLAG(huart3.hdmarx, DMA_LISR_HTIF1);

    huart3.hdmarx->Instance->PAR = (uint32_t)&huart3.Instance->DR;
    huart3.hdmarx->Instance->M0AR = (uint32_t)rx1_buf;
    huart3.hdmarx->Instance->M1AR = (uint32_t)rx2_buf;
    huart3.hdmarx->Instance->NDTR = dma_buf_num;
    
    SET_BIT(huart3.hdmarx->Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(huart3.hdmarx);
}