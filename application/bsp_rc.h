/**
  * @file    bsp_rc.h
  * @brief   遥控接收底层(DT7/D-BUS): UART5 + DMA 双缓冲 + 串口空闲中断
  */
#ifndef BSP_RC_H
#define BSP_RC_H

#include <stdint.h>

/* 初始化遥控接收。rx1_buf/rx2_buf 是DMA双缓冲的两块内存, dma_buf_num 是每块的字节数。
   初始化完成后就开始接收, 每收完一帧会进 RC_UART5_IRQHandler()(在 remote_control.c 里),
   它由 it.c 的 UART5_IRQHandler() 调用。 */
extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

/* 重新武装DMA接收(遥控掉线/数据出错时调用, 相当于把接收状态复位一次) */
extern void RC_restart(uint16_t dma_buf_num);

/* UART5 空闲中断处理: D-BUS一帧(18字节)收完时由 UART5_IRQHandler 调用, 实现在 remote_control.c */
extern void RC_UART5_IRQHandler(void);

#endif

/*------------------------------ End of File ------------------------------*/