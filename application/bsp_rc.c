/**
  * @file    bsp_rc.c
  * @brief   遥控接收底层(DT7/D-BUS): UART5 + DMA 双缓冲 + 串口空闲中断
  *
  *          接线/参数和官方 CtrBoard-H7_DBUS 例程一致:
  *            UART5_RX = PD2        (板子上的 D-BUS/SBUS 接收口, 接收机用板上那路可控5V供电)
  *            100000bps / 9位(8数据位+偶校验) / 2停止位
  *
  *          2026-09-19 改: UART5 / DMA / GPIO 的初始化现在全部由 CubeMX 生成, 本文件不再自己配:
  *            usart.c : huart5(100000/9E2) + hdma_uart5_rx(DMA1_Stream0) + PD2=UART5_RX
  *            dma.c   : MX_DMA_Init() 开 DMA1 时钟和中断
  *            it.c    : UART5_IRQHandler() -> RC_UART5_IRQHandler()
  *          这里只补 CubeMX 不生成的两步: 把 RX DMA 配成"双缓冲", 以及打开串口的
  *          DMA 请求(CR3.DMAR)和空闲中断(IDLEIE)。
  *
  *          工作原理:
  *            DMA 在后台把串口收到的字节搬进 rx1_buf / rx2_buf 两块内存(双缓冲),
  *            一帧数据发完、总线安静一个字节时间后串口产生"空闲中断",
  *            RC_UART5_IRQHandler() 就算出这一帧有多少字节并解析。
  */

#include "bsp_rc.h"
#include "main.h"
#include "usart.h"   /* huart5 (CubeMX 生成) */

/* CubeMX 在 usart.c 里定义的 DMA 句柄 */
extern DMA_HandleTypeDef hdma_uart5_rx;

/* 本次配置的DMA缓冲区长度(字节), RC_restart() 重新武装DMA时要用 */
static uint16_t rc_dma_buf_num = 0;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    if ((rx1_buf == NULL) || (rx2_buf == NULL) || (dma_buf_num == 0))
    {
        return;
    }

    rc_dma_buf_num = dma_buf_num;

    /* 双缓冲接收: 一次配好两块内存, 空闲中断里通过 DMA_SxCR_CT 位判断当前用的是哪一块。
       CubeMX 把 hdma_uart5_rx 配成了普通(NORMAL)模式, HAL_DMAEx_MultiBufferStart() 会
       自己把 DBM 位置上, 所以不需要去改 CubeMX 里的 DMA 配置 */
    if (HAL_DMAEx_MultiBufferStart(&hdma_uart5_rx,
                                   (uint32_t)&huart5.Instance->RDR,
                                   (uint32_t)rx1_buf,
                                   (uint32_t)rx2_buf,
                                   dma_buf_num) != HAL_OK)
    {
        Error_Handler();
    }

    /* 只把 DMA 流配好是不够的: 串口这侧的"DMA接收请求"开关 CR3.DMAR 也必须打开,
       否则 DMA 流虽然 EN=1、外设/内存地址都对, 串口却从不发出传输请求, 一个字节都搬不动,
       数据全堆在 RDR 里 -> RXNE/ORE 一直挂着, 遥控永远收不到 */
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);

    /* 打开串口空闲中断: 一帧收完(总线安静一个字节时间)就会进 UART5_IRQHandler */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}

void RC_restart(uint16_t dma_buf_num)
{
    if (dma_buf_num != 0)
    {
        rc_dma_buf_num = dma_buf_num;
    }

    /* 关掉DMA -> 清掉所有标志 -> 回到内存0 -> 重新计数 -> 开DMA */
    __HAL_DMA_DISABLE(&hdma_uart5_rx);

    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR   &= ~(DMA_SxCR_CT);
    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->NDTR  = rc_dma_buf_num;

    /* 清掉 RX DMA 流(本板是 DMA1_Stream0)的全部中断标志 */
    __HAL_DMA_CLEAR_FLAG(&hdma_uart5_rx, (DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4 |
                                          DMA_FLAG_TEIF0_4 | DMA_FLAG_DMEIF0_4 |
                                          DMA_FLAG_FEIF0_4));

    __HAL_DMA_ENABLE(&hdma_uart5_rx);
}

/*------------------------------ End of File ------------------------------*/