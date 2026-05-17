#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "wit.h"
#include "encoder.h"

uint8_t enable_group1_irq = 0;

void Interrupt_Init(void)
{
    if(enable_group1_irq)
    {
        NVIC_EnableIRQ(1);
    }
}

void SysTick_Handler(void)
{
    tick_ms++;
}

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while(rxSize >= 11)
    {
        checkSum=0;
        for(int i=packCnt*11; i<(packCnt+1)*11-1; i++)
            checkSum += wit_dmaBuffer[i];

        if((wit_dmaBuffer[packCnt*11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt*11+10]))
        {
            if(wit_dmaBuffer[packCnt*11+1] == 0x51)
            {
                wit_data.ax = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 2.048;
                wit_data.ay = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 2.048;
                wit_data.az = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 2.048;
                wit_data.temperature =  (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]) / 100.0;
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x52)
            {
                wit_data.gx = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 16.384;
                wit_data.gy = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 16.384;
                wit_data.gz = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 16.384;
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x53)
            {
                wit_data.roll  = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 32768.0 * 180.0;
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 32768.0 * 180.0;
                wit_data.yaw   = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 32768.0 * 180.0;
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }

    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif

/*
 * TIMG6_IRQHandler — 每 50ms 触发一次，采样编码器脉冲速度
 */
void TIMG6_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(TIMER_Encoder_Read_INST,
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT);

    uint32_t key = __get_PRIMASK();
    __disable_irq();

    encoder_left_speed  = (int16_t)encoder_left_pulse;
    encoder_right_speed = (int16_t)encoder_right_pulse;
    encoder_left_pulse  = 0;
    encoder_right_pulse = 0;

    __set_PRIMASK(key);
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_MULTIPLE_GPIOA_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOA))
            {
                case GPIO_EncoderA_PINA_0_IIDX: /* 左编码器 ChA (PA21) 上升沿 */
                    if (DL_GPIO_readPins(GPIO_EncoderA_PORT, GPIO_EncoderA_PINA_1_PIN) == GPIO_EncoderA_PINA_1_PIN)
                        encoder_left_pulse--;    /* ChB 为 HIGH → 反转 */
                    else
                        encoder_left_pulse++;    /* ChB 为 LOW → 正转 */
                    break;

                case GPIO_EncoderA_PINA_1_IIDX: /* 左编码器 ChB (PA22) 上升沿 */
                    if (DL_GPIO_readPins(GPIO_EncoderA_PORT, GPIO_EncoderA_PINA_0_PIN) == GPIO_EncoderA_PINA_0_PIN)
                        encoder_left_pulse++;    /* ChA 为 HIGH → 正转 */
                    else
                        encoder_left_pulse--;    /* ChA 为 LOW → 反转 */
                    break;

                case GPIO_EncoderB_PINB_0_IIDX: /* 右编码器 ChA (PA24) 上升沿 */
                    if (DL_GPIO_readPins(GPIO_EncoderB_PORT, GPIO_EncoderB_PINB_1_PIN) == GPIO_EncoderB_PINB_1_PIN)
                        encoder_right_pulse--;   /* ChB 为 HIGH → 反转 */
                    else
                        encoder_right_pulse++;   /* ChB 为 LOW → 正转 */
                    break;

                case GPIO_EncoderB_PINB_1_IIDX: /* 右编码器 ChB (PA23) 上升沿 */
                    if (DL_GPIO_readPins(GPIO_EncoderB_PORT, GPIO_EncoderB_PINB_0_PIN) == GPIO_EncoderB_PINB_0_PIN)
                        encoder_right_pulse++;   /* ChA 为 HIGH → 正转 */
                    else
                        encoder_right_pulse--;   /* ChA 为 LOW → 反转 */
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}
