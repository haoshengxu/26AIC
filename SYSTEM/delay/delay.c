#include "delay.h"
#include "stm32f4xx_hal.h"

/* Preserve the HAL 1 ms SysTick. The legacy delay rewrote LOAD/CTRL and
 * stopped SysTick, breaking button debounce, timeouts and HAL_GetTick(). */
void delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(u32 nus)
{
    uint32_t start;
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    /* Small chunks avoid multiplication overflow for long delays. */
    while (nus != 0U) {
        uint32_t chunk = nus > 1000U ? 1000U : nus;
        start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < chunk * cycles_per_us) { }
        nus -= chunk;
    }
}

void delay_ms(u16 nms)
{
    HAL_Delay(nms);
}
