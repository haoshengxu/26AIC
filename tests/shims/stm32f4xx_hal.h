#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
#define GPIOE ((void *)1)
#define GPIOA ((void *)2)
#define GPIO_PIN_0 1U
#define GPIO_PIN_8 256U
uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state);
static uint32_t irq_mask;
static __inline uint32_t __get_PRIMASK(void) { return irq_mask; }
static __inline void __disable_irq(void) { irq_mask=1; }
static __inline void __set_PRIMASK(uint32_t m) { irq_mask=m; }
#endif
