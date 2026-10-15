#include "usartx.h"
#include "gyro6.h"
#include "vofa_debug.h"
#include "vision_command.h"

u8 Usart2_Receive_buf[1];
u8 Usart2_Receive;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint8_t byte = Usart1_Receive_buf[0];
        if (HAL_UART_Receive_IT(&huart1, Usart1_Receive_buf, 1U) != HAL_OK) Vision_UartError();
        Vision_ProcessByte(byte);
        return;
    }
    if (huart->Instance == USART3) { Vofa_RxComplete(); return; }
    if (huart->Instance == USART2)
    {
        /* USART2 is dedicated to the six-axis gyro. */
        Gyro6_ProcessByte(Usart2_Receive_buf[0]);
        HAL_UART_Receive_IT(&huart2, Usart2_Receive_buf, sizeof(Usart2_Receive_buf));
    }
}

void usart2_send(u8 data)
{
    USART2->DR = data;
    while ((USART2->SR & 0x40) == 0)
    {
    }
}


/* HAL aborts interrupt reception on overrun; restore it after an error. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Vision_UartError();
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)HAL_UART_Receive_IT(&huart1, Usart1_Receive_buf, 1U);
        return;
    }
    if (huart->Instance == USART3) { Vofa_RxError(); return; }
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart2, Usart2_Receive_buf, sizeof(Usart2_Receive_buf));
    }
}
