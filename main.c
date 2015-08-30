#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#include "common_lib/utils.h"
#include "common_lib/usart.h"

int main(void)
{
    LED_Init2();
    int j;

    USART1_Init(9600);

    usart1_print("Hello World!");

    while (1) {
        for (j = 0; j < 2000000; ++j) {}
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
        for (j = 0; j < 2000000; ++j) {}
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);

    }
}
