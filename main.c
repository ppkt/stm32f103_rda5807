#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_iwdg.h>
#include <stm32f10x_pwr.h>

#include "misc_functions.h"

#include "common_lib/utils.h"
#include "common_lib/usart.h"
#include "common_lib/i2c_dma.h"

#include "device_lib/at24c64.h"
#include "device_lib/rda5807.h"
#include "device_lib/hd44780-i2c.h"
#include "device_lib/ir.h"

void NVIC_Configuration(void)
{

    /* 1 bit for pre-emption priority, 3 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_SetPriority(I2C1_EV_IRQn, 0x00);
    NVIC_EnableIRQ(I2C1_EV_IRQn);

    NVIC_SetPriority(I2C1_ER_IRQn, 0x01);
    NVIC_EnableIRQ(I2C1_ER_IRQn);


    NVIC_SetPriority(I2C2_EV_IRQn, 0x00);
    NVIC_EnableIRQ(I2C2_EV_IRQn);

    NVIC_SetPriority(I2C2_ER_IRQn, 0x01);
    NVIC_EnableIRQ(I2C2_ER_IRQn);

}

void iwdg_setup() {
    // Enable write access to IWDG_PR and IWDG_RLR registers
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    // IWDG counter clock: LSI / 256 -> max timeout = 26 s
    IWDG_SetPrescaler(IWDG_Prescaler_256);

    // Set counter reload value to obtain 3000 ms IWDG timeout
    // 40000 -> 40kHz LSI oscillator, but it might varies between 30 and 60 kHz
    IWDG_SetReload(40000 / 85);

    // Reload IWDG counter
    IWDG_ReloadCounter();

    // Enable IWDG
    IWDG_Enable();
}

void IR_Init() {
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Use PB6 as input from IR receiver
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Enable clock and its interrupts
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // interrupt for decoding command
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM_Init() {
    // Enable clock and its interrupts
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
    TIM_InitStructure.TIM_Period = 10000 - 1; // Update event every 10000 us / 10 ms
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void _low_power(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE, DISABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
}

int main(void)
{
    iwdg_setup();

    _low_power();

    NVIC_Configuration();

    LED_Init2();

    USART1_Init(9600);
    I2C_LowLevel_Init(I2C1);
    TIM_Init();
    IR_Init();

    setup_delay_timer(TIM4);

    ir_nec_init(GPIO_Pin_10, GPIOB);

    usart1_print("Hello World!\n\r");

    printf("Checking Radio\n\r");
    rda5807_init(TIM4);
    IWDG_ReloadCounter();

    hd44780_init(TIM3);
    hd44780_print("Radio");

    settings.volume = 15;
    settings.boost = false;
    settings.mute = false;
    settings.poweroff = false;

    populate_stations();
    setup_display();

    IWDG_ReloadCounter();

    load_settings();
//    rda5807_set_mute(true);

    while (1) {
//        delay_ms(TIM4, 100);
//        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
//        delay_ms(TIM4, 900);
//        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);

        IWDG_ReloadCounter();
        __WFI();
    }
}

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        // Timeout
        ir_nec_reset_transmission();
    }
}

// Handle software interrupt (after decoding command from IR, execute function)
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        u8 data = ir_nec_get_last_command();
        remote_function(data);
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

// Handle data from IR sensor
void EXTI15_10_IRQHandler(void)
{
    static unsigned int counter;

    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
        // Restart Timer
        counter = TIM_GetCounter(TIM2);
        TIM_SetCounter(TIM2, 0);
        ir_nec_state_machine(counter);

        EXTI_ClearITPendingBit(EXTI_Line10);
    }
}
