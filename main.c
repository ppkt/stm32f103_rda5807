#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_exti.h>

#include "misc_functions.h"

#include "common_lib/utils.h"
#include "common_lib/usart.h"
#include "common_lib/i2c_dma.h"

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

void IR_Init() {
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Use PB6 as input from IR receiver
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
    TIM_InitStructure.TIM_Prescaler = 72 - 1;
    TIM_InitStructure.TIM_Period = 10000 - 1; // Update event every 10000 us / 10 ms
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}


int main(void)
{
    NVIC_Configuration();

    LED_Init2();

    USART1_Init(9600);
    I2C_LowLevel_Init(I2C1);
    TIM_Init();
    IR_Init();

    ir_nec_init(GPIO_Pin_10, GPIOB);

    usart1_print("Hello World!\n\r");

    printf("Checking Radio\n\r");
    setup_delay_timer(TIM4);
    rda5807_init(TIM4);

    u8 tx[10];
    tx[0] = 0x00;
    tx[1] = 0x00;
    tx[2] = 0xAA;
//    I2C_Master_BufferWrite(I2C1, tx, 3, Polling, 0x50 << 1);


//    I2C_Master_BufferWrite(I2C1, tx, 2, DMA, 0x50 << 1);
    I2C_Master_BufferRead(I2C1, tx, 1, Polling, 0x50 << 1);

    hd44780_init(TIM3);
    hd44780_print("Radio");

    stations = malloc(sizeof(radio_list));
    stations->current = 0;
    stations->tail = 0;
    stations->head = 0;

    radio *s1 = add_radio("Trojka", 994);
    add_radio("Dwojka", 1020);
    radio *s2 = add_radio("Jedynka", 894);
    add_radio("Czworka", 972);
    radio *s3 = add_radio("RMF FM", 960);
    radio *s4 = add_radio("Radio Krakow", 1016);
    radio *s5 = add_radio("Radio ZET", 1041);
    radio *s6 = add_radio("Radio PLUS", 1061);
    radio *s7 = add_radio("Antyradio", 1013);
    radio *s8 = add_radio("Radio Wawa", 888);
    radio *s9 = add_radio("Rock radio", 1038);
    add_radio("RMF Classic", 878);
    add_radio("RMF Maxxx", 967);
    add_radio("Radiofonia", 1005);
    add_radio("Radio ZET Chilli", 1010);
    add_radio("Radio ZET Gold", 937);
    add_radio("Eska Krakow", 977);
    add_radio("Radio WAWA", 888);
    add_radio("Radio VOX FM", 107);
    add_radio("Radio Plus Krakow", 994);
    print_list(stations);

    shortcuts_list[0].hotkey = 1;
    shortcuts_list[0].station = s1;
    shortcuts_list[1].hotkey = 2;
    shortcuts_list[1].station = s2;
    shortcuts_list[2].hotkey = 3;
    shortcuts_list[2].station = s3;
    shortcuts_list[3].hotkey = 4;
    shortcuts_list[3].station = s4;
    shortcuts_list[4].hotkey = 5;
    shortcuts_list[4].station = s5;
    shortcuts_list[5].hotkey = 6;
    shortcuts_list[5].station = s6;
    shortcuts_list[6].hotkey = 7;
    shortcuts_list[6].station = s7;
    shortcuts_list[7].hotkey = 8;
    shortcuts_list[7].station = s8;
    shortcuts_list[8].hotkey = 9;
    shortcuts_list[8].station = s9;


    settings.volume = 15;
    settings.boost = false;
    settings.mute = false;
    settings.poweroff = false;

    setup_display();
    change_station(s1);
    rda5807_set_mute(true);

    while (1) {
        delay_ms(TIM4, 100);
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
        delay_ms(TIM4, 900);
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
//        __WFI();
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
