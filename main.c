#include <time.h>

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
#include "device_lib/esp8266.h"
#include "device_lib/rda5807.h"
#include "device_lib/hd44780-i2c.h"
#include "device_lib/ir.h"

static time_t _current_raw_time = 0;
static bool _time_set = false;

// Callback function called after receiving packet from WiFi module (+IPD)
void incoming_packet_handler(char* string, uint8_t size) {
    if (_time_set) {
        return;
    }

    LED_toggle(2);

    _time_set = true;

    // add one second, to compensate missing "tick"
    RTC_SetCounter(atoi(string) + 1);
    RTC_WaitForLastTask();
}

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
    IWDG_SetPrescaler(IWDG_Prescaler_128);

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

int main(void)
{
    // Start Independent Watchdog
    iwdg_setup();

    // Configure NVIC
    NVIC_Configuration();

    // Setup LED on board
    LED_Init2();

    IWDG_ReloadCounter();
    // Setup RTC
    rtc_setup();
    IWDG_ReloadCounter();

    // Setup UART

    USART1_Init(115200);
    // Setup I2C
    I2C_LowLevel_Init(I2C1);

    // Setup Timers and IR
    TIM_Init();

    IR_Init();
    setup_delay_timer(TIM4);
    ir_nec_init(GPIO_Pin_10, GPIOB);

    // Setup Wifi module (used for clock sync)
    IWDG_ReloadCounter();
    esp8266_init();
    // Open local UDP port (5505) for time sync
    esp8266_close_connection();
    esp8266_establish_two_way_connection(
        ESP8266_PROTOCOL_UDP, "0.0.0.0", 5555, 5505, 0,
        &incoming_packet_handler);

    // Setup FM radio module;
    rda5807_init(TIM4);
    IWDG_ReloadCounter();

    // Setup HD44780 I2C display
    hd44780_init(TIM4);
    hd44780_print("Radio");
    add_custom_characters();

    // Systicks every 1 second
    if (SysTick_Config(SysTick_LOAD_RELOAD_Msk)) { while (1); }

    // Add predefined radio stations
    populate_stations();

    // Load settings from EEPROM
    load_settings();

    while (1) {
        // Trigger watchdog and enter deep sleep mode to save power
        IWDG_ReloadCounter();
        __WFI();
    }
}

// Prints date and time when device is idle (power off)
void print_idle_time() {
    static char time_buffer[16];
    struct tm* current_time = gmtime(&_current_raw_time);
    hd44780_go_to_line(0);
    strftime(time_buffer, 16, "   %d.%m.%Y", current_time);
    hd44780_print(time_buffer);
    hd44780_go_to_line(1);
    strftime(time_buffer, 16, "     %R", current_time);
    hd44780_print(time_buffer);
}

// Prints time only when device is inactive (power on, but there have been some
// time since last action (determined by `function_timeout` variable
void print_time() {
    static char time_buffer[11];

    hd44780_go_to_line(1);

    struct tm* current_time = gmtime(&_current_raw_time);

    strftime(time_buffer, 11, " %T ", current_time);
    hd44780_print(time_buffer);

}

void SysTick_Handler(void) {
    if (function_timeout >= 20) {
        if (function_timeout == 20) {
            // Turn off display
            hd44780_backlight(false);
        }
    }
    // Saturated add to avoid overflow
    function_timeout = sadd8(function_timeout, 1);
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

// Handle USART interrupt (data from WiFi module)
void USART1_IRQHandler(void)
{
    static uint8_t index = 0;
    static char buffer[80];

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        char c = USART_ReceiveData(USART1);

        buffer[index] = c;
        index = (index + 1) % sizeof(buffer);

        if (c == '\n') {
            // if line has more than 2 chars (i.e. \r\n sequence) strip them
            // and send to esp8266 library
            if (index > 2)
                esp8266_new_line(strndup(buffer, index - 2));


            buffer[index] = 0;
            index = 0;
        }
    }
}

// Handle RTC interrupts (ticks and alarms)
void RTC_IRQHandler(void)
{
    if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        if (_time_set) {
            _current_raw_time = RTC_GetCounter();

            // Display time
            if (settings.poweroff)
                print_idle_time();
            else if (function_timeout > 20)
                print_time();
        }

//        LED_toggle(2);
        // Clear Interrupt Bit
        RTC_ClearITPendingBit(RTC_IT_SEC);
        // Wait for RTC Write Operations
        RTC_WaitForLastTask();
    }
}

