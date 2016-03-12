#include <stm32f10x_it.h>

extern time_t _current_raw_time;
extern struct tm* _current_time;
extern bool _time_set;
extern void incoming_packet_handler(char* string, uint8_t size);

void SysTick_Handler(void) {
    if (function_timeout == 20 && !settings.poweroff) {
        // Turn off display
        hd44780_backlight(false);
        // Print current time
        update_time();
    }
    // Saturated add to avoid overflow
    function_timeout = sadd8(function_timeout, 1);
}

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        // Timeout or IR communication
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
    static uint8_t ticks_without_clock = 0;

    // Tick every second
    if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        if (_time_set) {
            ticks_without_clock = 0;
            _current_raw_time = RTC_GetCounter();
            _current_time = gmtime(&_current_raw_time);

            if (_current_time->tm_sec == 0) {
                // Update time every minute
                update_time();
            }
            if (_current_time->tm_hour == 0) {
                // Schedule time sync every hour
                _time_set = 0;
            }
        } else {
            // Clock not configured
            ticks_without_clock++;
        }

        LED_toggle(2);
        // Clear Interrupt Bit
        RTC_ClearITPendingBit(RTC_IT_SEC);
        // Wait for RTC Write Operations
        RTC_WaitForLastTask();
    }
}

