#include "misc_functions.h"

void remote_function(u8 command) {
    if (settings.poweroff && command != 175)
        return;

    printf("%d\r\n", command);

    switch(command) {
        case 47: // Up arrow
            stations->current = stations->current->next;
            if (!stations->current)
                stations->current = stations->head;
            change_station(stations->current);
            break;
        case 143: // Down arrow
            if (stations->current == stations->head) {
                stations->current = stations->tail;
            } else {
                stations->current = stations->current->prev;
            }
            change_station(stations->current);
            break;
        case 87: // TV / Audio
            settings.mute = rda5807_toggle_mute();
            print_settings();
            break;
        case 199: // Search
            settings.boost = rda5807_toggle_bass_boost();
            print_settings();
            break;
        case 7: // Next song
            if (++settings.volume > 15)
                settings.volume = 15;
            rda5807_set_volume(settings.volume);
            print_settings();
            break;
        case 135: // Previous song
            if (--settings.volume > 15)
                settings.volume = 0;
            rda5807_set_volume(settings.volume);
            print_settings();
            break;
        case 175: // Poweroff
            poweroff();
            break;
        case 127: // 1
        case 191:
        case 63:
        case 223:
        case 95:
        case 159:
        case 31:
        case 239:
        case 111:
        case 255: // 0
            add_digit(command);
            break;
        default:
            break;
    }

}

unsigned int entered_digits = 0;
u8 sub_digits = 0;
bool ready = false;

void invalid_digit() {
    entered_digits = 0;
    sub_digits = 0;
    hd44780_print(" INVALID ");
}

void valid_digit() {
    char data[20];
    hd44780_go_to_line(2);
    sprintf(data, "%d.%d MHz VALID", entered_digits, sub_digits);
    entered_digits = 0;
    sub_digits = 0;
    u8 i = 0;
    for (i = 0; i < 2; ++i) {
        delay_us(TIM3, 1000);
    }
    hd44780_go_to_line(2);
    hd44780_print("            ");
    hd44780_go_to_line(2);
}

void add_digit(u8 command) {
    u8 digit = 0;
    switch (command) {
    case 127: // 1
        digit = 1;
        break;
    case 191:
        digit = 2;
        break;
    case 63:
        digit = 3;
        break;
    case 223:
        digit = 4;
        break;
    case 95:
        digit = 5;
        break;
    case 159:
        digit = 6;
        break;
    case 31:
        digit = 7;
        break;
    case 239:
        digit = 8;
        break;
    case 111:
        digit = 9;
        break;
    case 255: // 0
        digit = 0;
        break;
    }

    hd44780_go_to_line(2);
    u16 temp = entered_digits * 10 + digit;

    if ((digit == 0 || digit == 2 || digit == 3 || digit == 4 || digit == 5 || digit == 6 || digit == 7) && entered_digits == 0) {
        invalid_digit();
        return;
    }
    if (temp > 870 && temp < 1080) {
        sub_digits = digit;
        ready = true;
    } else if (temp <= 870) {
        entered_digits = entered_digits * 10 + digit;
    } else {
        entered_digits = 0;
        sub_digits = 0;
    }
    char data[16];
    sprintf(data, "%u.%u MHz      ", entered_digits, sub_digits);
    hd44780_print(data);

    if (ready) {
        valid_digit();
    }
}

radio* add_radio(char* name, u16 frequency) {
    radio *station = malloc(sizeof(radio));
    if (!station) while(1); // Out of memory

    if (stations->head == 0)
        stations->head = station;

    if (stations->current == 0)
        stations->current = station;

    radio *prev_tail = stations->tail;
    stations->tail = station;


    station->frequency = frequency;
    station->name = strdup(name);
    station->next = 0;
    if (prev_tail) {
        station->prev = prev_tail;
        prev_tail->next = station;
    }

    return station;
}

void print_station_name(radio *station) {
    hd44780_cmd(0x01); // clear display, go to 0x0
    hd44780_print(station->name);

    static char frequency[20];
    static u8 main_, rest;
    main_ = station->frequency / 10;
    rest = station->frequency - main_ * 10;
    sprintf(frequency, "%3d.%d MHz", main_, rest);
    hd44780_go_to_line(1);
    hd44780_print(frequency);

    print_settings();
}

void print_settings() {
    static char volume[2];

    hd44780_go_to(0, 14);
    if (settings.mute) {
        hd44780_print("X");
    } else {
        hd44780_print(" ");
    }

    if (settings.boost) {
        hd44780_print("B");
    } else {
        hd44780_print(" ");
    }

    hd44780_go_to(1, 14);
    sprintf(volume, "%2d", settings.volume);
    hd44780_print(volume);
}

void change_station(radio *station) {
    print_station_name(station);
    rda5807_set_frequency(station->frequency);
}

void poweroff() {
    settings.poweroff = !settings.poweroff;

    if (settings.poweroff) {
        rda5807_set_mute(true);
        hd44780_cmd(0x08);
        hd44780_backlight(false);
    } else {
        rda5807_set_mute(false);
        hd44780_cmd(0x0C);
        hd44780_backlight(true);
//        print_station_name(stations->current);
    }
}
