#include "misc_functions.h"

extern time_t _current_raw_time;
extern struct tm* _current_time;
extern bool _time_set;


void remote_function(u8 command) {
    // If poweroff, respond only on wakeup button, ignore empty commands
    if ((settings.poweroff && command != 175)
            || command == 0)
        return;

//    printf("%d\r\n", command);

    if (!settings.poweroff && function_timeout > 20) {
        hd44780_backlight(true);
        print_station_name(stations->current);
    }

    function_timeout = 0;

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
            settings.boost = !settings.boost;
            rda5807_set_bass_boost(settings.boost);
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
        case 125: // Exit
            break;
        case 127: // 1
            change_station(shortcuts_list[0].station);
            break;
        case 191:
            change_station(shortcuts_list[1].station);
            break;
        case 63:
            change_station(shortcuts_list[2].station);
            break;
        case 223:
            change_station(shortcuts_list[3].station);
            break;
        case 95:
            change_station(shortcuts_list[4].station);
            break;
        case 159:
            change_station(shortcuts_list[5].station);
            break;
        case 31:
            change_station(shortcuts_list[6].station);
            break;
        case 239:
            change_station(shortcuts_list[7].station);
            break;
        case 111:
            change_station(shortcuts_list[8].station);
            break;
        case 255: // 0
            break;
        default:
            break;
    }

    save_settings();
}

// Add radio station to list of stations. Station inserted according to its
// frequency to keep list sorted
radio* add_radio(char* name, u16 frequency) {
    radio *new_station = malloc(sizeof(radio));
    if (!new_station) while(1); // Out of memory

    new_station->frequency = frequency;
    new_station->name = strndup(name, 16);

//    printf("Adding: %s\n\r", name);

    if (stations->head == 0) {
        new_station->next = 0;
        new_station->prev = 0;
        stations->head = new_station;
        stations->tail = new_station;
        stations->current = new_station;
//        printf("Done!\r\n");
        return new_station;
    }

    // Iterate over list to find correct position
    radio *current_position = stations->head;
    while (current_position) {
        // New item will be placed before current position
        if (current_position->frequency > new_station->frequency) {
            radio *previous = current_position->prev;

            current_position->prev = new_station;
            new_station->next = current_position;

            new_station->prev = previous;
            if (previous) {
                previous->next = new_station;
            } else {
                stations->head = new_station;
            }

            break;
        }

        current_position = current_position->next;
    }

    if (current_position == 0) {
        radio *prev_tail = stations->tail;
        stations->tail = new_station;
        new_station->next = 0;

        if (prev_tail) {
            new_station->prev = prev_tail;
            prev_tail->next = new_station;
        }
    }

//    printf("Done!\r\n");
    return new_station;
}

// Print station name on LCD display
void print_station_name(radio *station) {
    hd44780_cmd(0x01); // clear display, go to 0x0
    hd44780_print(station->name);

    static char frequency[16];
    static u8 main_, rest;
    main_ = station->frequency / 10;
    rest = station->frequency - main_ * 10;
    sprintf(frequency, "%3d.%d0 MHz", main_, rest);
    hd44780_go_to_line(1);
    hd44780_print(frequency);

    print_settings();
}

// Print current settings on LCD display
void print_settings() {
    static char volume[2];

    hd44780_go_to(1, 11);

    hd44780_char(0);
    if (settings.mute) {
        hd44780_print(" ");
    } else {
        hd44780_char(1);
    }

    if (settings.boost) {
        hd44780_char(2);
    } else {
        hd44780_print("B");
    }

    sprintf(volume, "%2d", settings.volume);
    hd44780_print(volume);
}

// Change current active station
void change_station(radio *station) {
    if (!settings.poweroff)
        print_station_name(station);
    rda5807_set_frequency(station->frequency);
}

// Print list of stations (for debugging purposes)
void print_list(radio_list *list) {
    radio *ptr = list->head;
    while (ptr) {
//        printf("%p %s %d %p %p\r\n", ptr, ptr->name, ptr->frequency, ptr->prev, ptr->next);
        ptr = ptr->next;
    }
}

// "Power off" device
void poweroff() {
    settings.poweroff = !settings.poweroff;

    if (settings.poweroff) {
        rda5807_set_mute(true);
        hd44780_cmd(0x01);
//        hd44780_cmd(0x08);
        hd44780_backlight(false);
    } else {
        rda5807_set_mute(false);
//        hd44780_cmd(0x0C);
        print_station_name(stations->current);
        hd44780_backlight(true);
    }
    update_time();
}

// Add custom characters to LCD display memory in order to draw them
void add_custom_characters(void) {
    // Volume font, pt. 1
    u8 volume[] = {
	0b00001,
	0b00011,
	0b01111,
	0b01111,
	0b01111,
	0b00011,
	0b00001,
        0b00000
    };

    // Volume font, pt. 2
    u8 off[8] = {
        0b01000,
        0b10000,
        0b00000,
        0b11000,
        0b00000,
        0b10000,
        0b01000,
        0b00000
    };

    // Boost icon
    u8 boost[8] = {
        0b11110,
        0b11011,
        0b11011,
        0b11110,
        0b11011,
        0b11011,
        0b11110
    };

    hd44780_cgram_write(0, volume);
    hd44780_cgram_write(1, off);
    hd44780_cgram_write(2, boost);
}

// Create list of stations and assign shortcuts to some of them
void populate_stations(void) {
    stations = malloc(sizeof(radio_list));
    stations->current = 0;
    stations->tail = 0;
    stations->head = 0;

    radio *s1 = add_radio("Trojka", 994);
    radio *s2 = add_radio("Jedynka", 894);
    radio *s3 = add_radio("RMF FM", 960);
    radio *s4 = add_radio("Radio Krakow", 1016);
    radio *s5 = add_radio("Radio ZET", 1041);
    radio *s6 = add_radio("Radio Plus Krakow", 1061);
    radio *s7 = add_radio("Antyradio", 1013);
    radio *s8 = add_radio("Radio Wawa", 888);
    radio *s9 = add_radio("Rock Radio", 1038);
    add_radio("Dwojka", 1020);
    add_radio("Czworka", 972);
    add_radio("RMF Classic", 878);
    add_radio("RMF Maxxx", 967);
    add_radio("Radiofonia", 1005);
    add_radio("Radio ZET Chilli", 1010);
    add_radio("Radio ZET Gold", 937);
    add_radio("Eska Krakow", 977);
    add_radio("Radio VOX FM", 1070);
    add_radio("Radio TOK FM", 1029);
    add_radio("Zlote Przeboje Wanda", 925);
    add_radio("Radio Muzo FM", 1049);
    add_radio("Radio Bajka", 952);
    add_radio("Radio Pogoda", 1024);
    add_radio("Radio Maryja", 906);
//    print_list(stations);

    // Shortcuts are available from remote
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
}

// Store settings and current FM station on EEPROM
void save_settings(void) {
    uint8_t p_settings = settings.boost << 7 |
                         settings.mute << 6 |
                         settings.poweroff << 5 |
                         settings.volume;
    uint8_t p_current_station_h = stations->current->frequency >> 8;
    uint8_t p_current_station_l = stations->current->frequency;

    uint8_t tx[3];
    tx[0] = p_settings;
    tx[1] = p_current_station_h;
    tx[2] = p_current_station_l;

    at24c64_write_bytes(0x0000, tx, 3);
}

// Load settings from EEPROM and set previous station
void load_settings(void) {
    uint8_t rx[3];
    at24c64_read_bytes(0x0000, rx, 3);
    settings.boost = rx[0] >> 7 & 0x01;
    settings.mute = rx[0] >> 6 & 0x01;
    settings.poweroff = !(rx[0] >> 5 & 0x01);
    settings.volume = rx[0] & 0x0F;

    uint16_t frequency = rx[1] << 8 | rx[2];

    rda5807_set_bass_boost(settings.boost);
    rda5807_set_mute(settings.mute);
    rda5807_set_volume(settings.volume);

    // If radio was toggled off, shut it down, enable otherwise
    poweroff();

    radio *new_station = stations->head;
    while (new_station) {
        if (new_station->frequency == frequency) {
            break;
        }
        new_station = new_station->next;
    }

    if (new_station) {
        stations->current = new_station;
        change_station(new_station);
    } else {
        change_station(shortcuts_list[0].station);
    }

}

// Prints date and time when device is idle (power off)
void _print_idle_time(void) {
    if (!_time_set)
        return;

    hd44780_cmd(0x01);
    static char time_buffer[16];
    hd44780_go_to_line(0);
    strftime(time_buffer, 16, "   %d.%m.%Y", _current_time);
    hd44780_print(time_buffer);
    hd44780_go_to_line(1);
    strftime(time_buffer, 16, "     %R", _current_time);
    hd44780_print(time_buffer);
}

// Prints time only when device is inactive (power on, but there have been some
// time since last action (determined by `function_timeout` variable
void _print_time(void) {
    if (!_time_set)
        return;

    static char time_buffer[11];

    hd44780_go_to_line(1);

    strftime(time_buffer, 11, "   %R   ", _current_time);
    hd44780_print(time_buffer);

}

void update_time(void) {
    if (!_time_set)
        return;

    if (settings.poweroff)
        _print_idle_time();
    else if (function_timeout >= 20)
        _print_time();
}

void force_update_time(void) {
    _current_raw_time = RTC_GetCounter();
    _current_time = gmtime(&_current_raw_time);
    update_time();
}
