#include "misc_functions.h"

void remote_function(u8 command) {
    if ((settings.poweroff && command != 175)
            || command == 0)
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

}

radio* add_radio(char* name, u16 frequency) {
    radio *new_station = malloc(sizeof(radio));
    if (!new_station) while(1); // Out of memory

    new_station->frequency = frequency;
    // we have to discard 2 last chars (for Mute / Bass boost indicators)
    new_station->name = strndup(name, 14);

    printf("Adding: %s\n\r", name);

    if (stations->head == 0) {
        new_station->next = 0;
        new_station->prev = 0;
        stations->head = new_station;
        stations->tail = new_station;
        stations->current = new_station;
        printf("Done!\r\n");
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

    printf("Done!\r\n");
    return new_station;
}

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

//    hd44780_go_to(1, 14);
    sprintf(volume, "%2d", settings.volume);
    hd44780_print(volume);
}

void change_station(radio *station) {
    print_station_name(station);
    rda5807_set_frequency(station->frequency);
}

void print_list(radio_list *list) {
    radio *ptr = list->head;
    while (ptr) {
        printf("%x %s %d %x %x\r\n", ptr, ptr->name, ptr->frequency, ptr->prev, ptr->next);
        ptr = ptr->next;
    }
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
    }
}

void setup_display(void) {
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
