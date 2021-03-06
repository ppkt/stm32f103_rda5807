#ifndef __MISC_FUNCTIONS_H__
#define __MISC_FUNCTIONS_H__
#include "stm32f10x.h"

#include "string.h"
#include "stdlib.h"
#include "time.h"

#include "device_lib/at24c64.h"
#include "device_lib/rda5807.h"
#include "device_lib/hd44780-i2c.h"

typedef struct radio {
    char *name;
    uint16_t frequency;
    struct radio *next;
    struct radio *prev;
} radio;

struct {
    bool boost;
    bool mute;
    bool poweroff;
    u8 volume;
} settings;

typedef struct {
    int hotkey;
    radio *station;
} shortcut;

shortcut shortcuts_list[9];

u8 function_timeout;

typedef struct {
    radio *head;
    radio *tail;
    radio *current;
} radio_list;

radio_list *stations;

radio* add_radio(char* name, u16 frequency);
void add_digit(u8 command);

void populate_stations(void);
void add_custom_characters(void);
void print_station_name(radio *station);
void print_settings();
void print_list(radio_list *list);
void change_station(radio *station);
void poweroff();

void remote_function(u8 command);

void save_settings(void);
void load_settings(void);

//void print_time(void);
//void print_idle_time(void);
void update_time(void);
void force_update_time(void);


#endif // __MISC_FUNCTIONS_H__
