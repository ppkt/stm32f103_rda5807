#ifndef __MISC_FUNCTIONS_H__
#define __MISC_FUNCTIONS_H__
#include "stm32f10x.h"

#include "string.h"
#include "stdlib.h"

#include "device_lib/rda5807.h"
#include "device_lib/hd44780-i2c.h"

typedef struct radio {
    char *name;
    unsigned int frequency;
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
    radio *head;
    radio *tail;
    radio *current;
} radio_list;

radio_list *stations;

radio* add_radio(char* name, u16 frequency);
void add_digit(u8 command);

void print_station_name(radio *station);
void print_settings();
void change_station(radio *station);
void poweroff();

void remote_function(u8 command);


#endif // __MISC_FUNCTIONS_H__
