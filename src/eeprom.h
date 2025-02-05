#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "ee.h"
#include <stdint.h>

typedef struct
{
    uint16_t pwm;
    uint8_t  contrast;
} ee_storage_t;

extern ee_storage_t ee_storage;

#endif
