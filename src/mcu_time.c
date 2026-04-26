#include "mcu_time.h"
#include <string.h>

volatile mcu_time_t mcu_time;
volatile char mcu_time_string[9];
volatile bool mcu_time_dirty = false;

static void fmt_time(char* buf, uint8_t h, uint8_t m, uint8_t s) {
    buf[0] = '0' + (h / 10); buf[1] = '0' + (h % 10);
    buf[2] = ':';
    buf[3] = '0' + (m / 10); buf[4] = '0' + (m % 10);
    buf[5] = ':';
    buf[6] = '0' + (s / 10); buf[7] = '0' + (s % 10);
    buf[8] = '\0';
}

void mcu_time_init(int8_t initial_tz_offset) {
    mcu_time.hours = 0;
    mcu_time.minutes = 0;
    mcu_time.seconds = 0;
    mcu_time.timezone_offset = initial_tz_offset;
    mcu_time.gps_disciplined = false;
    mcu_time.seconds_since_gps = 0;

    char tmp[9];
    fmt_time(tmp, 0, 0, 0);
    memcpy((void*)mcu_time_string, tmp, sizeof(tmp));
    mcu_time_dirty = false;
}

void mcu_time_increment(void) {
    uint8_t h = mcu_time.hours;
    uint8_t m = mcu_time.minutes;
    uint8_t s = mcu_time.seconds;

    // increment one second
    s++;
    if (s >= 60) {
        s = 0;
        m++;
        if (m >= 60) {
            m = 0;
            h = (h + 1) % 24;
        }
    }

    mcu_time.hours = h;
    mcu_time.minutes = m;
    mcu_time.seconds = s;

    // track discipline age
    if (mcu_time.seconds_since_gps < 0xFFFFFFFFu) {
        mcu_time.seconds_since_gps++;
    }

    // minimally update formatted string (ISR-friendly)
    char tmp[9];
    fmt_time(tmp, h, m, s);
    memcpy((void*)mcu_time_string, tmp, sizeof(tmp));
}

void mcu_time_sync_from_string(const char* str, bool formatted_hh_colon_mm_colon_ss) {
    uint8_t h, m, s;
    if (formatted_hh_colon_mm_colon_ss) {
        // "HH:MM:SS"
        h = (uint8_t)((str[0] - '0') * 10 + (str[1] - '0'));
        m = (uint8_t)((str[3] - '0') * 10 + (str[4] - '0'));
        s = (uint8_t)((str[6] - '0') * 10 + (str[7] - '0'));
    } else {
        // "HHMMSS"
        h = (uint8_t)((str[0] - '0') * 10 + (str[1] - '0'));
        m = (uint8_t)((str[2] - '0') * 10 + (str[3] - '0'));
        s = (uint8_t)((str[4] - '0') * 10 + (str[5] - '0'));
    }

    if (h > 23) h = 0;
    if (m > 59) m = 0;
    if (s > 59) s = 0;

    mcu_time.hours = h;
    mcu_time.minutes = m;
    mcu_time.seconds = s;

    mcu_time.gps_disciplined = true;
    mcu_time.seconds_since_gps = 0;

    char tmp[9];
    fmt_time(tmp, h, m, s);
    memcpy((void*)mcu_time_string, tmp, sizeof(tmp));
}

void mcu_time_set(uint8_t h, uint8_t m, uint8_t s) {
    h %= 24; m %= 60; s %= 60;
    mcu_time.hours = h;
    mcu_time.minutes = m;
    mcu_time.seconds = s;

    char tmp[9];
    fmt_time(tmp, h, m, s);
    memcpy((void*)mcu_time_string, tmp, sizeof(tmp));
}

void mcu_time_set_timezone(int8_t offset) {
    mcu_time.timezone_offset = offset;
}

const char* mcu_time_get_status(void) {
    if (mcu_time.gps_disciplined) {
        if (mcu_time.seconds_since_gps <= 10) {
            return "GPS-Sync";
        }
        return "GPS-Disc";
    }
    return "Free-Run";
}
