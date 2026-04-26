#pragma once

#include <stdint.h>
#include <stdbool.h>

// Simple MCU-maintained time-of-day, disciplined by GPS PPS via TIM2 ISR.
typedef struct {
    uint8_t hours;               // 0-23
    uint8_t minutes;             // 0-59
    uint8_t seconds;             // 0-59
    int8_t  timezone_offset;     // -14..+14 hours (optional; not actively used yet)
    bool    gps_disciplined;     // true if recently synced to GPS time
    uint32_t seconds_since_gps;  // seconds since last GPS sync
} mcu_time_t;

extern volatile mcu_time_t mcu_time;

// "HH:MM:SS\0" - keep updates atomic via memcpy of 9 bytes
extern volatile char mcu_time_string[9];

// Optional dirty flag if you prefer formatting outside ISR (not used by default)
extern volatile bool mcu_time_dirty;

// Initialize module. Provide initial timezone offset if desired (can pass 0).
void mcu_time_init(int8_t initial_tz_offset);

// Increment by one second (call from TIM2 PeriodElapsed ISR).
void mcu_time_increment(void);

// Sync time from a string:
// - If formatted_hh_colon_mm_colon_ss is true, expects "HH:MM:SS"
// - Otherwise expects "HHMMSS"
void mcu_time_sync_from_string(const char* str, bool formatted_hh_colon_mm_colon_ss);

// Manual setters (optional)
void mcu_time_set(uint8_t h, uint8_t m, uint8_t s);
void mcu_time_set_timezone(int8_t offset);

// Returns a static string literal describing discipline state:
// "GPS-Sync" (if synced within last 10s), "GPS-Disc" (synced, but older), or "Free-Run"
const char* mcu_time_get_status(void);
