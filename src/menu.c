#include "frequency.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "LCD.h"
#include "eeprom.h"
#include "gps.h"
#include "stm32f1xx_hal_gpio.h"
#include "int.h"
#include "menu.h"
#include "mcu_time.h"

/// All times in ms
#define DEBOUNCE_TIME           50
#define BOOT_MENU_SAVE_TIME     3*1000

// Firmware version tag
#define FIRMWARE_VERSION        "v0.1.16"

volatile uint32_t rotary_down_time      = 0;
volatile uint32_t rotary_up_time        = 0;
volatile bool     rotary_press_detected = 0;



bool rotary_is_down() { return rotary_down_time > rotary_up_time && (HAL_GetTick() - rotary_down_time) > DEBOUNCE_TIME; }

bool rotary_get_click()
{
    bool is_down = rotary_is_down();

    if (is_down && !rotary_press_detected) {
        rotary_press_detected = true;
        return true;
    } else if (!is_down) {
        rotary_press_detected = false;
    }

    return false;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ROTARY_PRESS_Pin) {
        if (HAL_GPIO_ReadPin(ROTARY_PRESS_GPIO_Port, ROTARY_PRESS_Pin) == GPIO_PIN_SET) {
            rotary_up_time = HAL_GetTick();
        } else {
            rotary_down_time = HAL_GetTick();
        }
    }
}

uint8_t sat_icons[][8] =        {   { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b10000, 0b00000, 0b00000 },
                                    { 0b00000, 0b00000, 0b00000, 0b11000, 0b00100, 0b10100, 0b00000, 0b00000 },
                                    { 0b00000, 0b11100, 0b00010, 0b11001, 0b00101, 0b10101, 0b00000, 0b00000 },
                                    { 0b00000, 0b11101, 0b00010, 0b11001, 0b01101, 0b10101, 0b00000, 0b00000 },
                                    { 0b00000, 0b11100, 0b00010, 0b11001, 0b00101, 0b10101, 0b00000, 0b00000 },
                                };

uint8_t sat_icons_lock[][8] =   {   { 0b00010, 0b00111, 0b00111, 0b00000, 0b00000, 0b10000, 0b00000, 0b00000 },
                                    { 0b00010, 0b00111, 0b00111, 0b11000, 0b00100, 0b10100, 0b00000, 0b00000 },
                                    { 0b00010, 0b11111, 0b00111, 0b11001, 0b00101, 0b10101, 0b00000, 0b00000 },
                                    { 0b00000, 0b11101, 0b00010, 0b11001, 0b01101, 0b10101, 0b00000, 0b00000 },
                                    { 0b00000, 0b11100, 0b00010, 0b11001, 0b00101, 0b10101, 0b00000, 0b00000 },
                                };

void lcd_create_chars()
{
    for (int i = 0; i < 5; i++) {
        LCD_CreateChar(i+1, ppb_lock_status ? sat_icons_lock[i] : sat_icons[i]);
    }
}

typedef enum { SCREEN_MAIN, SCREEN_DATE, SCREEN_DATE_TIME, SCREEN_TREND, SCREEN_PPB, SCREEN_PWM, SCREEN_GPS, SCREEN_UPTIME, SCREEN_FRAMES, SCREEN_CONTRAST, SCREEN_PPS, SCREEN_VERSION, SCREEN_MAX } menu_screen;
typedef enum { SCREEN_TREND_MAIN, SCREEN_TREND_AUTO_V, SCREEN_TREND_AUTO_H, SCREEN_TREND_V_SCALE, SCREEN_TREND_H_SCALE, SCREEN_TREND_EXIT, SCREEN_TREND_MAX } menu_trend_screen;
typedef enum { SCREEN_GPS_TIME, SCREEN_GPS_LATITUDE, SCREEN_GPS_LONGITUDE, SCREEN_GPS_LATITUDE_DEC, SCREEN_GPS_LONGITUDE_DEC, SCREEN_GPS_LOCATOR, SCREEN_GPS_ALTITUDE, SCREEN_GPS_GEOID, SCREEN_GPS_SATELITES, SCREEN_GPS_HDOP, SCREEN_GPS_BAUDRATE, SCREEN_GPS_TIME_OFFSET, SCREEN_GPS_DATE_FORMAT, SCREEN_GPS_MODEL, SCREEN_GPS_LAST_FRAME, SCREEN_GPS_EXIT, SCREEN_GPS_MAX } menu_gps_screen;
typedef enum { SCREEN_PPB_MEAN, SCREEN_PPB_INST, SCREEN_PPB_FREQUENCY, SCREEN_PPB_ERROR, SCREEN_PPB_CORRECTION, SCREEN_PPB_PWM, SCREEN_PPB_OCXO_MODEL, SCREEN_PPB_WARMUP_TIME, SCREEN_PPB_ALGO, SCREEN_PPB_CORRECTION_FACTOR, SCREEN_PPB_MILLIS, SCREEN_PPB_AUTO_SAVE_PWM, SCREEN_PPB_AUTO_SYNC_PPS, SCREEN_PPB_LOCK_THRESHOLD, SCREEN_PPB_EXIT, SCREEN_PPB_MAX } menu_ppb_screen;
typedef enum { SCREEN_PPS_SHIFT, SCREEN_PPS_SHIFT_MS, SCREEN_PPS_SYNC_COUNT, SCREEN_PPS_SYNC_MODE, SCREEN_PPS_SYNC_DELAY, SCREEN_PPS_SYNC_THRESHOLD, SCREEN_PPS_FORCE_SYNC, SCREEN_PPS_EXIT, SCREEN_PPS_MAX } menu_pps_screen;

// Possible baudrate values
typedef enum { BAUDRATE_9600, BAUDRATE_19200, BAUDRATE_38400, BAUDRATE_57600, BAUDRATE_115200, BAUDRATE_230400, BAUDRATE_460800, BAUDRATE_921600, BAUDRATE_MAX} baudrate;

static menu_screen current_menu_screen = SCREEN_MAIN;
static menu_trend_screen current_menu_trend_screen = SCREEN_TREND_MAIN;
static menu_gps_screen current_menu_gps_screen = SCREEN_GPS_TIME;
static menu_ppb_screen current_menu_ppb_screen = SCREEN_PPB_MEAN;
static menu_pps_screen current_menu_pps_screen = SCREEN_PPS_SHIFT;
static uint8_t      menu_level          = 0;
static uint32_t     last_encoder_value  = 0;
static uint32_t     last_menu_change    = 0;

static bool         auto_save_pwm_done  = false;
static bool         auto_sync_pps_done  = false;

#define TREND_MAX_SIZE      7208 // 112 * 64 (TREND_MAX_H_SCALE) + 40 (TREND_SCREEN_SIZE)
#define TREND_SCREEN_SIZE   40
#define TREND_UNSET_VALUE   0xFFFF
#define TREND_MAX_H_SCALE   64
#define TREND_MAX_SHIFT     7168 // 7208 (TREND_MAX_SIZE) - 40 (TREND_SCREEN_SIZE)
static uint16_t     ppb_trend_values[TREND_MAX_SIZE];
static uint32_t     ppb_trend_position = 0;
static uint32_t     ppb_trend_size = 0;

uint32_t    trend_v_scale = 70; 
uint32_t    trend_h_scale = 1;
uint32_t    trend_shift = 0; 
uint8_t     trend_arrow = TREND_LEFT_CODE;

bool        trend_auto_h = true;
bool        trend_auto_v = true;

uint32_t    gps_baudrate = GPS_DEFAULT_BAUDRATE;
baudrate    gps_baudrate_enum = BAUDRATE_9600;

int8_t      gps_time_offset = 0;    // -14/+14
int8_t      gps_day_offset  = 0;    // -1/+1

#define         DATE_TIME_DURATION  5000 // Change date/time screen display every 5 seconds
static uint32_t last_hour_date_screen_update = 0;

uint32_t    ppb_lock_threshold = DEFAULT_PPB_LOCK_THRESHOLD;

correction_algo_type displayed_correction_algorithm;

static void menu_to_string_with_two_decimals(int value, char *buffer, size_t bufferSize)
{
    // Divide the value by 100 and keep the remainder.
    int integerPart = abs(value / 100);
    int decimalPart = abs(value % 100);

    // Make sure negative values <0 are displayed correctly.
    snprintf(buffer, bufferSize, "%s%d.%02d",
             value < 0 ? "-" : "",
             integerPart,
             decimalPart);
}

uint32_t menu_get_baudrate_value(baudrate baudrate_enum)
{
    uint32_t result;
    switch (baudrate_enum)
    {
        default:
        case BAUDRATE_9600:
            result = 9600;
            break;
        case BAUDRATE_19200:
            result = 19200;
            break;
        case BAUDRATE_38400:
            result = 38400;
            break;
        case BAUDRATE_57600:
            result = 57600;
            break;
        case BAUDRATE_115200:
            result = 115200;
            break;
        case BAUDRATE_230400:
            result = 230400;
            break;
        case BAUDRATE_460800:
            result = 460800;
            break;
        case BAUDRATE_921600:
            result = 921600;
            break;
    }
    return result;
}

baudrate menu_get_baudrate_enum(uint32_t baudrate_value)
{
    baudrate result;
    if(baudrate_value <= 9600)
    {
        result = BAUDRATE_9600;
    }
    else if(baudrate_value <= 19200)
    {
        result = BAUDRATE_19200;
    }
    else if(baudrate_value <= 38400)
    {
        result = BAUDRATE_38400;
    }
    else if(baudrate_value <= 57600)
    {
        result = BAUDRATE_57600;
    }
    else if(baudrate_value <= 115200)
    {
        result = BAUDRATE_115200;
    }
    else if(baudrate_value <= 230400)
    {
        result = BAUDRATE_230400;
    }
    else if(baudrate_value <= 460800)
    {
        result = BAUDRATE_460800;
    }
    else // if(baudrate_value <= 921600)
    {
        result = BAUDRATE_921600;
    }
    return result;
}

void menu_set_gps_baudrate(uint32_t baudrate)
{
    if(baudrate != gps_baudrate)
    {   // Baudrate changed
        gps_baudrate = baudrate;
        gps_baudrate_enum = menu_get_baudrate_enum(baudrate);
        gps_configure_module_uart(gps_baudrate);
        gps_reconfigure_uart(gps_baudrate);
    }
}

void menu_set_correction_algorithm(correction_algo_type algo)
{
    displayed_correction_algorithm = algo;
}

void menu_set_current_menu(uint8_t current_menu)
{
    if(current_menu > 0 && current_menu < SCREEN_MAX)
    {
        current_menu_screen = current_menu;
    }
}


static void menu_force_redraw() { refresh_screen = true; }

void init_trend_values()
{
    for(int i = 0 ; i < TREND_MAX_SIZE ; i++)
    {
        ppb_trend_values[i] = TREND_UNSET_VALUE;
    }
}

static uint32_t get_trend_data(uint32_t index)
{
    int32_t read_index = (ppb_trend_position + index);
    if(read_index<0)
    {   // Wrap around
        read_index = TREND_MAX_SIZE + read_index;
    }
    return ppb_trend_values[read_index];
}

static uint32_t get_trend_value(uint32_t position, uint32_t shift, uint32_t h_scale)
{
    if(h_scale == 1)
    {   // No h scaling
        return get_trend_data(position-TREND_SCREEN_SIZE-shift);
    }
    else
    {   // Compute mean value over h-scale size
        uint32_t result = 0;
        uint32_t new_value;
        for(uint32_t i = 0 ; i < h_scale ; i ++)
        {   // ' - (ppb_trend_position % h_scale)' : Use the same start position for each point in time within the given scale group
            new_value = get_trend_data((position*h_scale) + i - (TREND_SCREEN_SIZE*h_scale) - (ppb_trend_position % h_scale) - shift);
            if(new_value == TREND_UNSET_VALUE)
            {   // Don't compte mean value if one value is unset
                return TREND_UNSET_VALUE;
            }
            result += new_value;
        }
        return result/h_scale;
    }
}

static uint32_t get_trend_peak_value(uint32_t shift)
{
    uint32_t peak_value = 0;
    uint32_t cur_value;
    for(int32_t pos = 0; pos < TREND_SCREEN_SIZE ; pos++)
    {
        cur_value = get_trend_value(pos,shift,trend_h_scale);
        if((cur_value != TREND_UNSET_VALUE) && (cur_value > peak_value))
        {
            peak_value = cur_value;
        }
    }
    return peak_value;
}

static void add_trend_value(uint32_t value)
{
    ppb_trend_values[ppb_trend_position]=value;
    ppb_trend_position++;
    if(ppb_trend_position>=TREND_MAX_SIZE)
    {
        ppb_trend_position = 0;
    }
    else
    {
        ppb_trend_size++;
    }
}

static uint32_t menu_round_v_scale(uint32_t scale)
{
    uint32_t rounded_scale;
    if(scale < 70)
    {   // 70 is the lower possible scale (0.1 ppb = 1px)
        rounded_scale = 70;
    }
    else if(scale > 2000)
    {   // For large values round scale to 10 ppb
        rounded_scale = round(((double)scale)/1000)*1000;
    }
    else if(scale > 200)
    {   // For medium values round scale to 1 ppb
        rounded_scale = round(((double)scale)/100)*100;
    }
    else
    {   // For smaller values, round scale to 0.1 ppb
        rounded_scale = round(((double)scale)/10)*10;
    }
    return rounded_scale;
}

static uint32_t menu_roud_h_scale(uint32_t scale)
{
    uint32_t rounded_scale = 0;
    if(scale > TREND_MAX_H_SCALE)
    {
        rounded_scale = TREND_MAX_H_SCALE;
    }
    else if(scale < 1)
    {
        rounded_scale = 1;
    }
    else
    {   // Only keep powers of 2
        uint8_t shift = 6;
        while(rounded_scale == 0)
        {
            rounded_scale = ((scale >> shift) << shift);
            shift--;
        }
    }
    return rounded_scale;
}

static void menu_draw_trend(uint32_t shift)
{   // Horizontal autoscale
    if(trend_auto_h)
    {   // Need to zoom horizontally
        trend_h_scale = menu_roud_h_scale(ppb_trend_size/TREND_SCREEN_SIZE);
    }
    // Vertical auto-scale
    if(trend_auto_v)
    {   // Determine scale, to fit the screen
        trend_v_scale = menu_round_v_scale(get_trend_peak_value(shift));
    }
    for(int col_screen = 0 ; col_screen < 8 ; col_screen++)
    {
        uint8_t cust_char[8] = {0};
        for(int col_char = 0; col_char < 5 ; col_char++)
        {
            uint32_t cur_ppb = get_trend_value(col_screen * 5 + col_char,shift,trend_h_scale);
            // Ignore unset values
            if(cur_ppb != TREND_UNSET_VALUE)
            {   
                uint8_t cur_val = cur_ppb >= trend_v_scale ? 7 : cur_ppb * 7 / trend_v_scale;
                cust_char[7-cur_val]  |= (0b10000 >> col_char);
            }
        }
        LCD_CreateChar(col_screen,cust_char);
        LCD_PutCustom(col_screen,1,col_screen);
    }
}

#define PPB_STRING_SIZE     5
#define SCREEN_BUFFER_SIZE  14

static void menu_format_ppb(char* ppb_string, int32_t ppb_value)
{
    int32_t ppb = abs(ppb_value);

    if (ppb ==  0xFFFF) {
        strcpy(ppb_string, "   ?");
    } else if (ppb > 999999) {
        strcpy(ppb_string, ">10k");
    } else if (ppb > 9999) {
        snprintf(ppb_string, PPB_STRING_SIZE, "%4ld", (ppb / 100));
    } else if (ppb > 999) {
        snprintf(ppb_string, PPB_STRING_SIZE, "%ld.%01ld", ppb / 100, ((ppb % 100)/10));
    } else {
        snprintf(ppb_string, PPB_STRING_SIZE, "%ld.%02ld", ppb / 100, ppb % 100);
    }
}

static void menu_draw()
{
    char    screen_buffer[SCREEN_BUFFER_SIZE];
    char    ppb_string[PPB_STRING_SIZE];
    int32_t ppb;

    switch (current_menu_screen) {
    default:
    case SCREEN_MAIN:
    case SCREEN_DATE:
    case SCREEN_DATE_TIME:
        // Main screen with satellites, ppb and UTC time
        menu_format_ppb(ppb_string,frequency_get_ppb());
        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%02d %s", num_sats, ppb_string);
        LCD_Puts(1, 0, screen_buffer);
        if(current_menu_screen == SCREEN_MAIN)
        {
            LCD_Puts(0, 1, (const char*)mcu_time_string);
        }
        else if(current_menu_screen == SCREEN_DATE)
        {
            LCD_Puts(0, 1, gps_date);
        }
        else // SCREEN_DATE_TIME
        {
            uint32_t now = HAL_GetTick();
            uint32_t duration = now - last_hour_date_screen_update;
            if(duration <= DATE_TIME_DURATION)
            {
                LCD_Puts(0, 1, (const char*)mcu_time_string);
            }
            else
            {
                LCD_Puts(0, 1, gps_date);
            }
            if(duration >= 2*DATE_TIME_DURATION)
            {
                last_hour_date_screen_update = now;
            }
        }
        break;
    case SCREEN_TREND:
        // Trend screen 
        if(menu_level == 0)
        {
            menu_format_ppb(ppb_string,frequency_get_ppb());
            snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%02d %s", num_sats, ppb_string);
            LCD_Puts(1, 0, screen_buffer);
            menu_draw_trend(0);
        }
        else
        {
            switch (current_menu_trend_screen)
            {
                default:
                case SCREEN_TREND_MAIN:
                    if(menu_level == 1)
                    {
                        menu_format_ppb(ppb_string,frequency_get_ppb());
                        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%02d/%s", num_sats, ppb_string);
                        LCD_Puts(1, 0, screen_buffer);
                        menu_draw_trend(0);
                    }
                    else
                    {   // Show value at the left of the screen
                        menu_format_ppb(ppb_string,get_trend_value(TREND_SCREEN_SIZE-1,trend_shift,trend_h_scale));
                        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%03ld%c%s", trend_shift,trend_arrow,ppb_string);
                        LCD_Puts(0, 0, screen_buffer);
                        menu_draw_trend(trend_shift);
                    }
                    break;
                case SCREEN_TREND_AUTO_V:
                    LCD_Puts(1, 0, menu_level == 1 ? "Auto-V:":"Auto-V?");
                    LCD_Puts(0, 1, "        ");
                    LCD_Puts(0, 1, trend_auto_v ? "      ON" : "     OFF");
                    break;
                case SCREEN_TREND_AUTO_H:
                    LCD_Puts(1, 0, menu_level == 1 ? "Auto-H:":"Auto-H?");
                    LCD_Puts(0, 1, "        ");
                    LCD_Puts(0, 1, trend_auto_h ? "      ON" : "     OFF");
                    break;
                case SCREEN_TREND_V_SCALE:
                    LCD_Puts(1, 0, menu_level == 1 ? "V-Scal:":"V-Scal?");
                    LCD_Puts(0, 1, "        ");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld.%02ld", trend_v_scale / 100, trend_v_scale % 100);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_TREND_H_SCALE:
                    LCD_Puts(1, 0, menu_level == 1 ? "H-Scal:":"H-Scal?");
                    LCD_Puts(0, 1, "        ");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", trend_h_scale);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_TREND_EXIT:
                    LCD_Puts(1, 0, "Exit?");
                    LCD_Puts(0, 1, "        ");
                    break;
            }
        }
        break;
    case SCREEN_PPB:
        // Screen with ppb
        if(menu_level == 0)
        {
            ppb = frequency_get_ppb();
            LCD_Puts(1, 0, "PPB:   ");
            LCD_Puts(0, 1, "        ");
            menu_to_string_with_two_decimals(ppb, screen_buffer, SCREEN_BUFFER_SIZE);
            LCD_Puts(0, 1, screen_buffer);
        }
        else
        {
            // Clear line 2
            LCD_Puts(0, 1, "        ");
            switch (current_menu_ppb_screen)
            {
                default:
                case SCREEN_PPB_MEAN:
                    ppb = frequency_get_ppb();
                    LCD_Puts(1, 0, "Mean:");
                    menu_to_string_with_two_decimals(ppb, screen_buffer, SCREEN_BUFFER_SIZE);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPB_INST:
                    {
                    LCD_Puts(1, 0, "Inst:");
                    int32_t ppb_inst = (int64_t)ppb_error * 1000000000 * 100 / ((int64_t)HAL_RCC_GetHCLKFreq());
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld.%02d", ppb_inst / 100, abs(ppb_inst) % 100);
                    LCD_Puts(0, 1, screen_buffer);
                    }
                    break;
                case SCREEN_PPB_FREQUENCY:
                    LCD_Puts(1, 0, "Freq:");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", ppb_frequency);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPB_ERROR:
                    LCD_Puts(1, 0, "Error:");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", ppb_error);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPB_CORRECTION:
                    LCD_Puts(1, 0, "Corr.:");
                    if(frequency_adjustment_allowed())
                    {
                        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", ppb_correction);
                        LCD_Puts(0, 1, screen_buffer);
                    }
                    else
                    {
                        LCD_Puts(0, 1, "Warm-up");
                    }
                    break;
                case SCREEN_PPB_PWM:
                    LCD_Puts(1, 0, "PWM:");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", TIM1->CCR2);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPB_OCXO_MODEL:
                    LCD_Puts(1, 0, menu_level == 1 ? "OCXO:":"OCXO?");
                    switch(ocxo_model)
                    {
                        case OCXO_MODEL_ISOTEMP:
                            LCD_Puts(0, 1, "ISOTEMP");
                            break;
                        case OCXO_MODEL_OX256B:
                            LCD_Puts(0, 1, "OX256B");
                            break;
                        default:
                        case OCXO_MODEL_UNKNOWN:
                            LCD_Puts(0, 1, "Unknown");
                            break;
                    }
                    break;
                case SCREEN_PPB_WARMUP_TIME:
                    LCD_Puts(1, 0, menu_level == 1 ? "Warmup:":"Warmup?");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", warmup_time_seconds);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPB_ALGO:
                    LCD_Puts(1, 0, menu_level == 1 ? "Algo.:":"Algo?");
                    switch(displayed_correction_algorithm)
                    {
                        case CORRECTION_ALGO_DANKAR:
                            LCD_Puts(0, 1, "Dankar");
                            break;
                        case CORRECTION_ALGO_ERIC_H:
                            LCD_Puts(0, 1, "Eric H");
                            break;
                        default:
                        case CORRECTION_ALGO_FREDZO:
                            LCD_Puts(0, 1, "Fredzo");
                            break;
                    }
                    break;
                case SCREEN_PPB_CORRECTION_FACTOR:
                    LCD_Puts(1, 0, menu_level == 1 ? "Corr.F:":"Corr.F?");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", correction_factor);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPB_MILLIS:
                    LCD_Puts(1, 0, "Millis:");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", ppb_millis);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPB_AUTO_SAVE_PWM:
                    LCD_Puts(1, 0, menu_level == 1 ? "PWM S.:":"PWM S.?");
                    LCD_Puts(0, 1, pwm_auto_save ? "      ON" : "     OFF");
                    break;
                case SCREEN_PPB_AUTO_SYNC_PPS:
                    LCD_Puts(1, 0, menu_level == 1 ? "PPS S.:":"PPS S.?");
                    LCD_Puts(0, 1, pps_ppm_auto_sync ? "      ON" : "     OFF");
                    break;
                case SCREEN_PPB_LOCK_THRESHOLD:
                    LCD_Puts(1, 0, menu_level == 1 ? "PPB Lk:":"PPB Lk?");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld.%02ld", ppb_lock_threshold / 100, ppb_lock_threshold % 100);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPB_EXIT:
                    LCD_Puts(1, 0, "Exit?");
                    LCD_Puts(0, 1, "        ");
                    break;
            }
        }
        break;
    case SCREEN_PWM:
        // Screen with current PPM
        LCD_Puts(1, 0, "PWM:   ");
        LCD_Puts(0, 1, "        ");
        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", TIM1->CCR2);
        LCD_Puts(0, 1, screen_buffer);
        break;
    case SCREEN_GPS:
        if(menu_level == 0)
        {
            snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "GPS:%02d\5", num_sats);
            LCD_Puts(1, 0, screen_buffer);
            LCD_Puts(0, 1, (const char*)mcu_time_string);
        }
        else
        {
            // Clear line 2
            LCD_Puts(0, 1, "        ");
            switch (current_menu_gps_screen)
            {
                default:
                case SCREEN_GPS_TIME:
                    LCD_Puts(1, 0, "Time:");
                    LCD_Puts(0, 1, (const char*)mcu_time_string);
                    break;
                case SCREEN_GPS_LATITUDE:
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "Lat.: %s", gps_n_s);
                    LCD_Puts(1, 0, screen_buffer);
                    LCD_Puts(0, 1, gps_latitude);
                    break;
                case SCREEN_GPS_LONGITUDE:
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "Long.:%s", gps_e_w);
                    LCD_Puts(1, 0, screen_buffer);
                    LCD_Puts(0, 1, gps_longitude);
                    break;
                case SCREEN_GPS_LATITUDE_DEC:
                    {
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "Lat.D:");
                    LCD_Puts(1, 0, screen_buffer);
                    const char *fmt = "%d.%d";
                    double gps_latitude_double_abs = gps_latitude_double;
                    if (gps_latitude_double < 0.0)
                    {
                            gps_latitude_double_abs *= -1.0;
                            fmt = "-%d.%d";
                    }
                    double coord_int = floor(gps_latitude_double_abs);
                    double coord_frac = (gps_latitude_double_abs - coord_int)*1000000;
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, fmt, ((int)coord_int), ((int)coord_frac));
                    LCD_Puts(0, 1, screen_buffer);
                    }
                break;
                case SCREEN_GPS_LONGITUDE_DEC:
                    {
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "Long.D:");
                    LCD_Puts(1, 0, screen_buffer);
                    const char *fmt = "%d.%d";
                    double gps_longitude_double_abs = gps_longitude_double;
                    if (gps_longitude_double < 0.0)
                    {
                            gps_longitude_double_abs *= -1.0;
                            fmt = "-%d.%d";
                    }
                    double coord_int = floor(gps_longitude_double_abs);
                    double coord_frac = (gps_longitude_double_abs - coord_int)*1000000;
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, fmt, ((int)coord_int), ((int)coord_frac));
                    LCD_Puts(0, 1, screen_buffer);
                    }
                    break;
                case SCREEN_GPS_LOCATOR:
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "Lcator:");
                    LCD_Puts(1, 0, screen_buffer);
                    LCD_Puts(0, 1, gps_locator);
                    break;
                case SCREEN_GPS_ALTITUDE:
                    {
                        double alt_int = floor(gps_msl_altitude);
                        double alt_frac = (gps_msl_altitude - alt_int)*10;
                        LCD_Puts(1, 0, "Alt.:");
                        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%d.%d", ((int)alt_int), ((int)alt_frac));
                        LCD_Puts(0, 1, screen_buffer);
                    }
                    break;
                case SCREEN_GPS_GEOID:
                    {
                        double geoid_int = floor(gps_geoid_separation);
                        double geoid_frac = (gps_geoid_separation - geoid_int)*10;
                        LCD_Puts(1, 0, "Geoid:");
                        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%d.%d", ((int)geoid_int), ((int)geoid_frac));
                        LCD_Puts(0, 1, screen_buffer);
                    }
                    break;
                case SCREEN_GPS_SATELITES:
                    LCD_Puts(1, 0, "Sat. #:");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%02d", num_sats);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_GPS_HDOP:
                    LCD_Puts(1, 0, "HDOP:");
                    LCD_Puts(0, 1, gps_hdop);
                    break;
                case SCREEN_GPS_BAUDRATE:
                    LCD_Puts(1, 0, menu_level == 1 ? "Baud:":"Baud?");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", gps_baudrate);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_GPS_TIME_OFFSET:
                    LCD_Puts(1, 0, menu_level == 1 ? "TZ ofs:":"TZ ofs?");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%2d", (int)gps_time_offset);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_GPS_DATE_FORMAT:
                    LCD_Puts(1, 0, menu_level == 1 ? "Dt Fmt:":"Dt fmt?");
                    LCD_Puts(0, 1, (gps_date_format == DATE_FORMAT_UTC) ? "dd/mm/yy" : ((gps_date_format == DATE_FORMAT_US) ? "mm/dd/yy" : ((gps_date_format == DATE_FORMAT_ISO) ? "yy/mm/dd" : ((gps_date_format == DATE_FORMAT_UTC_DOT) ? "dd.mm.yy" : "yy-mm-dd"))));
                    break;
                case SCREEN_GPS_MODEL:
                    LCD_Puts(1, 0, menu_level == 1 ? "Model:":"Model?");
                    switch(gps_model)
                    {
                        case GPS_MODEL_ATGM336H:
                            LCD_Puts(0, 1, "ATGM336H");
                            break;
                        case GPS_MODEL_NEO6M:
                            LCD_Puts(0, 1, "NEO-6M");
                            break;
                        case GPS_MODEL_NEOM9N:
                            LCD_Puts(0, 1, "NEO-M9N");
                            break;
                        default:
                        case GPS_MODEL_UNKNOWN:
                            LCD_Puts(0, 1, menu_level == 1 ? "Unknown":"Auto");
                            break;
                    }
                    break;
                case SCREEN_GPS_LAST_FRAME:
                    LCD_Puts(1, 0, "Frame:");
                    LCD_Puts(0, 1, gps_last_frame);
                    if(gps_last_frame_changed)
                    {
                        menu_force_redraw();
                        gps_last_frame_changed = false;
                    }
                    break;
                case SCREEN_GPS_EXIT:
                    LCD_Puts(1, 0, "Exit?");
                    LCD_Puts(0, 1, "        ");
                    break;
            }
        }
        break;
    case SCREEN_UPTIME:
        LCD_Puts(1, 0, "UPTIME:");
        LCD_Puts(0, 1, "        ");
        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", device_uptime);
        LCD_Puts(0, 1, screen_buffer);
        break;
    case SCREEN_FRAMES:
        LCD_Puts(1, 0, "GGA FR:");
        LCD_Puts(0, 1, "        ");
        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", gga_frames);
        LCD_Puts(0, 1, screen_buffer);
        break;
    case SCREEN_CONTRAST:
        LCD_Puts(1, 0, menu_level == 0 ? "CNTRST:":"CNTRST?");
        LCD_Puts(0, 1, "        ");
        snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%d", contrast);
        LCD_Puts(0, 1, screen_buffer);
        break;
    case SCREEN_PPS:
        // Screen with pps
        // Clear line 2
        LCD_Puts(0, 1, "        ");
        if(menu_level == 0)
        {
            snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "PPS:%3ld", pps_sync_count);
            LCD_Puts(1, 0, screen_buffer);
            snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", pps_error);
            LCD_Puts(0, 1, screen_buffer);
        }
        else
        {
            switch (current_menu_pps_screen)
            {
                default:
                case SCREEN_PPS_SHIFT:
                    LCD_Puts(1, 0, "Shift:");
                    // Check we have enough space for minus sign
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", (pps_error < -9999999) ? abs(pps_error) : pps_error);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPS_SHIFT_MS:
                    LCD_Puts(1, 0, "Sft ms:");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld.%04d", pps_millis / 10000, abs(pps_millis) % 10000);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPS_SYNC_COUNT:
                    LCD_Puts(1, 0, "SynCnt:");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", pps_sync_count);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPS_SYNC_MODE:
                    LCD_Puts(1, 0, menu_level == 1 ? "Sync.:":"Sync.?");
                    LCD_Puts(0, 1, pps_sync_on ? "      ON" : "     OFF");
                    break;
                case SCREEN_PPS_SYNC_DELAY:
                    LCD_Puts(1, 0, menu_level == 1 ? "Delay:":"Delay?");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", pps_sync_delay);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPS_SYNC_THRESHOLD:
                    LCD_Puts(1, 0, menu_level == 1 ? "Thrsld:":"Thrsld?");
                    snprintf(screen_buffer, SCREEN_BUFFER_SIZE, "%ld", pps_sync_threshold);
                    LCD_Puts(0, 1, screen_buffer);
                    break;
                case SCREEN_PPS_FORCE_SYNC:
                    if(menu_level == 1)
                    {
                        LCD_Puts(1, 0,  " Force ");
                        LCD_Puts(0, 1, "  sync ?");
                    }
                    else
                    {
                        LCD_Puts(1, 0,  " Forced");
                        LCD_Puts(0, 1, "  sync !");
                        sync_pps_out = true;
                        menu_level = 1;
                    }
                    break;
                case SCREEN_PPS_EXIT:
                    LCD_Puts(1, 0, "Exit?");
                    LCD_Puts(0, 1, "        ");
                    break;
            }
        }
        break;
    case SCREEN_VERSION:
        LCD_Puts(1, 0, "Vers.:");
        LCD_Puts(0, 1, FIRMWARE_VERSION);
        break;
    }
}

void menu_run()
{

    // Detect rotary encoder value change
    uint32_t new_encoder_value = TIM3->CNT / 2;
    if(new_encoder_value != last_encoder_value)
    {
        menu_screen previous_menu_screen = current_menu_screen;
        int encoder_increment = (new_encoder_value < last_encoder_value)? -1 : +1;
        // Handle overflow cases
        if(new_encoder_value == 32767 && last_encoder_value == 0)
        {
            encoder_increment = -1;
        }
        else if (new_encoder_value == 0 && last_encoder_value == 32767)
        {
            encoder_increment = +1;
        }
        if(menu_level == 0)
        {   // Main menu => change menu screen
            current_menu_screen =  (current_menu_screen + encoder_increment) % SCREEN_MAX;
            if(current_menu_screen >= SCREEN_MAX) current_menu_screen = SCREEN_MAX-1; // Roll over for first sceen - 1
            uint32_t now = HAL_GetTick();
            if(current_menu_screen != previous_menu_screen)
            {
                last_menu_change = now;
            }
            // Reset counter for date/time screen
            last_hour_date_screen_update = now;
            LCD_Clear();
            menu_force_redraw();
        }
        else if(menu_level == 1)
        {   // Sub menu
            switch(current_menu_screen)
            {
                case SCREEN_TREND:
                    {
                        // Trend view => change trend menu
                        current_menu_trend_screen =  (current_menu_trend_screen + encoder_increment) % SCREEN_TREND_MAX;
                        if(current_menu_trend_screen >= SCREEN_TREND_MAX) current_menu_trend_screen = SCREEN_TREND_MAX-1; // Roll over for first sceen - 1
                        LCD_Clear();
                        menu_force_redraw();
                    }
                    break;
                case SCREEN_PWM:
                    // Go back to main menu
                    LCD_Clear();
                    menu_force_redraw();
                    menu_level = 0;
                    break;
                case SCREEN_PPB:
                    {
                        // PPB view => change ppb menu
                        current_menu_ppb_screen =  (current_menu_ppb_screen + encoder_increment) % SCREEN_PPB_MAX;
                        if(current_menu_ppb_screen >= SCREEN_PPB_MAX) current_menu_ppb_screen = SCREEN_PPB_MAX-1; // Roll over for first sceen - 1
                        LCD_Clear();
                        menu_force_redraw();
                    }
                    break;
                case SCREEN_GPS:
                    {
                        // GPS view => change gps menu
                        current_menu_gps_screen =  (current_menu_gps_screen + encoder_increment) % SCREEN_GPS_MAX;
                        if(current_menu_gps_screen >= SCREEN_GPS_MAX) current_menu_gps_screen = SCREEN_GPS_MAX-1; // Roll over for first sceen - 1
                        LCD_Clear();
                        menu_force_redraw();
                    }
                    break;
                case SCREEN_CONTRAST:
                    // Update contrast
                    contrast += encoder_increment;
                    if(contrast < 0) contrast = 0;
                    if(contrast > 100) contrast = 100;
                    update_contrast();
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                case SCREEN_PPS:
                    {
                        // PPB view => change ppb menu
                        current_menu_pps_screen =  (current_menu_pps_screen + encoder_increment) % SCREEN_PPS_MAX;
                        if(current_menu_pps_screen >= SCREEN_PPS_MAX) current_menu_pps_screen = SCREEN_PPS_MAX-1; // Roll over for first sceen - 1
                        LCD_Clear();
                        menu_force_redraw();
                    }
                    break;
                default:
                    break;
            }
        }
        else if(menu_level == 2 && current_menu_screen == SCREEN_TREND)
        {   // Sub-sub menu for TREND screen
            switch(current_menu_trend_screen)
            {
                case SCREEN_TREND_MAIN:
                    {
                    // Update position
                    int32_t new_trend_shift = trend_shift + (encoder_increment * trend_h_scale);
                    trend_arrow = encoder_increment < 0 ? TREND_LEFT_CODE : TREND_RIGHT_CODE;
                    if(new_trend_shift < 0)
                    {
                        trend_shift = 0;
                        trend_arrow = TREND_LEFT_CODE;
                    }
                    else if(new_trend_shift >= TREND_MAX_SHIFT)
                    {
                        trend_shift = TREND_MAX_SHIFT;
                        trend_arrow = TREND_RIGHT_CODE;
                    }
                    else
                    {
                        trend_shift = new_trend_shift;
                    }
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                    }
                case SCREEN_TREND_AUTO_V:
                    // Update mode
                    trend_auto_v = !trend_auto_v;
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                case SCREEN_TREND_AUTO_H:
                    // Update mode
                    trend_auto_h = !trend_auto_h;
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                case SCREEN_TREND_V_SCALE:
                    {
                    // Update v scale
                    uint32_t multiplier;
                    if(trend_v_scale > 2000 || ((trend_v_scale == 2000) && (encoder_increment > 0)))
                    {
                        multiplier = 1000;
                    }
                    else if(trend_v_scale > 200 || ((trend_v_scale == 200) && (encoder_increment > 0)))
                    {
                        multiplier = 100;
                    }
                    else
                    {
                        multiplier = 10;
                    }
                    trend_v_scale += (multiplier*encoder_increment);
                    trend_v_scale = menu_round_v_scale(trend_v_scale);
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                    }
                case SCREEN_TREND_H_SCALE:
                    // Update v scale
                    trend_h_scale = encoder_increment > 0 ? trend_h_scale * 2 : trend_h_scale/2;
                    trend_h_scale = menu_roud_h_scale(trend_h_scale);
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                default:
                    break;
            }
        }
        else if(menu_level == 2 && current_menu_screen == SCREEN_PPB)
        {   // Sub-sub menu for PPB screen
            switch(current_menu_ppb_screen)
            {
                case SCREEN_PPB_OCXO_MODEL:
                    { // Update model
                    ocxo_model =  (ocxo_model + encoder_increment) % (OCXO_MODEL_UNKNOWN+1);
                    if(ocxo_model > OCXO_MODEL_UNKNOWN) ocxo_model = OCXO_MODEL_UNKNOWN;
                    LCD_Clear();
                    menu_force_redraw();
                    }
                    break;
                case SCREEN_PPB_WARMUP_TIME:
                    { // Update ppb lock threshold
                    int new_warmup_time = warmup_time_seconds + (encoder_increment);
                    if(new_warmup_time < 0)
                    {
                        new_warmup_time = 0;
                    }
                    else if(new_warmup_time > 1000)
                    {
                        new_warmup_time = 1000;
                    }
                    warmup_time_seconds = new_warmup_time;
                    LCD_Clear();
                    menu_force_redraw();
                    }
                    break;
                case SCREEN_PPB_ALGO:
                    { // Update algorithm
                    displayed_correction_algorithm =  (displayed_correction_algorithm + encoder_increment) % (CORRECTION_ALGO_ERIC_H+1);
                    if(displayed_correction_algorithm > CORRECTION_ALGO_ERIC_H) displayed_correction_algorithm = CORRECTION_ALGO_ERIC_H;
                    LCD_Clear();
                    menu_force_redraw();
                    }
                    break;
                case SCREEN_PPB_CORRECTION_FACTOR:
                    { // Update correction factor
                    correction_factor = increment_correction_factor_value(correction_algorithm,correction_factor,encoder_increment);
                    LCD_Clear();
                    menu_force_redraw();
                    }
                    break;
                case SCREEN_PPB_AUTO_SAVE_PWM:
                    // Update mode
                    pwm_auto_save = !pwm_auto_save;
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                case SCREEN_PPB_AUTO_SYNC_PPS:
                    // Update mode
                    pps_ppm_auto_sync = !pps_ppm_auto_sync;
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                case SCREEN_PPB_LOCK_THRESHOLD:
                    { // Update ppb lock threshold
                    int new_threshold = ppb_lock_threshold + (5*encoder_increment);
                    if(new_threshold < 0)
                    {
                        new_threshold = 0;
                    }
                    else if(new_threshold > MAX_PPB_LOCK_THRESHOLD)
                    {
                        new_threshold = MAX_PPB_LOCK_THRESHOLD;
                    }
                    ppb_lock_threshold = new_threshold;
                    LCD_Clear();
                    menu_force_redraw();
                    }
                    break;
                default:
                    break;
            }
        }
        else if(menu_level == 2 && current_menu_screen == SCREEN_GPS)
        {   // Sub-sub menu for PPB screen
            switch(current_menu_gps_screen)
            {
                case SCREEN_GPS_BAUDRATE:
                    { // Update baudrate
                    baudrate max_baudrate = BAUDRATE_MAX;
                    switch (gps_model)
                    {
                        case GPS_MODEL_ATGM336H:
                            max_baudrate = BAUDRATE_115200 + 1;
                            break;
                        default:
                            break;
                    }
                    gps_baudrate_enum =  (gps_baudrate_enum + encoder_increment) % max_baudrate;
                    if(gps_baudrate_enum >= max_baudrate) gps_baudrate_enum = max_baudrate-1; // Roll over for first sceen - 1
                    gps_baudrate = menu_get_baudrate_value(gps_baudrate_enum);
                    LCD_Clear();
                    menu_force_redraw();
                    }
                    break;
                case SCREEN_GPS_TIME_OFFSET:
                    {   // Update time offset
                        gps_time_offset += encoder_increment;
                        if (gps_time_offset > MAX_TIME_OFFSET) {
                            gps_time_offset = MIN_TIME_OFFSET;
                        } 
                        else if(gps_time_offset < MIN_TIME_OFFSET)
                        {
                            gps_time_offset = MAX_TIME_OFFSET;
                        }

                        LCD_Clear();
                        menu_force_redraw();
                    }
                    break;
                case SCREEN_GPS_DATE_FORMAT:
                    {   // Update date format
                        int new_gps_date_format = gps_date_format+encoder_increment;
                        if(new_gps_date_format < DATE_FORMAT_UTC) 
                        {
                            new_gps_date_format = DATE_FORMAT_ISO_DASH;
                        }
                        else if(new_gps_date_format > DATE_FORMAT_ISO_DASH)
                        {
                            new_gps_date_format = DATE_FORMAT_UTC;
                        }
                        gps_date_format = new_gps_date_format;
                        LCD_Clear();
                        menu_force_redraw();
                    }
                    break;
                case SCREEN_GPS_MODEL:
                    { // Update model
                    gps_model =  (gps_model + encoder_increment) % (GPS_MODEL_UNKNOWN+1);
                    if(gps_model > GPS_MODEL_UNKNOWN) gps_model = GPS_MODEL_UNKNOWN; // Roll over for first sceen - 1
                    LCD_Clear();
                    menu_force_redraw();
                    }
                    break;
                default:
                    break;
            }
        }
        else if(menu_level == 2 && current_menu_screen == SCREEN_PPS)
        {   // Sub-sub menu for PPS screen
            switch(current_menu_pps_screen)
            {
                case SCREEN_PPS_SYNC_MODE:
                    // Update mode
                    pps_sync_on = !pps_sync_on;
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                case SCREEN_PPS_SYNC_DELAY:
                    // Update delay
                    pps_sync_delay += encoder_increment;
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                case SCREEN_PPS_SYNC_THRESHOLD:
                    // Update threshold
                    pps_sync_threshold += encoder_increment;
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                case SCREEN_PPS_FORCE_SYNC:
                    // PPB view => change ppb menu
                    current_menu_pps_screen =  (current_menu_pps_screen + encoder_increment) % SCREEN_PPS_MAX;
                    if(current_menu_pps_screen >= SCREEN_PPS_MAX) current_menu_pps_screen = SCREEN_PPS_MAX-1; // Roll over for first sceen - 1
                    LCD_Clear();
                    menu_force_redraw();
                    break;
                default:
                    break;
            }
        }
        if(previous_menu_screen == SCREEN_TREND)
        {   // After trend screen, restore custom icon chars
            lcd_create_chars();
        }
        last_encoder_value = new_encoder_value;
    }

    if (rotary_get_click()) {
        if (menu_level == 0) {
            switch(current_menu_screen)
            {
                case SCREEN_TREND:
                case SCREEN_PPB:
                case SCREEN_GPS:
                case SCREEN_PWM:
                case SCREEN_CONTRAST:
                case SCREEN_PPS:
                    menu_level = 1;
                    LCD_Clear();
                    break;
                default:
                    break;
            }
        } else  if (menu_level == 1){
            switch(current_menu_screen)
            {
                case SCREEN_TREND:
                    switch(current_menu_trend_screen)
                    {
                        case SCREEN_TREND_AUTO_H:
                        case SCREEN_TREND_AUTO_V:
                        case SCREEN_TREND_MAIN:
                            menu_level = 2;
                            break;
                        case SCREEN_TREND_V_SCALE:
                            // Prevent editing v scale if auto-v is on
                            menu_level = trend_auto_v ? 1 : 2;
                            break;
                        case SCREEN_TREND_H_SCALE:
                            // Prevent editing h scale if auto-h is on
                            menu_level = trend_auto_h ? 1 : 2;
                            break;
                        case SCREEN_TREND_EXIT:
                        default:
                            // Go back to main screen to prevent returning to exit screen
                            current_menu_trend_screen = SCREEN_TREND_MAIN;
                            menu_level = 0;
                            break;
                    }
                    break;
                case SCREEN_PWM:
                    ee_storage.pwm = TIM1->CCR2;
                    EE_Write();
                    menu_level = 0;
                    break;
                case SCREEN_CONTRAST:
                    if(ee_storage.contrast != contrast)
                    {   // Contrast has changed => save it to eeprom
                        ee_storage.contrast = contrast;
                        EE_Write();
                    }
                    menu_level = 0;
                    break;
                case SCREEN_PPB:
                    switch(current_menu_ppb_screen)
                    {
                        case SCREEN_PPB_OCXO_MODEL:
                        case SCREEN_PPB_WARMUP_TIME:
                        case SCREEN_PPB_ALGO:
                        case SCREEN_PPB_CORRECTION_FACTOR:
                        case SCREEN_PPB_AUTO_SAVE_PWM:
                        case SCREEN_PPB_AUTO_SYNC_PPS:
                        case SCREEN_PPB_LOCK_THRESHOLD:
                            menu_level = 2;
                            break;
                        case SCREEN_PPB_EXIT:
                            // Go back to main screen to prevent returning to exit screen
                            current_menu_ppb_screen = SCREEN_PPB_MEAN;
                            menu_level = 0;
                            break;
                        default:
                            menu_level = 0;
                            break;
                    }
                    break;
                case SCREEN_GPS:
                    switch(current_menu_gps_screen)
                    {
                        case SCREEN_GPS_MODEL:
                        case SCREEN_GPS_DATE_FORMAT:
                        case SCREEN_GPS_TIME_OFFSET:
                        case SCREEN_GPS_BAUDRATE:
                            menu_level = 2;
                            break;
                        case SCREEN_GPS_EXIT:
                            // Go back to main screen to prevent returning to exit screen
                            current_menu_gps_screen = SCREEN_GPS_TIME;
                            menu_level = 0;
                            break;
                        default:
                            menu_level = 0;
                            break;
                    }
                    break;
                case SCREEN_PPS:
                    switch(current_menu_pps_screen)
                    {
                        case SCREEN_PPS_SYNC_MODE:
                        case SCREEN_PPS_SYNC_DELAY:
                        case SCREEN_PPS_SYNC_THRESHOLD:
                        case SCREEN_PPS_FORCE_SYNC:
                            menu_level = 2;
                            break;
                        case SCREEN_PPS_EXIT:
                            // Go back to main screen to prevent returning to exit screen
                            current_menu_pps_screen = SCREEN_PPS_SHIFT;
                            menu_level = 0;
                            break;
                        default:
                            menu_level = 0;
                            break;
                    }
                    break;
                default:
                    menu_level = 0;
                    break;
            }
            LCD_Clear();
        } else  if (menu_level == 2 && current_menu_screen == SCREEN_TREND){
            switch(current_menu_trend_screen)
            {
                case SCREEN_TREND_AUTO_V:
                    if(ee_storage.trend_auto_v != trend_auto_v)
                    {   // Save changes
                        ee_storage.trend_auto_v = trend_auto_v;
                        EE_Write();
                    }
                    break;
                case SCREEN_TREND_AUTO_H:
                    if(ee_storage.trend_auto_h != trend_auto_h)
                    {   // Save changes
                        ee_storage.trend_auto_h = trend_auto_h;
                        EE_Write();
                    }
                    break;
                case SCREEN_TREND_V_SCALE:
                    if(ee_storage.trend_v_scale != trend_v_scale)
                    {   // Save changes
                        ee_storage.trend_v_scale = trend_v_scale;
                        EE_Write();
                    }
                    break;
                case SCREEN_TREND_H_SCALE:
                    if(ee_storage.trend_h_scale != trend_h_scale)
                    {   // Save changes
                        ee_storage.trend_h_scale = trend_h_scale;
                        EE_Write();
                    }
                    break;
                default:
                    break;
            }
            menu_level = 1;
            LCD_Clear();
        } else  if (menu_level == 2 && current_menu_screen == SCREEN_PPB){
            switch(current_menu_ppb_screen)
            {
                case SCREEN_PPB_OCXO_MODEL:
                    if(ee_storage.ocxo_model != ocxo_model)
                    {   // Save changes
                        ee_storage.ocxo_model = ocxo_model;
                        // Alsa change warmup time accordingly
                        warmup_time_seconds = get_default_warmup_time(ocxo_model);
                        ee_storage.warmup_time_seconds = warmup_time_seconds;
                        EE_Write();
                    }
                    break;
                case SCREEN_PPB_WARMUP_TIME:
                    if(ee_storage.warmup_time_seconds != warmup_time_seconds)
                    {   // Save changes
                        ee_storage.warmup_time_seconds = warmup_time_seconds;
                        EE_Write();
                    }
                    break;
                case SCREEN_PPB_ALGO:
                    if(ee_storage.correction_algorithm != displayed_correction_algorithm)
                    {   // Make sure correction algo and correction are consistant before activating new algo
                        // Reset correction factor to default value when algo is changed
                        correction_factor = get_default_correction_factor(displayed_correction_algorithm);
                        correction_algorithm = displayed_correction_algorithm;
                        // Save changes
                        ee_storage.correction_algorithm = correction_algorithm;
                        ee_storage.correction_factor = correction_factor;
                        EE_Write();
                    }
                    break;
                case SCREEN_PPB_CORRECTION_FACTOR:
                    if(ee_storage.correction_factor != correction_factor)
                    {   // Save changes
                        ee_storage.correction_factor = correction_factor;
                        EE_Write();
                    }
                    break;
                case SCREEN_PPB_AUTO_SAVE_PWM:
                    if(ee_storage.pwm_auto_save != pwm_auto_save)
                    {   // Save changes
                        ee_storage.pwm_auto_save = pwm_auto_save;
                        EE_Write();
                    }
                    break;
                case SCREEN_PPB_AUTO_SYNC_PPS:
                    if(ee_storage.pps_ppm_auto_sync != pps_ppm_auto_sync)
                    {   // Save changes
                        ee_storage.pps_ppm_auto_sync = pps_ppm_auto_sync;
                        EE_Write();
                    }
                    break;
                case SCREEN_PPB_LOCK_THRESHOLD:
                    if(ee_storage.ppb_lock_threshold != ppb_lock_threshold)
                    {   // Save changes
                        ee_storage.ppb_lock_threshold = ppb_lock_threshold;
                        EE_Write();
                    }
                    break;
                default:
                    break;
            }
            menu_level = 1;
            LCD_Clear();
        } else  if (menu_level == 2 && current_menu_screen == SCREEN_GPS){
            switch(current_menu_gps_screen)
            {
                case SCREEN_GPS_BAUDRATE:
                    if(ee_storage.gps_baudrate != gps_baudrate)
                    {   // Save changes
                        ee_storage.gps_baudrate = gps_baudrate;
                        EE_Write();
                        // Reconfigure uart
                        if(gps_configure_module_uart(gps_baudrate)>=0)
                        {   // Reconfigure uart
                            gps_reconfigure_uart(gps_baudrate);
                            // Save new baudrate on gps module
                            gps_save_config();
                        }
                    }
                    break;
                case SCREEN_GPS_TIME_OFFSET:
                    if(ee_storage.gps_time_offset != ((uint32_t)(gps_time_offset-MIN_TIME_OFFSET)))
                    {   // Save changes
                        ee_storage.gps_time_offset = gps_time_offset-MIN_TIME_OFFSET;
                        EE_Write();
                    }
                    break;
                case SCREEN_GPS_DATE_FORMAT:
                    if(ee_storage.gps_date_format != gps_date_format)
                    {   // Save changes
                        ee_storage.gps_date_format = gps_date_format;
                        EE_Write();
                    }
                    break;
                case SCREEN_GPS_MODEL:
                    if(ee_storage.gps_model != gps_model)
                    {   // Save changes
                        ee_storage.gps_model = gps_model;
                        EE_Write();
                    }
                    break;
                default:
                    break;
            }
            menu_level = 1;
            LCD_Clear();
        } else  if (menu_level == 2 && current_menu_screen == SCREEN_PPS){
            switch(current_menu_pps_screen)
            {
                case SCREEN_PPS_SYNC_MODE:
                    if(ee_storage.pps_sync_on != pps_sync_on)
                    {   // Save changes
                        ee_storage.pps_sync_on = pps_sync_on;
                        EE_Write();
                    }
                    break;
                case SCREEN_PPS_SYNC_DELAY:
                    if(ee_storage.pps_sync_delay != pps_sync_delay)
                    {   // Save changes
                        ee_storage.pps_sync_delay = pps_sync_delay;
                        EE_Write();
                    }
                    break;
                case SCREEN_PPS_SYNC_THRESHOLD:
                    if(ee_storage.pps_sync_threshold != pps_sync_threshold)
                    {   // Save changes
                        ee_storage.pps_sync_threshold = pps_sync_threshold;
                        EE_Write();
                    }
                    break;
                default:
                    break;
            }
            menu_level = 1;
            LCD_Clear();
        }
        else
        {
            menu_level = 0;
            LCD_Clear();
        }
        menu_force_redraw();
    }

    if (refresh_screen) {
        refresh_screen = false;

        // Display state icon
        if(current_menu_screen == SCREEN_TREND && (current_state_icon < 8))
        {   // Don't use custom icon in trend screen since all 8 custom chars are used for graphic display
            uint8_t icon;
            switch (current_state_icon)
            {
                default:
                case 1:
                    icon = SAT_ICON_1_CODE;
                    break;
                case 2:
                    icon = SAT_ICON_2_CODE;
                    break;
                case 3:
                    icon = SAT_ICON_3_CODE;
                    break;
                case 4:
                    icon = NO_SAT_STD_ICON_CODE;
                    break;
            }
            LCD_PutCustom(0,0,icon);
        }
        else
        {
            LCD_PutCustom(0,0,current_state_icon);
        }
        
        // Update PPB trend if needed
        if(update_trend)
        {
            add_trend_value(abs(frequency_get_ppb()));
            update_trend = false;
        }

        if (menu_level > 0 && current_menu_screen == SCREEN_PWM) {
            LCD_Puts(0, 0, " PRESS ");
            LCD_Puts(0, 1, "TO SAVE");
        } else {
            menu_draw();
        }

        // Check if we need resync or PWM save
        if(frequency_is_stable(0))
        {   // Frequency is stabilized
            // Save PWM if requested
            bool did_pwm = false;
            bool did_pps = false;
            if(pwm_auto_save && !auto_save_pwm_done)
            {
                ee_storage.pwm = TIM1->CCR2;
                EE_Write();
                // Only auto-save once per session
                auto_save_pwm_done = true;
                did_pwm = true;
            }
            if(pps_ppm_auto_sync && !auto_sync_pps_done)
            {
                sync_pps_out = true;
                // Only auto-sync once per session
                auto_sync_pps_done = true;
                did_pps = true;
            }
            if(did_pps && did_pwm)
            {
                LCD_Puts(0, 0, "PPS&PWM ");
                LCD_Puts(0, 1, " DONE ! ");
            }
            else if(did_pps)
            {
                LCD_Puts(0, 0, "  PPS  ");
                LCD_Puts(0, 1, "SYNCED!");
            }
            else if(did_pwm)
            {
                LCD_Puts(0, 0, "  PWM  ");
                LCD_Puts(0, 1, "SAVED !");
            }
        }
        bool new_ppb_lock_status = frequency_is_stable(ppb_lock_threshold);
        if(ppb_lock_status != new_ppb_lock_status )
        {   // Update PPB lock status
            ppb_lock_status = new_ppb_lock_status;
            HAL_GPIO_WritePin(PPB_LOCK_OUTPUT_GPIO_Port, PPB_LOCK_OUTPUT_Pin, !ppb_lock_status); // Active low
            if(current_menu_screen != SCREEN_TREND)
            {
                lcd_create_chars();
            }
        }

        // Check if boot menu has to be changed
        if(last_menu_change != 0 && ((HAL_GetTick() - last_menu_change) > BOOT_MENU_SAVE_TIME))
        {   // Filter on eligible boot screens
            switch (current_menu_screen)
            {
                case SCREEN_MAIN:
                case SCREEN_DATE:
                case SCREEN_DATE_TIME:
                case SCREEN_TREND:
                    if(ee_storage.boot_menu != current_menu_screen)
                    {
                        ee_storage.boot_menu = current_menu_screen;
                        EE_Write();
                    }
                    break;
                
                default:
                    break;
            }
            last_menu_change = 0;
        }
    }
}
