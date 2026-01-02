#include "gps.h"
#include "LCD.h"
#include "main.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"
#include "eeprom.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_GPS_LINE        512
#define GPS_LOCATOR_SIZE    8

char     gps_line[MAX_GPS_LINE];
char     gps_time[9]      = { '\0' };
char     gps_date[9]      = { ' ', ' ', '/', ' ', ' ', '/', ' ', ' ', '\0' };
char     gps_latitude[9]  = { '\0' };
char     gps_longitude[9] = { '\0' };
char     gps_n_s[2]       = { '\0' };
char     gps_e_w[2]       = { '\0' };
double   gps_msl_altitude;
double   gps_geoid_separation;
double   gps_latitude_double  = 0;
double   gps_longitude_double = 0;
char     gps_locator[GPS_LOCATOR_SIZE+1];

char     gps_hdop[9]      = { '\0' };
char     gps_last_frame[9]= { '\0' };
bool     gps_last_frame_changed = false;
uint8_t  num_sats         = 0;
uint32_t gga_frames       = 0;
size_t   gps_line_len     = 0;
gps_model_type  gps_model       = GPS_MODEL_UNKNOWN;
date_format     gps_date_format = DATE_FORMAT_UTC;

// Store last frame receive time
uint32_t last_frame_receive_time = 0;


#define FIFO_BUFFER_SIZE 256

typedef struct {
    uint8_t buffer[FIFO_BUFFER_SIZE];
    size_t  read;
    size_t  write;
} fifo_buffer_t;

typedef enum { FIFO_WRITE, FIFO_READ } fifo_operation;

volatile fifo_buffer_t fifo_buffer_gps  = { 0 };
volatile fifo_buffer_t fifo_buffer_comm = { 0 };

size_t fifo_next(volatile const fifo_buffer_t* fifo, fifo_operation op)
{
    if (op == FIFO_WRITE) {
        return (fifo->write + 1) % FIFO_BUFFER_SIZE;
    } else {
        return (fifo->read + 1) % FIFO_BUFFER_SIZE;
    }
}

bool fifo_write(volatile fifo_buffer_t* fifo, const uint8_t c)
{
    size_t next = fifo_next(fifo, FIFO_WRITE);
    if (next == fifo->read) {
        return false;
    }
    fifo->buffer[fifo->write] = c;
    fifo->write               = next;
    return true;
}

bool fifo_read(volatile fifo_buffer_t* fifo, uint8_t* c)
{
    if (fifo->read == fifo->write) {
        return false;
    }
    *c         = fifo->buffer[fifo->read];
    fifo->read = fifo_next(fifo, FIFO_READ);

    return true;
}

#define GPS_RX_BUFFER_SIZE  20
#define COMM_RX_BUFFER_SIZE 1

volatile uint8_t gps_it_buf[GPS_RX_BUFFER_SIZE];
volatile uint8_t comm_it_buf[COMM_RX_BUFFER_SIZE];

static void gps_start_gps_rx()
{
    if (HAL_UART_Receive_DMA(&huart3, (uint8_t*)gps_it_buf, GPS_RX_BUFFER_SIZE) != HAL_OK) {
        Error_Handler();
    }
}
static void gps_start_comm_rx()
{
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t*)comm_it_buf, COMM_RX_BUFFER_SIZE) != HAL_OK) {
        Error_Handler();
    }
}

// ATGM336H set baudrate commands
static const char*	atgm336h_baudcommands[] = {
    "$PCAS01,1*1D\r\n",     // 9600bps
    "$PCAS01,2*1E\r\n",     // 19200bps
    "$PCAS01,3*1F\r\n",     // 38400bps
    "$PCAS01,4*18\r\n",     // 57600bps
    "$PCAS01,5*19\r\n"      // 115200bps
};

// ATGM336H save configuration command
const char*	atgm336h_savecommand = "$PCAS00*01\r\n";

static void gps_sendcommand(const char* cmd, size_t len)
{
    while (huart3.gState != HAL_UART_STATE_READY);
    HAL_UART_Transmit_IT(&huart3, (const uint8_t*)cmd, len);
    // wait for transfer completed
    while (huart3.gState != HAL_UART_STATE_READY);
}

int	gps_configure_module_uart(uint32_t baudrate)
{
    const char*	command = NULL;
    switch(gps_model)
    {
        case GPS_MODEL_ATGM336H:
            switch (baudrate) {
                case 9600:
                    command = atgm336h_baudcommands[0];
                    break;
                case 19200:
                    command = atgm336h_baudcommands[1];
                    break;
                case 38400:
                    command = atgm336h_baudcommands[2];
                    break;
                case 57600:
                    command = atgm336h_baudcommands[3];
                    break;
                case 115200:
                    command = atgm336h_baudcommands[4];
                    break;
                default:
                    return -1;  // error
            }
            break;
        case GPS_MODEL_NEO6M:
            // TODO
        case GPS_MODEL_NEOM9N:
            // TODO
        case GPS_MODEL_UNKNOWN:
            break;
    }

    size_t len;
    if (command != NULL) {
        len = strlen(command);
        gps_sendcommand(command, len);
    }

    return	0;
}

void gps_reconfigure_uart(uint32_t baudrate)
{
    HAL_UART_DeInit(&huart2);
    HAL_UART_DeInit(&huart3);
    // Wait for buffers to be consumed
    HAL_Delay(50);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = baudrate;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
      Error_Handler();
    }

    huart3.Instance = USART3;
    huart3.Init.BaudRate = baudrate;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
      Error_Handler();
    }
    // Wait Uarts to init
    HAL_Delay(50);
    gps_start_gps_rx();
    gps_start_comm_rx();
}

void gps_save_config()
{
    const char* save_command = NULL;
    switch(gps_model)
    {
        case GPS_MODEL_ATGM336H:
            // Give some time for the module to reconfigure before sending the save command
            HAL_Delay(50);
            save_command = atgm336h_savecommand;
            break;
        case GPS_MODEL_NEO6M:
            // TODO
        case GPS_MODEL_NEOM9N:
            // TODO
        case GPS_MODEL_UNKNOWN:
            break;
    }

    size_t len;
    if (save_command != NULL) {
        len = strlen(save_command);
        gps_sendcommand(save_command, len);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart3) {
        for (size_t i = 0; i < GPS_RX_BUFFER_SIZE; i++) {
            fifo_write(&fifo_buffer_gps, gps_it_buf[i]);
        }
        gps_start_gps_rx();
    } else if (huart == &huart2) {
        for (size_t i = 0; i < COMM_RX_BUFFER_SIZE; i++) {
            fifo_write(&fifo_buffer_comm, comm_it_buf[i]);
        }
        gps_start_comm_rx();
    }
}

void gps_start_it()
{
    gps_start_gps_rx();
    gps_start_comm_rx();
}

static double gps_parse_coordinate(char* nmea_string, char* coord_string, size_t size)
{
    double result = 0;
    char* dot_substring = strstr(nmea_string,".");
    if(dot_substring != NULL)
    {
        uint8_t i = 0;
        uint8_t j = 0;
        bool lead = true;
        while(nmea_string[i] != 0 && j < (size-1))
        {
            if(nmea_string[i] != '.' && (nmea_string[i] != '0'||!lead))
            {
                coord_string[j] = nmea_string[i];
                j++;
                lead = false;
            }
            i++;
        }
        coord_string[j] = 0;
        // Parse value and convert to decimal
        int dot_position = dot_substring - nmea_string;
        int len = strlen(nmea_string);
        if(dot_position >= 2 && dot_position <= 5 && len < 16)
        {
            char gps_buffer[16];
            int buff_len = len-(dot_position-2);
            strncpy(gps_buffer, nmea_string+(dot_position-2),buff_len);
            gps_buffer[buff_len] = 0; // Terminate string
            double mins = atof(gps_buffer);
            buff_len = dot_position-2;
            strncpy(gps_buffer, nmea_string, buff_len);
            gps_buffer[buff_len] = 0; // Terminate string
            int deg = atoi(gps_buffer);
            result = deg + mins/60;
        }
    }
    return result;
}

static char gps_letterize(int x) {
    return (char) x + 65;
}

static void gps_compute_locator(double lat, double lon) {
    double LON_F[]={20,2.0,0.083333,0.008333};
    double LAT_F[]={10,1.0,0.0416665,0.004166};
    int i;
    lon += 180;
    lat += 90;

    for (i = 0; i < GPS_LOCATOR_SIZE/2; i++){
        if (i % 2 == 1) {
            gps_locator[i*2] = (char) (lon/LON_F[i] + '0');
            gps_locator[i*2+1] = (char) (lat/LAT_F[i] + '0');
        } else {
            gps_locator[i*2] = gps_letterize((int) (lon/LON_F[i]));
            gps_locator[i*2+1] = gps_letterize((int) (lat/LAT_F[i]));
        }
        lon = fmod(lon, LON_F[i]);
        lat = fmod(lat, LAT_F[i]);
    }
    gps_locator[i*2]=0;
}

static bool change_time(char* time_source, char* time_dest, int correction, int max_value)
{
    bool overlap = false;
    int value = (10*(time_source[0]-'0')) + (time_source[1]-'0') + correction;
    if(value > max_value)
    {
        value = 0;
        overlap = true;
    } 
    time_dest[0] = (char)((value/10)+'0');
    time_dest[1] = (char)((value%10)+'0');
    return overlap;
}

// Maybe use X-CUBE-GNSS here?
void gps_parse(char* line)
{
    if (strstr(line, "GGA") == line+3) 
    {
        char* pch = strsep(&line, ",");

        pch = strsep(&line, ","); // Time

        // GPSDO screen is updated once every second, when receiving the PPS signal
        // BUT, the GGA frame is received a fraction of second AFTER the PPS pulse
        // To achieve accurate time display, we will add one second to the received time
        // to compensate this delay

        // Let's start with seconds value, to propagate overlap to minutes and hours if needed
        bool overlap = change_time(pch+4,gps_time+6,1,59);
        if(overlap)
        {   // Need to propagate overlap to minutes
            overlap = change_time(pch+2,gps_time+3,1,59);
        }
        else
        {
            gps_time[3] = pch[2];
            gps_time[4] = pch[3];
        }

        if (gps_time_offset == 0 && !overlap) 
        {   // Leave hour unchanged
	        gps_time[0] = pch[0];
	        gps_time[1] = pch[1];
		} 
        else 
        {   // Need to fix hour
			char p0 = pch[0] - '0';
			char p1 = pch[1] - '0';
			int hour = p0 * 10 + p1;
            int relative_hour = (hour + (int)gps_time_offset);
            if(overlap)
            {   // Propagate second / minute overlap
                relative_hour+=1;
            }
            if(relative_hour >= 24)
            {
                hour = relative_hour - 24;
                gps_day_offset = 1;
            }
            else if(relative_hour < 0)
            {
                hour = relative_hour + 24;
                gps_day_offset = -1;
            }
            else
            {
                hour = relative_hour;
                gps_day_offset = 0;
            }
	        gps_time[0] = (char)((hour / 10) + '0');
	        gps_time[1] = (char)((hour % 10) + '0');
		}
        // Add separators
        gps_time[2] = ':';
        gps_time[5] = ':';
        // Terminaute time string
        gps_time[8] = '\0';

        pch = strsep(&line, ","); // Latitude
        gps_latitude_double = gps_parse_coordinate(pch,gps_latitude,sizeof(gps_latitude));
        pch = strsep(&line, ","); // N/S
        if(strlen(pch)<sizeof(gps_n_s))
        {
            strcpy(gps_n_s,pch);
            if(gps_n_s[0] == 'S')
                gps_latitude_double*=-1;
        }
        pch = strsep(&line, ","); // Longitude
        gps_longitude_double = gps_parse_coordinate(pch,gps_longitude,sizeof(gps_longitude));
        pch = strsep(&line, ","); // E/W
        if(strlen(pch)<sizeof(gps_e_w))
        {
            strcpy(gps_e_w,pch);
            if(gps_e_w[0] == 'W')
                gps_longitude_double*=-1;
        }
        gps_compute_locator(gps_latitude_double,gps_longitude_double);
        strsep(&line, ","); // Fix

        num_sats = atoi(strsep(&line, ",")); // Num sats used

        pch = strsep(&line, ","); // HDOP
        if(strlen(pch)<sizeof(gps_hdop))
        {
            strcpy(gps_hdop,pch);
        }
        
        gps_msl_altitude = atof(strsep(&line, ",")); // MSL Elevation
        strsep(&line, ","); // Unit
        gps_geoid_separation = atof(strsep(&line, ",")); // Geoid Separation
        // strsep(&line, ","); // Unit

        gga_frames++;
    } 
    else if (strstr(line, "RMC") == line+3) 
    {
        char* pch = strsep(&line, ",");

        pch = strsep(&line, ","); // Time
        pch = strsep(&line, ","); // Alert
        pch = strsep(&line, ","); // Latitude
        pch = strsep(&line, ","); // N/S
        pch = strsep(&line, ","); // Longitude
        pch = strsep(&line, ","); // E/W
        pch = strsep(&line, ","); // Speed
        pch = strsep(&line, ","); // Orientation
        pch = strsep(&line, ","); // Date

        if(strlen(pch)>=6)
        {   // Ignore empty dates
            char day0;
            char day1;
            char month0;
            char month1;
            char year0;
            char year1;
            if (gps_time_offset == 0) {
                day0 = pch[0];
                day1 = pch[1];
                month0 = pch[2];
                month1 = pch[3];
                year0 = pch[4];
                year1 = pch[5];
            } else {
                char d0 = pch[0] - '0';
                char d1 = pch[1] - '0';
                char m0 = pch[2] - '0';
                char m1 = pch[3] - '0';
                char y0 = pch[4] - '0';
                char y1 = pch[5] - '0';
                int day   = d0 * 10 + d1;
                int month = m0 * 10 + m1;
                int year  = y0 * 10 + y1;
                day += gps_day_offset;
                // Quick and dirty poor man's gregorian calendar handling
                bool is_leap_year = ((year % 4) == 0);
                if((day > (is_leap_year ? 29 : 28)) && month == 2)
                {   // Case of february
                    day = 1;
                    month = 3;
                }
                else if(day > 31 && month == 12)
                {   // Need to change year
                    day = 1;
                    month = 1;
                    year += 1; 
                }
                else if (day > 31 && (month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10))
                {   // Other month with 31 days
                    day = 1;
                    month++;
                }
                else if(day > 30)
                {   // Months with 30 days
                    day = 1;
                    month++;
                }
                else if(day < 1)
                {
                    if(month == 1)
                    {   // Need to change year
                        day = 31;
                        month = 12;
                        year--;
                    }
                    else if(month == 3)
                    {   // Case of february
                        day = is_leap_year ? 29 : 28;
                        month--;
                    }
                    else if(month == 2 || month == 4 || month == 6 ||month == 8 || month == 9 || month == 11)
                    {   // Months after a 31 day month
                        day = 31;
                        month--;
                    }
                    else
                    {   // Months after a 30 day month
                        day = 30;
                        month--;
                    }
                }
                day0   = (char)((day   / 10) + '0');
                day1   = (char)((day   % 10) + '0');
                month0 = (char)((month / 10) + '0');
                month1 = (char)((month % 10) + '0');
                year0  = (char)((year  / 10) + '0');
                year1  = (char)((year  % 10) + '0');
            }
            switch(gps_date_format)
            {
                case DATE_FORMAT_UTC:
                case DATE_FORMAT_UTC_DOT:
                default:
                    gps_date[0] = day0;
                    gps_date[1] = day1;
                    gps_date[3] = month0;
                    gps_date[4] = month1;
                    gps_date[6] = year0;
                    gps_date[7] = year1;
                    break;
                case DATE_FORMAT_US:
                    gps_date[0] = month0;
                    gps_date[1] = month1;
                    gps_date[3] = day0;
                    gps_date[4] = day1;
                    gps_date[6] = year0;
                    gps_date[7] = year1;
                    break;
                case DATE_FORMAT_ISO:
                case DATE_FORMAT_ISO_DASH:
                    gps_date[0] = year0;
                    gps_date[1] = year1;
                    gps_date[3] = month0;
                    gps_date[4] = month1;
                    gps_date[6] = day0;
                    gps_date[7] = day1;
                    break;
            }
            switch(gps_date_format)
            {
                default:
                    gps_date[2] = '/';
                    gps_date[5] = '/';
                    break;
                case DATE_FORMAT_UTC_DOT:
                    gps_date[2] = '.';
                    gps_date[5] = '.';
                    break;
                case DATE_FORMAT_ISO_DASH:
                    gps_date[2] = '-';
                    gps_date[5] = '-';
                    break;
            }
            gps_date[8] = '\0';
        }
    } 
    else if ((gps_model == GPS_MODEL_UNKNOWN) && strstr(line, "TXT") == line+3) 
    {
        bool model_found = false;
        if (strstr(line, "AT6558F-5N")) {
            // this is ATGM336H module
            gps_model = GPS_MODEL_ATGM336H;
            model_found = true;
        }
        else if(strstr(line, "HW UBX-G"))
        {
            gps_model = GPS_MODEL_NEO6M;
            model_found = true;
        }
        else if(strstr(line, "HW UBX 9"))
        {
            gps_model = GPS_MODEL_NEOM9N;
            model_found = true;
        }
        if(model_found && (ee_storage.gps_model != gps_model))
        {   // Save changes
            ee_storage.gps_model = gps_model;
            EE_Write();
        }
    }
    // Store last received frame for debug purpose
    if(strlen(line)>(sizeof(gps_last_frame)+3))
    {
        strncpy(gps_last_frame,line+3,sizeof(gps_last_frame)-1);
        gps_last_frame_changed = true;
    }
    // Get reception time
    last_frame_receive_time = HAL_GetTick();
}

#define	SEND_BUFFER_SIZE	FIFO_BUFFER_SIZE
uint8_t send_buf[SEND_BUFFER_SIZE];
uint8_t gps_send_buf[SEND_BUFFER_SIZE];
uint8_t comm_send_buf[SEND_BUFFER_SIZE];
size_t  send_size;

void gps_read()
{
    send_size = 0;
    uint8_t c;
    while (fifo_read(&fifo_buffer_gps, &c)) {
        gps_line[gps_line_len++] = c;
        send_buf[send_size++]    = c;
        if (c == '\n') {
            gps_line[gps_line_len] = '\0';
            gps_parse(gps_line);
            gps_line_len = 0;
            continue;
        }
        if (gps_line_len >= MAX_GPS_LINE) {
            gps_line_len = 0;
            return;
        }
    }

    if (send_size) {
        while (huart2.gState != HAL_UART_STATE_READY)
            ;
        memcpy(gps_send_buf, send_buf, SEND_BUFFER_SIZE);
        HAL_UART_Transmit_IT(&huart2, gps_send_buf, send_size);
    }

    send_size = 0;
    while (fifo_read(&fifo_buffer_comm, &c)) {
        send_buf[send_size++] = c;
    }
    
    if (send_size) {
        while (huart3.gState != HAL_UART_STATE_READY)
            ;
        memcpy(comm_send_buf, send_buf, SEND_BUFFER_SIZE);
        HAL_UART_Transmit_IT(&huart3, comm_send_buf, send_size);
    }
    
}
