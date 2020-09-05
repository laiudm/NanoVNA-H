/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef NANOVNAH
#define NANOVNAH

// ****************************

// Need enable HAL_USE_SPI in halconf.h
#define __USE_DISPLAY_DMA__

// Add SD card support, req enable RTC (additional settings for file system see FatFS lib ffconf.h)
#define __USE_SD_CARD__

#ifdef __USE_SD_CARD__
   // Add RTC clock support
   #define __USE_RTC__

   // add SD card serial commands
   #define __USE_SD_CARD_CMDS__

   // add backup/restore config and calibrations to SD card menu/code
   //#define __USE_SD_CARD_BACKUP_RESTORE_CONFIG_CAL__

// add SD card auto save measurments
   //#define __USE_SD_CARD_AUTO_SAVE_PERIOD__
#endif

// this enables showing the battery level percentage on the main screen
#define SHOW_BATTERY_LEVEL_PERCENT

// Add the routines to allow s-param integration
//#define USE_INTEGRATOR
#define INTEGRATOR_COEFF  0.1f

// Add the routines to calculate the component values to match an impedance (DUT on CH-0) to 50R
#define USE_LC_MATCHING

// uncomment to use a desired coloured menu theme, otherwise it's a grey theme
//#define MENU_THEME_BLUE
#define MENU_THEME_WHITE

// ****************************

#include "ch.h"
#ifdef  __USE_SD_CARD__
   #include "../FatFs/ff.h"
   #include "../FatFs/diskio.h"
#endif

// Define size of screen buffer in pixels (one pixel 16bit size)
#define SPI_BUFFER_SIZE             2048
extern uint16_t spi_buffer[SPI_BUFFER_SIZE];

#ifdef  __USE_SD_CARD__
   extern void testLog(void);

   extern FATFS *fs_volume;
   extern FIL   *fs_file;
   extern char  *fs_filename;

   #ifdef __USE_SD_CARD_AUTO_SAVE_PERIOD__
   	extern uint16_t auto_save_secs;
	#endif
#endif

#define DEFAULT_BATTERY_OFFSET_MV          150
#define DEFAULT_BATTERY_TOP_MV             4200
#define DEFAULT_BATTERY_WARNING_MV         3450
#define DEFAULT_BATTERY_BOTTOM_MV          3200

//#define DEFAULT_DAC_VALUE                2048
#define DEFAULT_DAC_VALUE                  1922

#ifndef ARRAYSIZE
   // returns the number of array elements
   #define ARRAYSIZE(x) (sizeof(x) / sizeof(*(x)))
#endif

// ******************************
// flash.c

#define FLASH_PAGESIZE             0x800

#if defined(NANOVNA_F303)
   #define SAVEAREA_MAX 7

   // Depend from config_t size, should be aligned by FLASH_PAGESIZE
   #define SAVE_CONFIG_SIZE        0x00001000
   // Depend from properties_t size, should be aligned by FLASH_PAGESIZE
   #define SAVE_PROP_CONFIG_SIZE   0x00002000

   // Save config_t and properties_t flash area (see flash7  : org = 0x08030000, len = 64k from *.ld settings)
   // Properties save area follow after config
   // len = SAVE_CONFIG_SIZE + SAVEAREA_MAX * SAVE_PROP_CONFIG_SIZE   0x00010000  64k
   #define SAVE_CONFIG_ADDR        0x08030000
   #define SAVE_PROP_CONFIG_ADDR   (SAVE_CONFIG_ADDR + SAVE_CONFIG_SIZE)
   #define SAVE_FULL_AREA_SIZE     (SAVE_CONFIG_SIZE + SAVEAREA_MAX * SAVE_PROP_CONFIG_SIZE)
#else
   #define SAVEAREA_MAX 5

   // Depend from config_t size, should be aligned by FLASH_PAGESIZE
   #define SAVE_CONFIG_SIZE        0x00000800
   // Depend from properties_t size, should be aligned by FLASH_PAGESIZE
   #define SAVE_PROP_CONFIG_SIZE   0x00001800      // 6144 bytes
   // Save config_t and properties_t flash area (see flash7  : org = 0x08018000, len = 32k from *.ld settings)
   // Properties save area follow after config
   // len = SAVE_CONFIG_SIZE + SAVEAREA_MAX * SAVE_PROP_CONFIG_SIZE   0x00008000  32k
   #define SAVE_CONFIG_ADDR        0x08018000
   #define SAVE_PROP_CONFIG_ADDR   (SAVE_CONFIG_ADDR + SAVE_CONFIG_SIZE)
   #define SAVE_FULL_AREA_SIZE     (SAVE_CONFIG_SIZE + (SAVEAREA_MAX * SAVE_PROP_CONFIG_SIZE))
#endif

#define CONFIG_MAGIC               0x434f4e45 // 'CONF'

// ******************************

#define SAVE_S1P_FILE       1
#define SAVE_S2P_FILE       2
#define BACKUP_CONFIG_FILE  3
#define BACKUP_CAL_FILE     4
#define RESTORE_CONFIG_FILE 5
#define RESTORE_CAL_FILE    6
#define AUTO_SAVE_SECS      7

#ifdef __USE_SD_CARD__
   extern void sd_save_sparam_file(uint8_t data, bool show_messages);
	#ifdef __USE_SD_CARD_BACKUP_RESTORE_CONFIG_CAL__
   	extern void sd_restore_config(void);
   	extern void sd_restore_cal(void);
   	extern void sd_backup_config(void);
   	extern void sd_backup_cal(void);
	#endif
#endif

/*
 * main.c
 */

#define START_MIN                   800U                     // Minimum frequency set
#define STOP_MAX                    2700000000U              // Maximum frequency set

#define DEFAULT_FREQUENCY_THRESHOLD 300000100U               // Frequency threshold (max frequency for si5351, harmonic mode after)
#define FREQUENCY_OFFSET            8000                     // Frequency offset (sin_cos table in dsp.c generated for 6k, 8k, 10k, if change need create new table )

//#define USE_VARIABLE_OFFSET                                // Use real time build table (undef for use constant)
#define SPEED_OF_LIGHT              299792458                // Speed of light const
#define VNA_PI                      3.14159265358979323846   // pi const

// Optional sweep point
#if !defined(NANOVNA_F303)
   #define POINTS_SET_51             51                      // uncomment to allow using different number of sweep points
   #define POINTS_SET_101           101
   #define POINTS_COUNT             POINTS_SET_101           // Maximum sweep point count
#else
   #define POINTS_SET_51             51
   #define POINTS_SET_101           101
   #define POINTS_SET_201           201
   #define POINTS_COUNT             POINTS_SET_201           // Maximum sweep point count
#endif

extern float    measured[2][POINTS_COUNT][2];
extern uint32_t frequencies[POINTS_COUNT];

#define CAL_LOAD                0
#define CAL_OPEN                1
#define CAL_SHORT               2
#define CAL_THRU                3
#define CAL_ISOLN               4

#define CALSTAT_LOAD            (1u << 0)
#define CALSTAT_OPEN            (1u << 1)
#define CALSTAT_SHORT           (1u << 2)
#define CALSTAT_THRU            (1u << 3)
#define CALSTAT_ISOLN           (1u << 4)
#define CALSTAT_ES              (1u << 5)
#define CALSTAT_ER              (1u << 6)
#define CALSTAT_ET              (1u << 7)
#define CALSTAT_APPLY           (1u << 8)
#define CALSTAT_INTERPOLATED    (1u << 9)
#define CALSTAT_ED              CALSTAT_LOAD
#define CALSTAT_EX              CALSTAT_ISOLN

#define ETERM_ED                0 /* error term directivity */
#define ETERM_ES                1 /* error term source match */
#define ETERM_ER                2 /* error term refrection tracking */
#define ETERM_ET                3 /* error term transmission tracking */
#define ETERM_EX                4 /* error term isolation */

#define DOMAIN_MODE             (1u << 0)
#define DOMAIN_FREQ             (0u << 0)
#define DOMAIN_TIME             (1u << 0)
#define TD_FUNC                 (3u << 1)
#define TD_FUNC_BANDPASS        (0u << 1)
#define TD_FUNC_LOWPASS_IMPULSE (1u << 1)
#define TD_FUNC_LOWPASS_STEP    (2u << 1)
#define TD_WINDOW               (3u << 3)
#define TD_WINDOW_NORMAL        (0u << 3)
#define TD_WINDOW_MINIMUM       (1u << 3)
#define TD_WINDOW_MAXIMUM       (2u << 3)

#define FFT_SIZE                256

int shell_printf(const char *fmt, ...);

void cal_collect(int type);
void cal_done(void);

#define MAX_FREQ_TYPE           5
enum stimulus_type {ST_START = 0, ST_STOP, ST_CENTER, ST_SPAN, ST_CW};

void set_sweep_frequency(int type, uint32_t frequency);
uint32_t get_sweep_frequency(int type);
uint32_t get_bandwidth_frequency(uint16_t bw_freq);

extern uint32_t get_marker_frequency(int marker);

double my_atof(const char *p);

void toggle_sweep(void);
void load_default_properties(void);
int  load_properties(uint32_t id);
void set_sweep_points(uint16_t points);

#define SWEEP_ENABLE            0x01
#define SWEEP_ONCE              0x02
extern int8_t     sweep_mode;
extern const char *info_about[];

/*
 * dsp.c
 */
// 5ms @ 96kHz
// Define aic3204 source clock frequency (for 8MHz used fractional multiplier, and possible little phase error)
#define AUDIO_CLOCK_REF         ( 8000000U)
//#define AUDIO_CLOCK_REF       (10752000U)
// Disable AIC PLL clock, use input as CODEC_CLKIN
//#define AUDIO_CLOCK_REF       (86016000U)

// Define ADC sample rate
#define AUDIO_ADC_FREQ          96000
#define AUDIO_ADC_FREQ_k        96
//#define AUDIO_ADC_FREQ        48000
// Define sample count for one step measure
#define AUDIO_SAMPLES_COUNT     48
// Buffer contain left and right channel samples (need x2)
#define AUDIO_BUFFER_LEN        (AUDIO_SAMPLES_COUNT * 2)

#define FREQUENCY_IF_FREQ       12000
#define FREQUENCY_IF_FREQ_k     12

// Bandwidth depend from AUDIO_SAMPLES_COUNT and audio ADC frequency
// for AUDIO_SAMPLES_COUNT = 48 and ADC = 96kHz one measure give 96000 / 48 = 2000Hz
// define additional measure count
#define BANDWIDTH_2000          (  1 - 1)
#define BANDWIDTH_1000          (  2 - 1)
#define BANDWIDTH_333           (  6 - 1)
#define BANDWIDTH_100           ( 20 - 1)
#define BANDWIDTH_30            ( 66 - 1)
#define BANDWIDTH_10            (200 - 1)

#ifdef ENABLED_DUMP
   extern int16_t ref_buf[];
   extern int16_t samp_buf[];
#endif

void dsp_process(int16_t *src, size_t len);
void reset_dsp_accumerator(void);
void calculate_gamma(float *gamma);
void fetch_amplitude(float *gamma);
void fetch_amplitude_ref(float *gamma);
void generate_DSP_Table(int offset);

/*
 * tlv320aic3204.c
 */

void tlv320aic3204_init(void);
void tlv320aic3204_set_gain(uint8_t lgain, uint8_t rgain);
void tlv320aic3204_select(uint8_t channel);
void tlv320aic3204_write_reg(uint8_t page, uint8_t reg, uint8_t data);

/*
 * ui.c
 */

#define MENU_BUTTON_WIDTH        100   // Menu button width
#define NUM_INPUT_HEIGHT         32    // Height of numerical input field (at bottom)
#define KP_WIDTH                  (LCD_WIDTH / 4)                       // numeric keypad button width
#define KP_HEIGHT                 (((LCD_HEIGHT - OFFSETY) - NUM_INPUT_HEIGHT) / 4) // numeric keypad button height
#define KP_GET_X(posx)            ((posx) * KP_WIDTH)                   // numeric keypad left
#define KP_GET_Y(posy)            ((posy) * KP_HEIGHT)                  // numeric keypad top

enum { UI_NORMAL =  0, UI_MENU, UI_NUMERIC, UI_KEYPAD };

extern bool ui_menu_user_present;

extern void ui_init(void);
extern void ui_process(void);

// Irq operation process set
#define OP_NONE       0x00
#define OP_LEVER      0x01
#define OP_TOUCH      0x02
//#define OP_FREQCHANGE 0x04
extern volatile uint8_t operation_requested;

// lever_mode
enum lever_mode {LM_MARKER = 0, LM_SEARCH, LM_CENTER, LM_SPAN, LM_EDELAY};

// marker smith value format
enum marker_smithvalue {MS_LIN = 0, MS_LOG, MS_REIM, MS_RX, MS_RLC};

typedef struct uistat
{
  int8_t   digit;         // 0~5
  bool     digit_mode;
//  int8_t   current_trace; // 0..3 .. moved to config (now saveable)
  int32_t  value;         // for editing at numeric input area
  int32_t  original_value;
  uint8_t  lever_mode;
  uint8_t  marker_delta;
  uint8_t  marker_tracking;
} uistat_t;

extern uistat_t uistat;

void ui_init(void);
void ui_show(void);
void ui_hide(void);

void touch_start_watchdog(void);
void touch_position(int *x, int *y);
void handle_touch_interrupt(void);

// PA0 pin enables/disables the new HPF
#define HPF_ENABLE               palClearPad(GPIOA, GPIOA_HPF_CONTROL)
#define HPF_DISABLE              palSetPad(GPIOA, GPIOA_HPF_CONTROL)

// lever switch assignment
#define BIT_DOWN1                (1u << GPIOA_LEVER1)
#define BIT_PUSH                 (1u << GPIOA_PUSH)
#define BIT_UP1                  (1u << GPIOA_LEVER2)
#define READ_BUTTONS()           (palReadPort(GPIOA) & (BIT_DOWN1 | BIT_PUSH | BIT_UP1))

#define TOUCH_THRESHOLD_MIN     500
#define TOUCH_THRESHOLD_DEFAULT 2000

extern uint16_t touch_threshold;

void touch_cal_exec(void);
void touch_auto_cal_touch_threshold(void);
void touch_draw_test(void);
void enter_dfu(void);

/*
 * plot.c
 */

#ifndef NANOVNA_H4
   // NanoVNA-H
   // font 5x7
   extern const uint8_t x5x7_bits [];
   #define FONT_GET_DATA(ch)     (&x5x7_bits[ch * 7])
   #define FONT_GET_WIDTH(ch)    (8 - (x5x7_bits[ch * 7] & 7))
   #define FONT_MAX_WIDTH        7
   #define FONT_WIDTH            5
   #define FONT_GET_HEIGHT       7
   #define FONT_STR_HEIGHT       8
#else
   // NanoVNA-H4
   // font 7x11 font definition macros
   extern const uint8_t          x7x11b_bits [];
   #define FONT_GET_DATA(ch)    (&x7x11b_bits[ch * 11])
   #define FONT_GET_WIDTH(ch)   (8 - (x7x11b_bits[ch * 11] & 7))
   #define FONT_MAX_WIDTH        8
   #define FONT_WIDTH            7
   #define FONT_GET_HEIGHT        11
   #define FONT_STR_HEIGHT        11
#endif

// Offset of plot area
#ifndef NANOVNA_H4
   // NanoVNA-H
   #define OFFSETX            10
   #define OFFSETY            0
//   #define OFFSETY            (FONT_GET_HEIGHT * 2)
#else
   // NanoVNA-H4
   #define OFFSETX            15
   #define OFFSETY            0
#endif

#ifndef NANOVNA_H4
   // NanoVNA-H
   // WIDTH better be n*(POINTS_COUNT-1)
   #define WIDTH              300
   // HEIGHT = 8*GRIDY
   #define HEIGHT             232
#else
   // NanoVNA-H4
   // WIDTH better be n*(POINTS_COUNT-1)
   #define WIDTH              455
   // HEIGHT = 8*GRIDY
   #define HEIGHT             304
#endif

// number of horizontal screen grid lines
//#define NGRIDY              10
#define NGRIDY                8

#ifndef NANOVNA_H4
   // NanoVNA-H
   #define FREQUENCIES_XPOS1  OFFSETX
   #define FREQUENCIES_XPOS2  206
   #define FREQUENCIES_XPOS3  135
   #define FREQUENCIES_YPOS   (LCD_HEIGHT - FONT_GET_HEIGHT)
#else
   // NanoVNA-H4
   #define FREQUENCIES_XPOS1  OFFSETX
   #define FREQUENCIES_XPOS2  320
   #define FREQUENCIES_XPOS3  200
   #define FREQUENCIES_YPOS   (LCD_HEIGHT - FONT_GET_HEIGHT  - 1)
#endif

// GRIDX calculated depends from frequency span
//#define GRIDY 29
#define GRIDY                 (HEIGHT / NGRIDY)

//
#define CELLOFFSETX           5
#define AREA_WIDTH_NORMAL     (CELLOFFSETX + WIDTH + 1 + 4)
#define AREA_HEIGHT_NORMAL    (HEIGHT + 1)

// Smith/polar chart
#define P_CENTER_X            (CELLOFFSETX + (WIDTH / 2))
#define P_CENTER_Y            (HEIGHT / 2)
#define P_RADIUS              (P_CENTER_Y - (FONT_STR_HEIGHT * 2))

extern int16_t area_width;
extern int16_t area_height;

//extern const uint16_t numfont16x22[];
#define NUM_FONT_GET_DATA(ch) (&numfont16x22[ch * 22])
#define NUM_FONT_GET_WIDTH    16
#define NUM_FONT_GET_HEIGHT   22

#define S_DELTA               "\004"
#define S_DEGREE              "\037"
#define S_SARROW              "\030"
#define S_INFINITY            "\031"
#define S_LARROW              "\032"
#define S_RARROW              "\033"
#define S_PI                  "\034"
#define S_MICRO               "\035"
#define S_OHM                 "\036"

// trace
#ifndef NANOVNA_H4
   // NanoVNA-H
   #define MAX_TRACE_TYPE     12
   enum trace_type {TRC_LOGMAG = 0, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_Q, TRC_OFF};
#else
   // NanoVNA-H4
   #define MAX_TRACE_TYPE     13   // ERROR ??? ..  we only have 12 items here
   enum trace_type {TRC_LOGMAG = 0, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_Q, TRC_OFF};
#endif

// Mask for define rectangular plot
#define RECTANGULAR_GRID_MASK ((1 << TRC_LOGMAG) | (1 << TRC_PHASE) | (1 << TRC_DELAY) | (1 << TRC_LINEAR) | (1 << TRC_SWR) | (1 << TRC_REAL) | (1 << TRC_IMAG) | (1 << TRC_R) | (1 << TRC_X) | (1 << TRC_Q))

// LOGMAG: SCALE,  REFPOS,   REFVAL
// PHASE:  SCALE,  REFPOS,   REFVAL
// DELAY:  SCALE,  REFPOS,   REFVAL
// SMITH:  SCALE, <REFPOS>, <REFVAL>
// LINMAG: SCALE,  REFPOS,   REFVAL
// SWR:    SCALE,  REFPOS,   REFVAL
// Electrical Delay
// Phase

// config.freq_mode flags
#define FREQ_MODE_START_STOP    0x0
#define FREQ_MODE_CENTER_SPAN   0x1
#define FREQ_MODE_DOTTED_GRID   0x2

// config.flags
#define CONFIG_FLAGS_LC_MATCH   (1u << 0)

#define TRACES_MAX              4
#define MARKERS_MAX             4

typedef struct trace
{
  uint8_t enabled;
  uint8_t type;
  uint8_t channel;
  uint8_t reserved;
  float   scale;
  float   refpos;
} trace_t;

typedef struct config
{
   uint32_t magic;                          // marker value to help validate these config settings

   uint16_t dac_value;                      // output DAC value

   uint16_t grid_color;                     // screen grid colour
   uint16_t menu_normal_color;              // normal menu button background colour
   uint16_t menu_active_color;              // active menu button background colour
   uint16_t trace_color[TRACES_MAX];        // the colour of each trace

   int16_t  touch_cal[4];                   // touch screen calibration values

   uint32_t harmonic_freq_threshold;        // the freequency at which  we switch to harmonic mode

   uint16_t vbat_offset_mv;                 // battery voltage series diode millivolt drop level

   uint16_t bandwidth;                      // current bandwidth

   uint8_t  current_id;                     // currently selected calibration slot/memory
   int8_t   current_trace;                  // currently selected trace

   uint8_t  flags;                          // various flags

   uint8_t  _reserved1[1];                  // spare

   float    integrator_coeff;               // integrator level 0.0 to 1.0

   uint8_t  _reserved2[75];                 // spare

   uint32_t checksum;                       // checksum to valid these config settings
} config_t;                                 // sizeof(config_t) = 128

typedef struct properties
{
  uint32_t magic;                           // marker value to help validate these settings

  uint32_t _frequency0;                     // sweep start frequency
  uint32_t _frequency1;                     // sweep stop frequency

  uint16_t _sweep_points;                   // number of frequency sweep points

  uint16_t _cal_status;                     // state of each calibration step

  float    _cal_data[5][POINTS_COUNT][2];   // calibration data for open, short, load, isolation and through

  float    _electrical_delay;               // delay in picoseconds

  trace_t  _trace[TRACES_MAX];              // each screen trace details

  float    _velocity_factor;                // fraction (0.0 to 1.0

  uint8_t  _marker_index[MARKERS_MAX];      // the frequency index of each marker
  struct
  {
     uint8_t active:3;							  // 3 MSB's are the active marker index .. '7' = no active marker
     uint8_t spare:1;							  // spare bit
     uint8_t enabled:4;							  // 5 LSB's are marker enable flags (one per marker)
  } _marker_status;

  uint8_t  _domain_mode;                    // 0bxxxxxffm : where ff: TD_FUNC m: DOMAIN_MODE

  uint8_t  _marker_smith_format;            // display format used on the smith chart

  uint8_t  _freq_mode;                      // frequency mode flags

  uint32_t checksum;                        // checksum to valid these settings
} properties_t;    // with POINTS_COUNT = 101, sizeof(properties_t) == 4124 (need reduce size on 28 bytes to 4096 for more compact save slot size)

extern int8_t previous_marker;
extern config_t config;
extern properties_t *active_props;
extern properties_t current_props;

void set_trace_type(int t, int type);
void set_trace_channel(int t, int channel);
void set_trace_scale(int t, float scale);
void set_trace_refpos(int t, float refpos);
float get_trace_scale(int t);
float get_trace_refpos(int t);
const char *get_trace_typename(int t);

void set_electrical_delay(float picoseconds);
float get_electrical_delay(void);
float groupdelay_from_array(int i, float array[POINTS_COUNT][2]);

float pwr(const float *v);
float logmag(const float *v);
float linear(const float *v);
float phase(const float *v);
float groupdelay(const float *v, const float *w, float deltaf);
float swr(const float *v);
void impedance(const float *v, float *zr, float *zi, float z0);
//float resistance(const float *v, float z0);
//float reactance(const float *v, float z0);
float qualityfactor(const float *v);

void plot_init(void);
void update_grid(void);
void request_to_redraw_grid(void);
void redraw_frame(void);
//void redraw_all(void);
void request_to_draw_cells_behind_menu(void);
void request_to_draw_cells_behind_numeric_input(void);
void redraw_marker(int marker);
void plot_into_index(float measured[2][POINTS_COUNT][2]);
void force_set_markmap(void);
void draw_frequencies(void);
void draw_all(bool flush);

void draw_cal_status(void);

//void markmap_all_markers(void);

void marker_position(int m, int t, int *x, int *y);
int search_nearest_index(int x, int y, int t);
int get_marker_search(void);
void set_marker_search(int mode);
int marker_search(void);
int marker_search_left(int from);
int marker_search_right(int from);

// _request flag for update screen
#define REDRAW_CELLS                (1u << 0)
#define REDRAW_FREQUENCY            (1u << 1)
#define REDRAW_CAL_STATUS           (1u << 2)
#define REDRAW_MARKER               (1u << 3)
#define REDRAW_BATTERY              (1u << 4)
#define REDRAW_AREA                 (1u << 5)
extern volatile uint8_t redraw_request;

/*
 * ili9341.c
 */
// SPI bus revert byte order
//gggBBBbbRRRrrGGG
#define RGBHEX(hex)                  ( (((hex) & 0x001c00) << 3) | (((hex) & 0x0000f8) << 5) | (((hex)&0xf80000) >> 16) | (((hex) & 0x00e000) >> 13) )
#define RGB565(r, g, b)               ( (((uint16_t)(g) & 0b00011100) << 11) | (((uint16_t)(b) & 0b11111000) << 5) | ((uint16_t)(r) & 0b11111000) | (((uint16_t)(g) & 0b11100000) >> 5) )
#define RGB_RED(c)                  ( ((c) >> 0) & 0b11111000)
#define RGB_GRN(c)                  ((((c) << 5) & 0b11100000) | (((c) >> 11) & 0b00011100))
#define RGB_BLU(c)                  ( ((c) >> 5) & 0b11111000)

#ifndef NANOVNA_H4
   // NanoVNA-H
   #define LCD_WIDTH                320
   #define LCD_HEIGHT               240
#else
   // NanoVNA-H4
   #define LCD_WIDTH                480
   #define LCD_HEIGHT               320
#endif

// ***********

#define DEFAULT_FG_COLOR            RGB565(255, 255, 255)
#define DEFAULT_BG_COLOR            RGB565(  0,   0,   0)   // black background
//#define DEFAULT_FG_COLOR            RGB565(  0,   0,   0)
//#define DEFAULT_BG_COLOR            RGB565(255, 255, 255)   // white background

#define DEFAULT_GRID_COLOR          RGB565(128, 128, 128)

//#define DEFAULT_TRACE_1_COLOR       RGB565(255, 255,   0)	// yellow
//#define DEFAULT_TRACE_2_COLOR       RGB565(  0, 255, 255)	// cyan
//#define DEFAULT_TRACE_3_COLOR       RGB565(  0, 255,   0)	// green
//#define DEFAULT_TRACE_4_COLOR       RGB565(255,   0, 255)	// purple

#define DEFAULT_TRACE_1_COLOR       RGB565(248, 208, 144)	// browny orange
#define DEFAULT_TRACE_2_COLOR       RGB565(  0, 248, 248)	// cyan
#define DEFAULT_TRACE_3_COLOR       RGB565(  0, 248,   0)	// green
#define DEFAULT_TRACE_4_COLOR       RGB565(248, 160, 248)	// pinky purple

#define DEFAULT_NORMAL_BAT_COLOR    DEFAULT_FG_COLOR
#define DEFAULT_LOW_BAT_COLOR       RGB565(255, 100,  80)

#define DEFAULT_SPEC_SELECT_COLOR   RGB565(255, 128, 128)
#define DEFAULT_SPEC_INPUT_COLOR    RGB565(128, 255, 128)

#define DEFAULT_FINGER_COLOR        RGB565(200, 140,  68)

#define DEFAULT_BANDWIDTH_COLOR     RGB565( 64, 255,  64)

// ***********
// menu/button colours

#if defined(MENU_THEME_BLUE)

   // blue
   #define DEFAULT_MENU_COLOR             RGB565( 90, 160, 200)

   #define DEFAULT_MENU_BORDER_LO_COLOR   RGB565( 50, 110, 170)
   #define DEFAULT_MENU_BORDER_HI_COLOR   RGB565(100, 170, 220)

   #define DEFAULT_MENU_TEXT_COLOR        RGB565(255, 255, 255)
   #define DEFAULT_MENU_TEXT_DIM_COLOR    RGB565(220, 220, 255)

   #define DEFAULT_MENU_SELECT_COLOR      RGB565(255,  24, 180)
   #define DEFAULT_MENU_SELECT_TEXT_COLOR RGB565(255, 255, 255)

//   #define DEFAULT_MENU_ACTIVE_COLOR      RGB565(255, 200, 255)
   #define DEFAULT_MENU_ACTIVE_COLOR      RGB565(255, 210, 255)
   #define DEFAULT_MENU_ACTIVE_TEXT_COLOR RGB565(  0,   0,   0)

   #define DEFAULT_NUMERIC_LABEL_COLOR    RGB565(255, 255,   0)

#elif defined(MENU_THEME_WHITE)

   // white
   #define DEFAULT_MENU_COLOR             RGB565(240, 240, 240)

   #define DEFAULT_MENU_BORDER_LO_COLOR   RGB565(220, 220, 220)
   #define DEFAULT_MENU_BORDER_HI_COLOR   RGB565(255, 255, 255)

   #define DEFAULT_MENU_TEXT_COLOR        RGB565(  0,   0,   0)
   #define DEFAULT_MENU_TEXT_DIM_COLOR    RGB565(128, 128, 128)

   #define DEFAULT_MENU_SELECT_COLOR      RGB565(255,  24, 180)
   #define DEFAULT_MENU_SELECT_TEXT_COLOR RGB565(255, 255, 255)

   #define DEFAULT_MENU_ACTIVE_COLOR      RGB565(128, 200, 128)
   #define DEFAULT_MENU_ACTIVE_TEXT_COLOR RGB565(255, 255, 255)

   #define DEFAULT_NUMERIC_LABEL_COLOR    RGB565(128, 128, 255)

#else

   // grey
   #define DEFAULT_MENU_COLOR             RGB565(170, 170, 170)

   #define DEFAULT_MENU_BORDER_LO_COLOR   RGB565(160, 160, 160)
   #define DEFAULT_MENU_BORDER_HI_COLOR   RGB565(180, 180, 180)

   #define DEFAULT_MENU_TEXT_COLOR        RGB565(255, 255, 255)
   #define DEFAULT_MENU_TEXT_DIM_COLOR    RGB565(220, 220, 255)

   #define DEFAULT_MENU_SELECT_COLOR      RGB565(255,  24, 180)
   #define DEFAULT_MENU_SELECT_TEXT_COLOR RGB565(255, 255, 255)

   #define DEFAULT_MENU_ACTIVE_COLOR      RGB565(255, 200, 255)
   #define DEFAULT_MENU_ACTIVE_TEXT_COLOR RGB565(  0,   0,   0)

   #define DEFAULT_NUMERIC_LABEL_COLOR    RGB565(255, 255,   0)

#endif

// ***********

extern uint16_t foreground_color;
extern uint16_t background_color;

// ***********

void ili9341_init(void);
void ili9341_test(int mode);
void ili9341_bulk(int x, int y, int w, int h);
void ili9341_fill(int x, int y, int w, int h, uint16_t color);
void ili9341_set_foreground(uint16_t fg);
void ili9341_set_background(uint16_t fg);
void ili9341_clear_screen(void);
void blit8BitWidthBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *bitmap);
void ili9341_drawchar(uint8_t ch, int x, int y);
void ili9341_drawstring(const char *str, int x, int y, bool fixed_pitch);
void ili9341_drawstringV270(const char *str, int x, int y, bool fixed_pitch);
void ili9341_drawstringV90(const char *str, int x, int y, bool fixed_pitch);
int  ili9341_drawchar_size(uint8_t ch, int x, int y, uint8_t size, bool fixed_pitch);
void ili9341_drawstring_size(const char *str, int x, int y, uint8_t size, bool fixed_pitch);
void ili9341_drawfont(uint8_t ch, int x, int y);
void ili9341_read_memory(int x, int y, int w, int h, int len, uint16_t* out);
void ili9341_line(int x0, int y0, int x1, int y1);
void show_version(void);

/*
 * rtc.c
 */
#ifdef __USE_RTC__
   #define RTC_START_YEAR          2000

   #define RTC_DR_YEAR(dr)         (((dr) >> 16) & 0xFF)
   #define RTC_DR_MONTH(dr)        (((dr) >>  8) & 0xFF)
   #define RTC_DR_DAY(dr)          (((dr) >>  0) & 0xFF)

   #define RTC_TR_HOUR(dr)         (((tr) >> 16) & 0xFF)
   #define RTC_TR_MIN(dr)          (((tr) >>  8) & 0xFF)
   #define RTC_TR_SEC(dr)          (((tr) >>  0) & 0xFF)

   // Init RTC
   void rtc_init(void);
   // Then read time and date TR should read first, after DR !!!
   // Get RTC time as bcd structure in 0x00HHMMSS
   #define rtc_get_tr_bcd() (RTC->TR & 0x007F7F7F)
   // Get RTC date as bcd structure in 0x00YYMMDD (remove day of week information!!!!)
   #define rtc_get_dr_bcd() (RTC->DR & 0x00FF1F3F)
   // read TR as 0x00HHMMSS in bin (TR should be read first for sync)
   uint32_t rtc_get_tr_bin(void);
   // read DR as 0x00YYMMDD in bin (DR should be read second)
   uint32_t rtc_get_dr_bin(void);
   // Read time in FAT filesystem format
   uint32_t rtc_get_FAT(void);
   // Write date and time (need in bcd format!!!)
   void rtc_set_time(uint32_t dr, uint32_t tr);
#endif

#define frequency0             current_props._frequency0
#define frequency1             current_props._frequency1
#define sweep_points           current_props._sweep_points
#define cal_status             current_props._cal_status
#define cal_data               active_props->_cal_data
#define electrical_delay       current_props._electrical_delay

#define trace                  current_props._trace

#define disable_marker(m)      { current_props._marker_status.enabled &= ~(1u << (m));}
#define enable_marker(m)       { current_props._marker_status.enabled |=  (1u << (m));}
#define marker_enabled(m)      ((current_props._marker_status.enabled >> (m)) & 1u)
#define active_marker            current_props._marker_status.active
#define set_active_marker(m)   { current_props._marker_status.active = (m & 0x07);}
#define set_no_active_marker   { current_props._marker_status.active = 7;}
#define marker_index(m)          current_props._marker_index[m]
#define set_marker_index(m, i) { current_props._marker_index[m] = i;}

#define domain_mode            current_props._domain_mode
#define velocity_factor        current_props._velocity_factor
#define marker_smith_format    current_props._marker_smith_format
#define freq_mode              current_props._freq_mode

#define FREQ_IS_STARTSTOP()    (!(freq_mode & FREQ_MODE_CENTER_SPAN))
#define FREQ_IS_CENTERSPAN()   (freq_mode & FREQ_MODE_CENTER_SPAN)
#define FREQ_IS_CW()           (frequency0 == frequency1)

int caldata_save(uint32_t id, bool fixed);
int caldata_recall(uint32_t id, bool fixed);
const properties_t *caldata_ref(uint32_t id);

void config_reset(void);
int config_save(void);
int config_recall(void);

void clear_all_config_prop_data(void);

/*
 * adc.c
 */
#define ADC_TOUCH_X  ADC_CHSELR_CHSEL6
#define ADC_TOUCH_Y  ADC_CHSELR_CHSEL7

void adc_init(void);
uint16_t adc_single_read(uint32_t chsel);
void adc_start_analog_watchdogd(uint32_t chsel, uint16_t threshold);
void adc_stop(void);
int16_t adc_vbat_read(void);

/*
 * misclinous
 */
void insert_sort(uint16_t in[], uint16_t out[], unsigned int size);

int plot_printf(char *str, int, const char *fmt, ...);
#define PULSE do { palClearPad(GPIOC, GPIOC_LED); palSetPad(GPIOC, GPIOC_LED); } while(0)

// Speed profile definition
#define START_PROFILE     systime_t time = chVTGetSystemTimeX();
#define STOP_PROFILE      {char string_buf[12]; plot_printf(string_buf, sizeof string_buf, "T:%06d", chVTGetSystemTimeX() - time); ili9341_drawstringV(string_buf, 1, 60); }
// Macros for convert define value to string
#define STR1(x)           #x
#define define_to_STR(x)  STR1(x)

#endif
