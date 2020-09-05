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

#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"
#ifdef USE_LC_MATCHING
   #include "lc_matching.h"
#endif

//#define SHOW_FINGER                // show the finger whilst the user in touching the screen .. comment out to not show it

uistat_t uistat =
{
   digit:           6,
   //current_trace:   0,   // moved to config
   lever_mode:      LM_MARKER,
   marker_delta:    FALSE,
   marker_tracking: FALSE,
};

#define INTEGRATOR_MENU_DATA      2
#define LC_MATCHING_MENU_DATA     3

#define XY_TOUCH_SCALE            16

// Key names
#define KP_0                      0
#define KP_1                      1
#define KP_2                      2
#define KP_3                      3
#define KP_4                      4
#define KP_5                      5
#define KP_6                      6
#define KP_7                      7
#define KP_8                      8
#define KP_9                      9
#define KP_PERIOD                10
#define KP_MINUS                 11
#define KP_X1                    12
#define KP_K                     13
#define KP_M                     14
#define KP_G                     15
#define KP_BS                    16
#define KP_INF                   17
#define KP_DB                    18
#define KP_PLUSMINUS             19
#define KP_KEYPAD                20
#define KP_N                     21
#define KP_P                     22
#define KP_NONE                  23	// must be the last one

// button_status
#define NO_EVENT                 0
#define EVT_BUTTON_SINGLE_CLICK  (1u << 0)
#define EVT_BUTTON_DOUBLE_CLICK  (1u << 1)
#define EVT_BUTTON_DOWN_LONG     (1u << 2)
#define EVT_UP                   (1u << 4)
#define EVT_DOWN                 (1u << 5)
#define EVT_REPEAT               (1u << 6)

#define BUTTON_DOWN_LONG_TICKS   3000   // 300ms
#define BUTTON_DOUBLE_TICKS      2500   // 250ms
#define BUTTON_REPEAT_TICKS      100    //  10ms
#define BUTTON_DEBOUNCE_TICKS    400    //  40ms
/*
#define GPT_TIMER_MS             5       // timer int period

#define BUTTON_DEBOUNCE_MS       70
#define BUTTON_DOWN_LONG_MS      500
#define BUTTON_DOUBLE_MS         250
#define BUTTON_REPEAT_MS         20
*/

static uint16_t      last_button             = 0;
static systime_t     last_button_down_tick   = 0;
static systime_t     last_button_repeat_tick = 0;
static volatile bool inhibit_until_release   = false;

//static volatile uint32_t timer_tick               = 0;
//static volatile uint32_t timer_tick_button        = 0;
//static volatile uint32_t timer_tick_repeat_button = 0;
//static volatile uint8_t  button_status            = 0;

//static volatile bool op_lever = false;
//static volatile bool op_touch = false;

volatile uint8_t operation_requested = OP_NONE;

#define NUMINPUT_LEN             10

int8_t previous_marker           = -1;

enum { KM_NONE   = -1, KM_START, KM_STOP, KM_CENTER, KM_SPAN, KM_CW, KM_SCALE, KM_REFPOS, KM_EDELAY, KM_VELOCITY_FACTOR, KM_SCALEDELAY, KM_AUTO_SAVE_SECS };

bool           ui_menu_user_present     = false;
static uint8_t ui_mode                  = UI_NORMAL;
static  int8_t keypad_mode              = KM_NONE;
static uint8_t keypads_last_index       = 0;
static char    kp_buf[NUMINPUT_LEN + 1] = {0};
static int8_t  kp_index                 = 0;
static uint8_t menu_current_level       = 0;
static int8_t  selection                = 0;

// Touch screen
#define EVT_TOUCH_NONE     0
#define EVT_TOUCH_DOWN     1
#define EVT_TOUCH_PRESSED  2
#define EVT_TOUCH_RELEASED 3

uint16_t        touch_threshold   = TOUCH_THRESHOLD_DEFAULT;
static uint16_t last_touch_status = EVT_TOUCH_NONE;
static int16_t  last_touch_x;
static int16_t  last_touch_y;

//int16_t touch_cal[4] = { 1000, 1000, 10 * XY_TOUCH_SCALE, 12 * XY_TOUCH_SCALE };
//int16_t touch_cal[4] = { 620, 600, 130, 180 };

//int awd_count;
//int touch_x, touch_y;

#define KP_CONTINUE          0
#define KP_DONE              1
#define KP_CANCEL            2

// used for x.y median touch filtering
#define XY_FILTER_BUF_SIZE   9      // the higher this value the more/longer filtering is done
uint16_t touch_prev_x        = 0;
uint16_t touch_prev_y        = 0;
uint16_t touch_dx            = 0;   // for debug
uint16_t touch_dy            = 0;   // for debug
uint16_t touch_pressure      = 0;
int      touch_count         = 0;
int      xy_filter_buf_index = 0;
struct
{
   uint16_t x[XY_FILTER_BUF_SIZE];
   uint16_t y[XY_FILTER_BUF_SIZE];
} xy_filter_buf;
struct
{
   uint16_t x[XY_FILTER_BUF_SIZE];
   uint16_t y[XY_FILTER_BUF_SIZE];
} xy_filter_buf_sorted;

static void ui_mode_normal(void);
static void ui_mode_menu(void);
static void ui_mode_numeric(int _keypad_mode);
static void ui_mode_keypad(int _keypad_mode);
static void draw_menu(void);
static void leave_ui_mode(void);
static void erase_menu_buttons(void);
static void ui_process_keypad(void);
static void ui_process_numeric(void);

static uint8_t save_cal_index = 0;

// Set structure align as WORD (save flash memory)
#pragma pack(push, 2)
typedef struct
{
   uint8_t    type;
   uint8_t    data;
   char       *label;
   const void *reference;
} menuitem_t;
#pragma pack(pop)

static void menu_move_back(bool leave_ui, bool redraw_ui);
static void menu_push_submenu(const menuitem_t *submenu);

// **********************************************************

void insert_sort(uint16_t in[], uint16_t out[], unsigned int size)
{
   unsigned int i;

   if (out != in)
   {	// copy input to output buffer
   	for (i = 0; i < size; i++)
   		out[i] = in[i];
   }

   // sort in order of increasing value
   for (i = 1; i < size; i++)
   {
      unsigned int j;
      const uint16_t value = out[i];
      for (j = i; j >= 1 && value < out[j - 1]; j--)
         out[j] = out[j - 1];
      out[j] = value;
   }
}

uint16_t median_x(void)
{
   insert_sort(xy_filter_buf.x, xy_filter_buf_sorted.x, XY_FILTER_BUF_SIZE);
   #if (XY_FILTER_BUF_SIZE & 1)
      return xy_filter_buf_sorted.x[XY_FILTER_BUF_SIZE / 2];   // exact center
   #else
      return (xy_filter_buf_sorted.x[(XY_FILTER_BUF_SIZE / 2) + 0] + xy_filter_buf_sorted.x[(XY_FILTER_BUF_SIZE / 2) + 1]) / 2;
   #endif
}

uint16_t median_y(void)
{
   insert_sort(xy_filter_buf.y, xy_filter_buf_sorted.y, XY_FILTER_BUF_SIZE);
   #if (XY_FILTER_BUF_SIZE & 1)
      return xy_filter_buf_sorted.y[XY_FILTER_BUF_SIZE / 2];   // exact center
   #else
      return (xy_filter_buf_sorted.y[(XY_FILTER_BUF_SIZE / 2) + 0] + xy_filter_buf_sorted.y[(XY_FILTER_BUF_SIZE / 2) + 1]) / 2;
   #endif
}

static uint8_t btn_check(void)
{
   systime_t tick;
   uint8_t cur_button;
   uint8_t button_set;
   uint8_t old_button = last_button;

   const bool button_was_pressed = last_button ? true : false;

   // debounce input
   while (TRUE)
   {
      tick = chVTGetSystemTimeX();

      cur_button = READ_BUTTONS();
      button_set = (old_button ^ cur_button) & cur_button;

      if (old_button != cur_button)
      {
         button_set            = (old_button ^ cur_button) & cur_button;
         old_button            = cur_button;
         last_button_down_tick = tick;
      }

      if ((tick - last_button_down_tick) >= BUTTON_DEBOUNCE_TICKS)
         break;

      chThdSleepMilliseconds(1);
   }

   // Detect only changed and pressed buttons
   button_set            = (last_button ^ cur_button) & cur_button;
// last_button_down_tick = tick;
   last_button           = cur_button;

   uint8_t status = 0;
   if (button_set & BIT_PUSH)  status |= EVT_BUTTON_SINGLE_CLICK;
   if (button_set & BIT_UP1)   status |= EVT_UP;
   if (button_set & BIT_DOWN1) status |= EVT_DOWN;

   if (status)
      ui_menu_user_present = true;
   else
   if (button_was_pressed)
      ui_menu_user_present = false;

   return status;
}

static uint8_t btn_wait_release(void)
{
   while (TRUE)
   {
      const systime_t tick = chVTGetSystemTime();
      const systime_t dt   = tick - last_button_down_tick;

      const uint8_t cur_button = READ_BUTTONS();
      const uint8_t changed    = last_button ^ cur_button;

//    const bool button_was_pressed = last_button ? true : false;

      chThdSleepMilliseconds(1);

      if (dt >= BUTTON_DOWN_LONG_TICKS && (cur_button & BIT_PUSH))
      {
         ui_menu_user_present = false;
         return EVT_BUTTON_DOWN_LONG;
      }

      if (changed & BIT_PUSH) // release
      {
         ui_menu_user_present = false;
         return EVT_BUTTON_SINGLE_CLICK;
      }

      if (changed)
      {   // finished
         last_button           = cur_button;
         last_button_down_tick = tick;

         ui_menu_user_present = false;
         return 0;
      }

      if (dt > BUTTON_DOWN_LONG_TICKS && tick > last_button_repeat_tick)
      {
         uint8_t status = 0;
         if (cur_button & BIT_DOWN1) status |= EVT_DOWN | EVT_REPEAT;
         if (cur_button & BIT_UP1)   status |= EVT_UP   | EVT_REPEAT;
         last_button_repeat_tick = tick + BUTTON_REPEAT_TICKS;

         if (!status)
            ui_menu_user_present = false;

         return status;
      }
   }
}

static uint16_t touch_measure_y(void)
{
   uint16_t v;

   // open Y line
   palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_PULLDOWN);
   palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_PULLDOWN);

   // drive low to high on X line
   palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL);
   palClearPad(GPIOB, 0);
   palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL);
   palSetPad(GPIOA, 6);

   chThdSleepMilliseconds(1);
   v = adc_single_read(ADC_TOUCH_Y);

   //chThdSleepMilliseconds(1);
   //v += adc_single_read(ADC1, ADC_TOUCH_Y);

   return v;
}

static uint16_t touch_measure_x(void)
{
   uint16_t v;

   // open X line
   palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_PULLDOWN);
   palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_PULLDOWN);

   // drive low to high on Y line
   palSetPadMode(GPIOB, 1, PAL_MODE_OUTPUT_PUSHPULL);
   palSetPad(GPIOB, 1);
   palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL);
   palClearPad(GPIOA, 7);

   chThdSleepMilliseconds(1);
   v = adc_single_read(ADC_TOUCH_X);

   //chThdSleepMilliseconds(1);
   //v += adc_single_read(ADC1, ADC_TOUCH_X);

   return v;
}

void touch_prepare_sense(void)
{
   // open Y line
   palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_PULLDOWN);
   palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_PULLDOWN);

   // force high X line
   palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL);
   palSetPad(GPIOB, 0);
   palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL);
   palSetPad(GPIOA, 6);
}

void touch_start_watchdog(void)
{
   touch_prepare_sense();
   adc_start_analog_watchdogd(ADC_TOUCH_Y, touch_threshold);
}

void touch_position(int *x, int *y)
{
   *x = ((last_touch_x - config.touch_cal[0]) * XY_TOUCH_SCALE) / config.touch_cal[2];
   *y = ((last_touch_y - config.touch_cal[1]) * XY_TOUCH_SCALE) / config.touch_cal[3];
}

static int touch_status(void)
{
   adc_stop();

   touch_prepare_sense();

   chThdSleepMilliseconds(2);

   touch_pressure = adc_single_read(ADC_TOUCH_Y);

   if (touch_pressure >= touch_threshold)
   {   // touching
      const uint16_t x = touch_measure_x();
      const uint16_t y = touch_measure_y();

      // save the new x/y into their circular buffers
      xy_filter_buf.x[xy_filter_buf_index] = x;
      xy_filter_buf.y[xy_filter_buf_index] = y;
      if (++xy_filter_buf_index >= XY_FILTER_BUF_SIZE)
         xy_filter_buf_index = 0;

      // average filter
//    uint16_t filt_x = average_x();
//    uint16_t filt_y = average_y();

      // median filter
      uint16_t filt_x = median_x();
      uint16_t filt_y = median_y();

      // movement since previous x/y
      touch_dx = abs((int)touch_prev_x - filt_x);
      touch_dy = abs((int)touch_prev_y - filt_y);

      if (++touch_count >= XY_FILTER_BUF_SIZE)
      {   // we're in touch down mode
         touch_count = XY_FILTER_BUF_SIZE;

         const int threshold = 40 * XY_TOUCH_SCALE;
//       if (touch_dx >= threshold || touch_dy >= threshold)
         if (abs((int)x - filt_x) >= threshold || abs((int)y - filt_y) >= threshold)
         {   // x/y seems a bit too noisy
            filt_x = last_touch_x;
            filt_y = last_touch_y;
         }
      }
      else
      {  // not currently in touch down mode
         // we want a stable x/y before deciding on a new touch down
         int i;

         const int threshold = 20 * XY_TOUCH_SCALE;   // we want the touch position to remain small
         int k = xy_filter_buf_index;
         for (i = 0; i < touch_count; i++)
         {
            if (--k < 0)
               k += XY_FILTER_BUF_SIZE;
            const uint16_t xx = xy_filter_buf.x[k];
            const uint16_t yy = xy_filter_buf.y[k];
            if (abs((int)x - xx) >= threshold || abs((int)y - yy) >= threshold || abs((int)filt_x - xx) >= threshold || abs((int)filt_y - yy) >= threshold)
            {   // x/y seems noisy .. hold off on the touch down until the x/y settles
               if (--touch_count < 0)
                  touch_count = 0;
            }
         }
      }

      last_touch_x = filt_x;
      last_touch_y = filt_y;

      touch_prev_x = filt_x;
      touch_prev_y = filt_y;
   }
   else
   {   // not touching
      touch_count = 0;
   }

   return (touch_count >= XY_FILTER_BUF_SIZE) ? 1 : 0;
}

static uint16_t touch_check(void)
{
   const int stat = touch_status();
   if (stat)
   {
//    chThdSleepMilliseconds(10);
//    touch_prepare_sense();
   }

   if (last_touch_status != stat)
   {
      last_touch_status = stat;
      return stat ? EVT_TOUCH_PRESSED : EVT_TOUCH_RELEASED;
   }

   return stat ? EVT_TOUCH_DOWN : EVT_TOUCH_NONE;
}

static inline void touch_wait_pressed(void)
{
   while (TRUE)
   {
      const uint16_t status = touch_check();
      if (status == EVT_TOUCH_DOWN || status == EVT_TOUCH_PRESSED)
         break;
   }
}

static inline void touch_wait_release(void)
{
   while (TRUE)
   {
      const uint16_t status = touch_check();
      if (status == EVT_TOUCH_NONE || status == EVT_TOUCH_RELEASED)
         break;
   }
}

void touch_cal_exec(void)
{
   int x1, x2;
   int y1, y2;

   adc_stop();

   if (touch_check())
      touch_wait_release();

   adc_stop();

   ili9341_set_foreground(DEFAULT_FG_COLOR);
   ili9341_set_background(DEFAULT_BG_COLOR);
   ili9341_clear_screen();

   ili9341_line(0, 0,  0, 32);
   ili9341_line(0, 0, 32,  0);
   ili9341_drawstring("TOUCH UPPER LEFT", 10, 10, false);

   touch_wait_pressed();
   touch_wait_release();

   x1 = last_touch_x;
   y1 = last_touch_y;

   ili9341_clear_screen();
   ili9341_line(LCD_WIDTH - 1, LCD_HEIGHT - 1, LCD_WIDTH -  1, LCD_HEIGHT - 32);
   ili9341_line(LCD_WIDTH - 1, LCD_HEIGHT - 1, LCD_WIDTH - 32, LCD_HEIGHT - 1);
   ili9341_drawstring("TOUCH LOWER RIGHT", LCD_WIDTH - 17 * (FONT_WIDTH) - 10, LCD_HEIGHT - FONT_GET_HEIGHT - 10, false);

   touch_wait_pressed();
   touch_wait_release();

   x2 = last_touch_x;
   y2 = last_touch_y;

   config.touch_cal[0] = x1;
   config.touch_cal[1] = y1;
   config.touch_cal[2] = ((x2 - x1) * XY_TOUCH_SCALE) / LCD_WIDTH;
   config.touch_cal[3] = ((y2 - y1) * XY_TOUCH_SCALE) / LCD_HEIGHT;

   touch_start_watchdog();
}

void touch_draw_test(void)
{
   int x0 = 0;
   int y0 = 0;
   char buf[32];

   adc_stop();

   memset(&xy_filter_buf, 0, sizeof(xy_filter_buf));
   touch_pressure      = 0;
   touch_count         = 0;
   xy_filter_buf_index = 0;

   ili9341_set_foreground(DEFAULT_FG_COLOR);
   ili9341_set_background(DEFAULT_BG_COLOR);
   ili9341_clear_screen();

   ili9341_drawstring_size("Press button to exit       ", OFFSETX, LCD_HEIGHT - (FONT_GET_HEIGHT * 2), 2, false);

   do
   {
      int x1;
      int y1;
      const uint16_t tc = touch_check();
      touch_position(&x1, &y1);

      switch (tc)
      {
         case EVT_TOUCH_PRESSED:      // just pressed
            ili9341_clear_screen();
            ili9341_fill(x0 - 5, y0 - 5, 11, 11, RGB565(0, 255, 0));   // start blob

         case EVT_TOUCH_DOWN:         // pressed
            ili9341_line(x0, y0, x1, y1);
            plot_printf(buf, sizeof(buf), "%3d %3d %4u %4u %3u %3u    ", x1, y1, touch_pressure, touch_threshold, touch_dx, touch_dy);
            ili9341_drawstring_size(buf, OFFSETX, LCD_HEIGHT - (FONT_GET_HEIGHT * 2), 2, false);
            break;

         case EVT_TOUCH_RELEASED:   // just released
            ili9341_fill(x0 - 5, y0 - 5, 11, 11, RGB565(255, 0, 0));   // stop blob
            ili9341_drawstring_size("Press button to exit           ", OFFSETX, LCD_HEIGHT - (FONT_GET_HEIGHT * 2), 2, false);
         default:
            break;
      }

      x0 = x1;
      y0 = y1;

   } while (!(btn_check() & EVT_BUTTON_SINGLE_CLICK));   // stay in loop until the push button is pressed

   // wait for button to be released
   btn_wait_release();

   touch_start_watchdog();
}

void show_version(void)
{
   const int line_spacing = 3;
   const int x = 5;
   int y = 5;
   int counter = 0;

   adc_stop();

   ili9341_set_foreground(DEFAULT_FG_COLOR);
   ili9341_set_background(DEFAULT_BG_COLOR);
   ili9341_clear_screen();

   ili9341_drawstring_size(BOARD_NAME, x, y, 3, false);
   y += (FONT_GET_HEIGHT * 3) + line_spacing;

   {
      uint16_t shift = 0b0000000000100000;
      int i = 1;
      while (info_about[i])
      {
         do {
            shift >>= 1;
            y += line_spacing;
         } while (shift & 1);
         ili9341_drawstring(info_about[i++], x, y += FONT_STR_HEIGHT, false);
      }
   }
   y += FONT_STR_HEIGHT + (line_spacing * 2);

   while (1)
   {
      char buffer[38];

      chThdSleepMilliseconds(1);

      if (btn_check() & EVT_BUTTON_SINGLE_CLICK)
      {
         btn_wait_release();
         break;
      }
      if (touch_check())
      {
         touch_wait_release();
         break;
      }

      const int16_t bat_mv = adc_vbat_read();

      if (--counter > 0)
         continue;
      counter = 150;

      #if 1 // show battery state
         ili9341_set_foreground(DEFAULT_FG_COLOR);
         plot_printf(buffer, sizeof(buffer), "vbat_offset   %u mV", config.vbat_offset_mv);
         ili9341_drawstring(buffer, x, y + (0 * (FONT_STR_HEIGHT + line_spacing)), false);
         plot_printf(buffer, sizeof(buffer), "vbat_top       %u mV", DEFAULT_BATTERY_TOP_MV);
         ili9341_drawstring(buffer, x, y + (1 * (FONT_STR_HEIGHT + line_spacing)), false);

         plot_printf(buffer, sizeof(buffer), "%d.%03dV %3d%%", bat_mv / 1000, bat_mv % 1000, ((bat_mv - DEFAULT_BATTERY_BOTTOM_MV) * 100) / (DEFAULT_BATTERY_TOP_MV - DEFAULT_BATTERY_BOTTOM_MV));
         ili9341_set_foreground(bat_mv < DEFAULT_BATTERY_WARNING_MV ? DEFAULT_LOW_BAT_COLOR : DEFAULT_NORMAL_BAT_COLOR);
         ili9341_drawstring_size(buffer, x, LCD_HEIGHT - (2 * FONT_STR_HEIGHT), 2, false);
      #endif

      #ifdef __USE_RTC__
         const uint32_t tr = rtc_get_tr_bin(); // TR read first
         const uint32_t dr = rtc_get_dr_bin(); // DR read second
         plot_printf(buffer, sizeof(buffer), "%04d.%02d.%02d %02d:%02d:%02d    ",
				RTC_START_YEAR + RTC_DR_YEAR(dr),
            RTC_DR_MONTH(dr),
            RTC_DR_DAY(dr),
            RTC_TR_HOUR(dr),
            RTC_TR_MIN(dr),
            RTC_TR_SEC(dr));
         ili9341_set_foreground(DEFAULT_FG_COLOR);
         ili9341_drawstring_size(buffer, x, LCD_HEIGHT - (5 * FONT_STR_HEIGHT), 2, false);
      #endif
   }

   redraw_frame();
   draw_cal_status();
   request_to_redraw_grid();

   touch_start_watchdog();
}

void enter_dfu(void)
{
   adc_stop();

   int x = 10;
   int y = 20;

   ili9341_set_foreground(DEFAULT_FG_COLOR);
   ili9341_set_background(DEFAULT_BG_COLOR);
   ili9341_clear_screen();

   ili9341_drawstring_size("Device Firmware Update Mode.", x, y += FONT_STR_HEIGHT * 3, 2, false);

   ili9341_drawstring_size("To exit DFU mode, please",     x, y += FONT_STR_HEIGHT * 6, 2, false);
   ili9341_drawstring_size("reset this device yourself.",  x, y += FONT_STR_HEIGHT * 3, 2, false);

   // see __early_init in ./NANOVNA_STM32_F072/board.c
   *((unsigned long *)BOOT_FROM_SYTEM_MEMORY_MAGIC_ADDRESS) = BOOT_FROM_SYTEM_MEMORY_MAGIC;
   NVIC_SystemReset();
}

static void select_lever_mode(int mode)
{
   if (uistat.lever_mode != mode)
   {
      uistat.lever_mode = mode;
      redraw_request |= REDRAW_FREQUENCY | REDRAW_MARKER;
   }
}

typedef void (*menuaction_cb_t)(int item, uint8_t data);

const char * cal_str[] = {"LOAD", "OPEN", "SHORT", "THRU", "ISOLN"};

static void menu_calop_cb(int item, uint8_t data)
{
   char str[20];

// display an on-screen message to let the user know what's happening
   if (data < 5)
   {
      plot_printf(str, sizeof(str), "Sampling %s ..", cal_str[data]);
      ili9341_set_background(DEFAULT_BG_COLOR);
      ili9341_set_foreground(DEFAULT_FG_COLOR);
      ili9341_drawstring_size(str, OFFSETX + 12, LCD_HEIGHT / 5, 2, false);
   }

   cal_collect(data);

// redraw_frame();
   request_to_redraw_grid();
   draw_cal_status();

   selection = item + 1;

   draw_menu();
}

static void menu_caldone_cb(int item, uint8_t data)
{
   (void)item;
   (void)data;
// extern const menuitem_t menu_save_cal[];

   cal_done();
   draw_cal_status();

   menu_move_back(false, true);
//   menu_push_submenu(menu_save_cal);
}

static void menu_cal2_cb(int item, uint8_t data)
{
   (void)data;

   switch (item)
   {
      case 2: // RESET
         cal_status = 0;
         break;
      case 3: // CORRECTION
         // toggle applying correction
         cal_status ^= CALSTAT_APPLY;
         break;
   }
   draw_menu();
   draw_cal_status();
}

static void menu_recall_cb(int item, uint8_t data)
{
   (void)item;

   load_properties(data);

//   redraw_frame();
   request_to_redraw_grid();
   draw_cal_status();
   draw_menu();
}

static void menu_touch_cb(int item, uint8_t data)
{
   (void)data;

   switch (item)
   {
      case 0:
         touch_cal_exec();
         break;
      case 1:
         touch_draw_test();
         break;
   }

   redraw_frame();
   request_to_redraw_grid();
   draw_menu();
}

static void menu_config_cb(int item, uint8_t data)
{
   (void)data;

   switch (item)
   {
      case 0:
         ui_mode_normal();
         show_version();
         return;
   }

   redraw_frame();
   request_to_redraw_grid();
   draw_menu();
}

static void menu_calop_init_cb(int item, uint8_t data)
{
   (void)item;
   (void)data;
   extern const menuitem_t menu_calop[];

   #ifdef USE_LC_MATCHING
      // cancel LC matching
      config.flags &= ~CONFIG_FLAGS_LC_MATCH;
   #endif

   #ifdef USE_INTEGRATOR
      // disable the integrator
      config.integrator_coeff = 1.0f;
	#endif

   request_to_redraw_grid();

   // move onto next menu
   menu_push_submenu(menu_calop);
}

static void menu_config_save_cb(int item, uint8_t data)
{
   (void)item;
   (void)data;

   config_save();

   redraw_frame();
   request_to_redraw_grid();
   draw_menu();
}

static void menu_dfu_cb(int item, uint8_t data)
{
   (void)item;
   (void)data;

   enter_dfu();
}

static void menu_reset_config_cb(int item, uint8_t data)
{
   (void)item;
   (void)data;

   config_reset();

   redraw_frame();
   request_to_redraw_grid();
   draw_menu();
}

static void menu_reset_all_cals_cb(int item, uint8_t data)
{
   (void)item;
   (void)data;

   clear_all_config_prop_data();
   load_default_properties();

   redraw_frame();
   request_to_redraw_grid();
   draw_menu();
}

static void menu_save_cal_confirm_cb(int item, uint8_t data)
{
   (void)item;
   (void)data;

   if (caldata_save(save_cal_index, true) >= 0)
   {
//      menu_move_back(true, false);
//      menu_move_back(true, true);

        // back to main menu
      menu_current_level = 0;
        ui_mode_normal();

        draw_cal_status();
   }
}

static void menu_save_cal_cb(int item, uint8_t data)
{
   (void)item;
   extern const menuitem_t menu_save_cal_confirm[];

   save_cal_index = data;

   if (btn_check())
   {
        selection = 1;   // default to cancel
        btn_wait_release();
   }

   if (touch_status())
        touch_wait_release();

   // pause a bit to try to help prevent accidental save select
   chThdSleepMilliseconds(300);

   menu_push_submenu(menu_save_cal_confirm);
}

static void choose_active_trace(void)
{
   int t;

   if (config.current_trace >= 0 && config.current_trace < TRACES_MAX)
      if (trace[config.current_trace].enabled)
         return;

     for (t = 0; t < TRACES_MAX; t++)
     {
        if (trace[t].enabled)
        {
           config.current_trace = t;
           return;
        }
     }
}

static void menu_trace_cb(int item, uint8_t data)
{
   (void)item;

   if (trace[data].enabled)
   {
      if (data == config.current_trace)
      {  // disable if active trace is selected
         trace[data].enabled = FALSE;
         choose_active_trace();
      }
      else
      {  // make active selected trace
         config.current_trace = data;
      }
   }
   else
   {
      trace[data].enabled = TRUE;
      config.current_trace = data;
   }

   request_to_redraw_grid();
   draw_menu();
}

static void menu_format_cb(int item, uint8_t data)
{
   (void)item;

   set_trace_type(config.current_trace, data);

   request_to_redraw_grid();
   draw_menu();
}

static void menu_channel_cb(int item, uint8_t data)
{
   (void)item;

   set_trace_channel(config.current_trace, data);

   draw_menu();
}

static void menu_transform_window_cb(int item, uint8_t data)
{
   (void)item;

   // TODO
   domain_mode = (domain_mode & ~TD_WINDOW) | data;

   request_to_redraw_grid();
   draw_menu();
}

static void menu_transform_cb(int item, uint8_t data)
{
   (void)item;
   (void)data;

   domain_mode ^= DOMAIN_TIME;
   select_lever_mode(LM_MARKER);
   draw_frequencies();

   request_to_redraw_grid();
   draw_menu();
}

static void menu_velocity_cb(int item, uint8_t data)
{
   (void)item;
   (void)data;

   if (btn_wait_release() & EVT_BUTTON_DOWN_LONG)
   {
      ui_mode_numeric(KM_VELOCITY_FACTOR);
      ui_process_numeric();
   }
   else
   {
      ui_mode_keypad(KM_VELOCITY_FACTOR);
      ui_process_keypad();
   }
}

static void menu_transform_filter_cb(int item, uint8_t data)
{
   (void)item;

   domain_mode = (domain_mode & ~TD_FUNC) | data;

   draw_menu();
}

static void menu_bandwidth_cb(int item, uint8_t data)
{
   (void)item;

   config.bandwidth = data;
   draw_frequencies();

   draw_menu();
}

#ifdef USE_INTEGRATOR
	static void menu_integrator_cb(int item, uint8_t data)
	{
		(void)item;
		(void)data;

		config.integrator_coeff = (config.integrator_coeff == 1.0f) ? INTEGRATOR_COEFF : 1.0f;	// toggle 1.0/INTEGRATOR_COEFF

		request_to_redraw_grid();
		draw_menu();
	}
#endif

#ifdef USE_LC_MATCHING
   static void menu_lc_match_cb(int item, uint8_t data)
   {
      (void)item;
        (void)data;

      config.flags ^= CONFIG_FLAGS_LC_MATCH;

      if (config.flags & CONFIG_FLAGS_LC_MATCH)
         lc_match_process();

      request_to_redraw_grid();
      draw_menu();
   }
#endif

#ifdef POINTS_SET_51
   static void menu_points_cb(int item, uint8_t data)
   {
      (void)item;
      set_sweep_points(data);
      draw_menu();
   }
#endif

static void choose_active_marker(void)
{
   int i;
   for (i = 0; i < MARKERS_MAX; i++)
   {
      if (marker_enabled(i))
      {
         set_active_marker(i);
         return;
      }
   }
   set_no_active_marker;
}

static void menu_scale_cb(int item, uint8_t data)
{
   (void)item;

   if (data == KM_SCALE && trace[config.current_trace].type == TRC_DELAY)
      data = KM_SCALEDELAY;

   if (btn_wait_release() & EVT_BUTTON_DOWN_LONG)
   {
      ui_mode_numeric(data);
      ui_process_numeric();
   }
   else
   {
      ui_mode_keypad(data);
      ui_process_keypad();
   }
}

static void menu_stimulus_cb(int item, uint8_t data)
{
   (void)data;

   int km_mode = KM_NONE;

   switch (item)
   {
      case 0:
         km_mode = KM_START;
         break;
      case 1:
         km_mode = KM_STOP;
         break;
      case 2:
         km_mode = KM_CENTER;
         break;
      case 3:
         km_mode = KM_SPAN;
         break;
      case 4:
         km_mode = KM_CW;
         break;
      case 5:   // PAUSE
         toggle_sweep();
         draw_menu();
      default:
         return;
   }

   if (btn_wait_release() & EVT_BUTTON_DOWN_LONG)
   {
      ui_mode_numeric(km_mode);
      ui_process_numeric();
   }
   else
   {
      ui_mode_keypad(km_mode);
      ui_process_keypad();
   }
}

static void menu_marker_op_cb(int item, uint8_t data)
{
   const int marker = active_marker;
   if (marker < 0 || marker >= MARKERS_MAX)
      return;

   uint32_t marker_freq1 = get_marker_frequency(marker);
   uint32_t marker_freq2 = (previous_marker >= 0) ? get_marker_frequency(previous_marker) : 0;

   if (marker_freq1 == 0)
      return; // no valid marker

   const int current_trace = config.current_trace;

   const uint32_t center = get_sweep_frequency(ST_CENTER);
   const uint32_t span   = (center > marker_freq1) ? center - marker_freq1 : marker_freq1 - center;

   switch (item)
   {
      case 0: /* MARKER->START */
      case 1: /* MARKER->STOP */
      case 2: /* MARKER->CENTER */
         set_sweep_frequency(data, marker_freq1);
         break;

      case 3: /* MARKERS->SPAN */
         if (previous_marker < 0 || marker == previous_marker)
         {  // if only 1 marker is active, keep center freq and make span the marker comes to the edge
            set_sweep_frequency(ST_SPAN, span * 2);
         }
         else   // if 2 or more marker active, set start and stop freq to each marker
         if (marker_freq2 > 0)
         {
             if (marker_freq1 > marker_freq2)
              {   // swap
                const uint32_t f = marker_freq2;
                 marker_freq2 = marker_freq1;
                 marker_freq1 = f;
              }
              set_sweep_frequency(ST_START, marker_freq1);
              set_sweep_frequency(ST_STOP,  marker_freq2);
         }
         break;

      case 4: /* MARKERS->EDELAY */
         if (current_trace >= 0 && current_trace < TRACES_MAX)
         {
             float (*array)[2] = measured[trace[current_trace].channel];
             const float v = groupdelay_from_array(marker_index(marker), array);
             if (v != 0.0f)
                 set_electrical_delay(electrical_delay + (v / 1e-12f));
         }
         break;
   }

   draw_frequencies();
   draw_menu();
}

static void menu_marker_search_cb(int item, uint8_t data)
{
   (void)data;

   const int marker = active_marker;
   if (marker < 0 || marker >= MARKERS_MAX)
      return;

   const int index = marker_index(marker);
   int new_index = -1;

   int marker_mode = uistat.lever_mode;

   switch (item)
   {
      case 0: // none
         uistat.marker_tracking = FALSE;
         marker_mode = LM_MARKER;
         break;

      case 1: // maximum
         set_marker_search(0);
         new_index = marker_search();
         marker_mode = (uistat.marker_tracking) ? LM_SEARCH : LM_MARKER;
         break;

      case 2: // minimum
         set_marker_search(1);
         new_index = marker_search();
         marker_mode = (uistat.marker_tracking) ? LM_SEARCH : LM_MARKER;
         break;

      case 3: // search Left
         new_index = marker_search_left(index);
         uistat.marker_tracking = FALSE;
         marker_mode = LM_MARKER;
         break;

      case 4: // search right
         new_index = marker_search_right(index);
         uistat.marker_tracking = FALSE;
         marker_mode = LM_MARKER;
         break;

      case 5: // tracking
         uistat.marker_tracking = !uistat.marker_tracking;
         marker_mode = (uistat.marker_tracking) ? LM_SEARCH : LM_MARKER;
         break;
   }

   if (new_index >= 0 && new_index < sweep_points)
      set_marker_index(marker, new_index);

   select_lever_mode(marker_mode);

   redraw_marker(marker);

   draw_menu();
}

static void menu_marker_smith_cb(int item, uint8_t data)
{
   (void)item;

   marker_smith_format = data;

   redraw_marker(active_marker);
   draw_menu();
}

static void active_marker_select(int item)
{
   if (item < 0)
   {
      set_active_marker(previous_marker);
      previous_marker = -1;
      if (active_marker >= MARKERS_MAX)
         choose_active_marker();
   }
   else
   if (item >= 0 && item < MARKERS_MAX)
   {
      if (previous_marker != active_marker)
         previous_marker = active_marker;
      set_active_marker(item);
   }
}

static void menu_marker_sel_cb(int item, uint8_t data)
{
   (void)data;

   if (item >= 0 && item < MARKERS_MAX)
   {
      if (marker_enabled(item))
      {
         if (item == active_marker)
         { // disable if active trace is selected
            disable_marker(item);
            active_marker_select(-1);
         }
         else
         {
            active_marker_select(item);
         }
      }
      else
      {
         enable_marker(item);
         active_marker_select(item);
      }
   }
   else
   if (item == 4)
   { // all off
      int m;
      for (m = 0; m < MARKERS_MAX; m++)
         disable_marker(m);
      previous_marker = -1;
      set_no_active_marker;
   }
   else
   if (item == 5)
   { // marker delta
      uistat.marker_delta = !uistat.marker_delta;
   }

   redraw_marker(active_marker);

   draw_menu();
}

static void draw_button(const int x, const int y, const int w, const int h, const int bw, const bool highlight, uint16_t bg_colour)
{  // draw button 3D border
   // x .......... left
   // y .......... top
   // w .......... width
   // h .......... height
   // bw ......... border width
   // bg_colour .. background colour
   // highlight .. if button is highlighted

   // background
   ili9341_fill(x + bw, y + bw, w - (bw * 2), h - (bw * 2), bg_colour);

   // 3D border
// const int hcol = highlight ? RGB565(0, 0, 0) : DEFAULT_MENU_BORDER_HI_COLOR;
// const int lcol = highlight ? RGB565(0, 0, 0) : DEFAULT_MENU_BORDER_LO_COLOR;
   const int hcol = highlight ? DEFAULT_MENU_BORDER_LO_COLOR : DEFAULT_MENU_BORDER_HI_COLOR;
   const int lcol = highlight ? DEFAULT_MENU_BORDER_HI_COLOR : DEFAULT_MENU_BORDER_LO_COLOR;
   ili9341_fill(x,          y,          w,  bw, hcol);   // top
   ili9341_fill(x + w - bw, y,          bw,  h, hcol);   // right
   ili9341_fill(x,          y,          bw,  h, lcol);   // left
   ili9341_fill(x,          y + h - bw, w,  bw, lcol);   // bottom
}

#ifdef __USE_SD_CARD__
   static const char s_file_header1[] = "! File created by: NanoVNA\r\n";
   static const char s_file_header2[] = "! Date: %04u.%02u.%02u %02u:%02u:%02u\r\n";
   static const char s_file_header3[] = "! Start: %uHz Stop: %uHz Points: %u\r\n";
   static const char s1_file_header[] = "! S-Parameter data: F S11\r\n";
   static const char s2_file_header[] = "! S-Parameter data: F S11 S21\r\n";
   static const char s_file_header4[] = "# HZ S RI R 50\r\n";
   static const char s1_file_param[]  = "%10u %+f %+f\r\n";
   static const char s2_file_param[]  = "%10u %+f %+f %+f %+f 0 0 0 0\r\n";

   static void menu_show_mounting(void)
   {
      ili9341_set_foreground(DEFAULT_MENU_TEXT_COLOR);
      ili9341_set_background(DEFAULT_MENU_COLOR);   // test only
      draw_button((LCD_WIDTH / 2) - 125, (LCD_HEIGHT / 2) - 30, 250, 60, 2, false, DEFAULT_MENU_COLOR);   // test only

      ili9341_drawstring_size("MOUNTING CARD", (LCD_WIDTH - (13 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT - (FONT_GET_HEIGHT * 6)) / 2, 2, false);
   }

   static void menu_show_message(const char *s1, const char *s2)
   {
      if (s1 || s2)
      {
         ili9341_set_foreground(DEFAULT_MENU_TEXT_COLOR);
         ili9341_set_background(DEFAULT_MENU_COLOR);   // test only
         draw_button((LCD_WIDTH / 2) - 125, (LCD_HEIGHT / 2) - 30, 250, 60, 2, false, DEFAULT_MENU_COLOR);   // test only
      }

      if (s1)
         ili9341_drawstring_size(s1, (LCD_WIDTH - (13 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT - (FONT_GET_HEIGHT * 6)) / 2, 2, false);

      if (s2)
      {
         #if (FF_USE_LFN >= 1)
            ili9341_drawstring_size(s2, (LCD_WIDTH - (22 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT + (FONT_GET_HEIGHT * 2)) / 2, 2, false);
         #else
            ili9341_drawstring_size(s2, (LCD_WIDTH - (13 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT + (FONT_GET_HEIGHT * 2)) / 2, 2, false);
         #endif
      }
   }

   void sd_save_sparam_file(uint8_t data, bool show_messages)
   {
      char *buf = (char *)spi_buffer;

     	if (data != SAVE_S1P_FILE && data != SAVE_S2P_FILE)
     		return;

      ui_mode_normal();

      #ifdef __USE_RTC__
         const uint32_t tr = rtc_get_tr_bin(); // TR read first
         const uint32_t dr = rtc_get_dr_bin(); // DR read second
      #endif

      // Prepare filename = .s1p or .s2p and open for write
      #if (FF_USE_LFN >= 1) && defined(__USE_RTC__)
         plot_printf(fs_filename, FF_LFN_BUF, "VNA_%06X_%06X.s%dp", dr, tr, data);
      #else
         plot_printf(fs_filename, FF_LFN_BUF, "%08X.s%dp", rtc_get_FAT(), data);
      #endif

      if (show_messages)
         menu_show_mounting();

      //shell_printf("S file\r\n");
      FRESULT res = f_mount(fs_volume, "", 1);
      //shell_printf("Mount = %d\r\n", res);
      if (res == FR_OK)
      {
         UINT size;
          //UINT total_size = 0;
         //systime_t time  = chVTGetSystemTimeX();

         if (show_messages)
         	menu_show_message("SAVING S-PARAMS", fs_filename);

         res = f_open(fs_file, fs_filename, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
         //shell_printf("Open %s, = %d\r\n", fs_filename, res);
         if (res == FR_OK)
         {
         	// "! File created by: NanoVNA\r\n"
   			f_write(fs_file, s_file_header1, strlen(s_file_header1), &size);
            //total_size += size;

				#ifdef __USE_RTC__
   				// "! Date: %04u.%02u.%02u %02u:%02u:%02u\r\n";
   				size = plot_printf(buf, 128, s_file_header2,
						RTC_START_YEAR + RTC_DR_YEAR(dr),
						RTC_DR_MONTH(dr),
						RTC_DR_DAY(dr),
						RTC_TR_HOUR(dr),
						RTC_TR_MIN(dr),
						RTC_TR_SEC(dr));
   				f_write(fs_file, buf, size, &size);
   				//total_size += size;
				#endif

   			// "! Start: %uHz Stop: %uHz Points: %u\r\n";
   			size = plot_printf(buf, 128, s_file_header3, get_sweep_frequency(ST_START), get_sweep_frequency(ST_STOP), sweep_points);
  				f_write(fs_file, buf, size, &size);
  				//total_size += size;

   			if (data == SAVE_S1P_FILE)
         	{
            	// "! S-Parameter data: F S11\r\n";
   				size = plot_printf(buf, 128, s1_file_header);
   				f_write(fs_file, buf, size, &size);
   				//total_size += size;
         	}
         	else
         	if (data == SAVE_S2P_FILE)
         	{
         		// "! S-Parameter data: F S11 S21\r\n";
   				size = plot_printf(buf, 128, s2_file_header);
   				f_write(fs_file, buf, size, &size);
   				//total_size += size;
         	}

            // static const char s_file_header4[] = "# HZ S RI R 50\r\n";
   			size = plot_printf(buf, 128, s_file_header4);
            f_write(fs_file, buf, size, &size);
            //total_size += size;

            {	// Write SP data to file
            	unsigned int i;
            	for (i = 0; i < sweep_points && res == FR_OK; i++)
            	{
            		size = 0;
            		if (data == SAVE_S1P_FILE)
            			size = plot_printf(buf, 128, s1_file_param, frequencies[i], measured[0][i][0], measured[0][i][1]);
            		else
            			if (data == SAVE_S2P_FILE)
            				size = plot_printf(buf, 128, s2_file_param, frequencies[i], measured[0][i][0], measured[0][i][1], measured[1][i][0], measured[1][i][1]);
            		if (size > 0)
            		{
            			res = f_write(fs_file, buf, size, &size);
            			//shell_printf("Write %s %u, = %d\r\n", fs_filename, size, res);
            			if (res != FR_OK)
            				break;
            			//total_size += size;
            		}
            	}
            }

            if (res != FR_OK)
            {
            	res = f_close(fs_file);
					#if (FF_FS_MINIMIZE == 0)
            		// delete the file
						f_unlink(fs_filename);
					#endif
            }
            else
            	res = f_close(fs_file);
            //shell_printf("Close = %d\r\n", res);

            //testLog();
            //time = chVTGetSystemTimeX() - time;
            //shell_printf("Total time: %dms (write %d byte/sec)\r\n", time/10, total_size*10000/time);
         }

         if (show_messages)
         	ili9341_drawstring_size((res == FR_OK) ? "S-PARAMS SAVED  " : "Write failed  ", (LCD_WIDTH - (13 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT - (FONT_GET_HEIGHT * 6)) / 2, 2, false);
      }
      else
      {
      	if (show_messages)
      		ili9341_drawstring_size("failed", (LCD_WIDTH - (7 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT + (FONT_GET_HEIGHT * 2)) / 2, 2, false);
      }

      if (show_messages)
      {
      	draw_frequencies();
      	request_to_redraw_grid();

      	touch_wait_release();

      	chThdSleepMilliseconds(800);
      }
   }

	#ifdef __USE_SD_CARD_BACKUP_RESTORE_CONFIG_CAL__
   	void sd_restore_config(void)
   	{
   		ui_mode_normal();

   		plot_printf(fs_filename, FF_LFN_BUF, "config.bin");

   		FRESULT res = f_mount(fs_volume, "", 1);
   		if (res == FR_OK)
   		{
   			UINT size = 0;

   			menu_show_message("LOADING CONFIG", fs_filename);

   			res = f_open(fs_file, fs_filename, FA_OPEN_EXISTING | FA_READ);
   			if (res == FR_OK)
   			{  // read config from file
   				res = f_read(fs_file, &config, sizeof(config), &size);

   				f_close(fs_file);

   				if (res != FR_OK || size != sizeof(config))
   					res = FR_DISK_ERR;
   			}

  				if (res != FR_OK)
  				{
  					// read the config back from flash
  					config_recall();
  				}

   			ili9341_drawstring_size((res == FR_OK) ? "CONFIG LOADED " : "Write failed  ", (LCD_WIDTH - (13 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT - (FONT_GET_HEIGHT * 6)) / 2, 2, false);
   		}
   		else
   		{
   			ili9341_drawstring_size("failed", (LCD_WIDTH - (7 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT + (FONT_GET_HEIGHT * 2)) / 2, 2, false);
   		}

   		redraw_frame();
   		request_to_redraw_grid();
 //  		draw_cal_status();

   		touch_wait_release();

   		chThdSleepMilliseconds(800);
   	}

   	void sd_restore_cal(void)
   	{
   		ui_mode_normal();

   		//shell_printf("CAL file\r\n");

   		FRESULT res = f_mount(fs_volume, "", 1);
      	//shell_printf("Mount = %d\r\n", res);
  			if (res == FR_OK)
  			{
  				int i;
  				for (i = 0; i < SAVEAREA_MAX; i++)
  				{
   				UINT size = 0;

   				plot_printf(fs_filename, FF_LFN_BUF, "cal_%d.bin", i);

   				//shell_printf("CAL file %s\r\n", fs_filename);

   				menu_show_message("LOADING CAL", fs_filename);

   				res = f_open(fs_file, fs_filename, FA_READ);
   				//shell_printf("Open %s, = %d\r\n", fs_filename, res);
   				if (res == FR_OK)
   				{  // read calibration from file

						res = f_read(fs_file, &current_props, sizeof(current_props), &size);
  						//shell_printf("read %s %u, = %d\r\n", fs_filename, size, res);
/*
  						// do it in smaller blocks because the disk_read() can't deal with multiple sectors
  						uint8_t *dst = (uint8_t *)&current_props;
  						unsigned int k = 0;
  						while (res == FR_OK && k < sizeof(current_props))
  						{
  							unsigned int m = sizeof(current_props) - k;
  							if (m > FF_MAX_SS)
								m = FF_MAX_SS;
  							res = f_read(fs_file, &dst[k], m, &size);
  							//shell_printf("read %u/%u %s %u, = %d\r\n", k, sizeof(current_props), fs_filename, size, res);
  							k += m;
  						}
*/
   					f_close(fs_file);
   					//FRESULT cres = f_close(fs_file);
   					//shell_printf("Close = %d\r\n", cres);

  						if (res != FR_OK)
  							res = FR_DISK_ERR;
  						else
  							caldata_save(i, false);	// save to flash
   				}
   			}

				if (res != FR_OK)
				{	// reload the calbration we had before saving
					caldata_recall(config.current_id, false);
				}
				else
				{
					caldata_recall(config.current_id, true);
				}

   			ili9341_drawstring_size((res == FR_OK) ? "CAL LOADED    " : "read failed   ", (LCD_WIDTH - (13 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT - (FONT_GET_HEIGHT * 6)) / 2, 2, false);
   		}
   		else
   		{
   			ili9341_drawstring_size("failed", (LCD_WIDTH - (7 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT + (FONT_GET_HEIGHT * 2)) / 2, 2, false);
   		}

   		redraw_frame();
   		request_to_redraw_grid();
//   		draw_cal_status();

   		touch_wait_release();

   		chThdSleepMilliseconds(800);
   	}

   	void sd_backup_config(void)
   	{
   		ui_mode_normal();

   		plot_printf(fs_filename, FF_LFN_BUF, "config.bin");

   		//shell_printf("CONFIG file %s\r\n", fs_filename);

   		FRESULT res = f_mount(fs_volume, "", 1);
   		if (res == FR_OK)
   		{
   			UINT size = 0;

   			menu_show_message("SAVING CONFIG", fs_filename);

   			res = f_open(fs_file, fs_filename, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
   			if (res == FR_OK)
   			{  // write config to file
   				res = f_write(fs_file, &config, sizeof(config), &size);

   				f_close(fs_file);

   				if (res != FR_OK || size != sizeof(config))
   					res = FR_DISK_ERR;
   			}

  				if (res != FR_OK)
  				{
					#if (FF_FS_MINIMIZE == 0)
            		// delete the file
						f_unlink(fs_filename);
					#endif
  				}

   			ili9341_drawstring_size((res == FR_OK) ? "CONFIG SAVED  " : "Write failed  ", (LCD_WIDTH - (13 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT - (FONT_GET_HEIGHT * 6)) / 2, 2, false);
   		}
   		else
   		{
   			ili9341_drawstring_size("failed", (LCD_WIDTH - (7 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT + (FONT_GET_HEIGHT * 2)) / 2, 2, false);
   		}

//   		draw_frequencies();
   		request_to_redraw_grid();

   		touch_wait_release();

   		chThdSleepMilliseconds(800);
   	}

   	void sd_backup_cal(void)
   	{
   		ui_mode_normal();

   		//shell_printf("CAL file\r\n");

   		FRESULT res = f_mount(fs_volume, "", 1);
      	//shell_printf("Mount = %d\r\n", res);
  			if (res == FR_OK)
  			{
  				int i;
  				for (i = 0; i < SAVEAREA_MAX; i++)
  				{
   				UINT size = 0;

  					plot_printf(fs_filename, FF_LFN_BUF, "cal_%d.bin", i);

   				menu_show_message("SAVING CAL", fs_filename);

   				res = f_open(fs_file, fs_filename, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
   				//shell_printf("Open %s, = %d\r\n", fs_filename, res);
   				if (res == FR_OK)
   				{  // write calibration to file

   					// fetch the calibration
   					const int cal_res = caldata_recall(i, false);
   					if (cal_res >= 0)
   					{
   						res = f_write(fs_file, &current_props, sizeof(current_props), &size);
/*
   						// do it in smaller blocks because the disk_write() can't deal with multiple sectors
   						const uint8_t *src = (const uint8_t *)&current_props;
   						unsigned int k = 0;
   						while (res == FR_OK && k < sizeof(current_props))
   						{
   							unsigned int m = sizeof(current_props) - k;
   							if (m > FF_MAX_SS)
  									m = FF_MAX_SS;
   							res = f_write(fs_file, &src[k], m, &size);
   							//shell_printf("Write %u/%u %d %s %u, = %d\r\n", k, sizeof(current_props), cal_res, fs_filename, size, res);
   							k += m;
   						}
*/
   					}

   					f_close(fs_file);
   					//FRESULT cres = f_close(fs_file);
   					//shell_printf("Close = %d\r\n", cres);

  						if (res != FR_OK || cal_res < 0)
  						{
							#if (FF_FS_MINIMIZE == 0)
            				// delete the file
								res = f_unlink(fs_filename);
   							//shell_printf("unlink %s, = %d\r\n", fs_filename, res);
							#endif
  							res = FR_DISK_ERR;
  						}
   				}
   			}

  				// load the calbration slot/memory
  				caldata_recall(config.current_id, true);

   			ili9341_drawstring_size((res == FR_OK) ? "CAL SAVED     " : "Write failed  ", (LCD_WIDTH - (13 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT - (FONT_GET_HEIGHT * 6)) / 2, 2, false);
   		}
   		else
   		{
   			ili9341_drawstring_size("failed", (LCD_WIDTH - (7 * (FONT_WIDTH * 2))) / 2, (LCD_HEIGHT + (FONT_GET_HEIGHT * 2)) / 2, 2, false);
   		}

//   		draw_frequencies();
   		request_to_redraw_grid();

   		touch_wait_release();

   		chThdSleepMilliseconds(800);
   	}
	#endif

   static void menu_sdcard_cb(int item, uint8_t data)
   {
      (void)item;

      switch (data)
      {
      	case SAVE_S1P_FILE:
      	case SAVE_S2P_FILE:
      		sd_save_sparam_file(data, true);
      		break;
			#ifdef __USE_SD_CARD_BACKUP_RESTORE_CONFIG_CAL__
				case BACKUP_CONFIG_FILE:
					sd_backup_config();
					break;
				case BACKUP_CAL_FILE:
					sd_backup_cal();
					break;
				case RESTORE_CONFIG_FILE:
					sd_restore_config();
					break;
				case RESTORE_CAL_FILE:
					sd_restore_cal();
					break;
			#endif
			#ifdef __USE_SD_CARD_AUTO_SAVE_PERIOD__
				case AUTO_SAVE_SECS:
					ui_mode_keypad(KM_AUTO_SAVE_SECS);
					ui_process_keypad();
					break;
			#endif
      }
   }
#endif

// ****************************************************************************************

// type of menu item
enum { MT_NONE, MT_BLANK, MT_SUBMENU, MT_CALLBACK, MT_CANCEL, MT_CLOSE };

// example format .. can be 1, 2 or 3 text lines
//  { MT_CALLBACK, 0, "line 1",                   menu_cb },
//  { MT_CALLBACK, 1, "\2line 1\0line 2",         menu_cb },
//  { MT_CALLBACK, 2, "\3line 1\0line 2\0line 3", menu_cb },

#ifdef __USE_SD_CARD__
   const menuitem_t menu_sdcard[] = {
      { MT_CALLBACK,    SAVE_S1P_FILE,       "SAVE S1P",       menu_sdcard_cb },
      { MT_CALLBACK,    SAVE_S2P_FILE,       "SAVE S2P",       menu_sdcard_cb },
		#ifdef __USE_SD_CARD_BACKUP_RESTORE_CONFIG_CAL__
			{ MT_CALLBACK, BACKUP_CONFIG_FILE,  "SAVE CONFIG",    menu_sdcard_cb },
			{ MT_CALLBACK, BACKUP_CAL_FILE,     "SAVE CAL's",     menu_sdcard_cb },
			{ MT_CALLBACK, RESTORE_CONFIG_FILE, "LOAD CONFIG",    menu_sdcard_cb },
			{ MT_CALLBACK, RESTORE_CAL_FILE,    "LOAD CAL's",     menu_sdcard_cb },
		#endif
		#ifdef __USE_SD_CARD_AUTO_SAVE_PERIOD__
			{ MT_CALLBACK, AUTO_SAVE_SECS,      "AUTO SAVE SECS", menu_sdcard_cb },
		#endif
		{ MT_CANCEL,      255,                 S_LARROW" BACK", NULL },
      { MT_NONE,        255,                 NULL,            NULL } // sentinel
   };
#endif

const menuitem_t menu_calop[] = {
  { MT_CALLBACK, CAL_OPEN,  "CH0   OPEN",    menu_calop_cb   },
  { MT_CALLBACK, CAL_SHORT, "CH0   SHORT",   menu_calop_cb   },
  { MT_CALLBACK, CAL_LOAD,  "CH0   LOAD",    menu_calop_cb   },
  { MT_CALLBACK, CAL_ISOLN, "CH0-1 ISOLN",   menu_calop_cb   },
  { MT_CALLBACK, CAL_THRU,  "CH0-1 THRU",    menu_calop_cb   },
  { MT_CALLBACK,   0,       "DONE",          menu_caldone_cb },
  { MT_CANCEL,   255,       S_LARROW" BACK", NULL            },
  { MT_NONE,     255,       NULL,            NULL            } // sentinel
};

const menuitem_t menu_save_cal_confirm[] = {
  { MT_CALLBACK,   0, "SAVE CAL",      menu_save_cal_confirm_cb },
  { MT_CANCEL,   255, S_LARROW" BACK", NULL                     },
  { MT_NONE,     255, NULL,            NULL                     } // sentinel
};

const menuitem_t menu_save_cal[] = {
  { MT_CALLBACK,   0, "SAVE CAL 0",    menu_save_cal_cb },
  { MT_CALLBACK,   1, "SAVE CAL 1",    menu_save_cal_cb },
  { MT_CALLBACK,   2, "SAVE CAL 2",    menu_save_cal_cb },
  { MT_CALLBACK,   3, "SAVE CAL 3",    menu_save_cal_cb },
  { MT_CALLBACK,   4, "SAVE CAL 4",    menu_save_cal_cb },
  { MT_CANCEL,   255, S_LARROW" BACK", NULL             },
  { MT_NONE,     255, NULL,            NULL             } // sentinel
};

const menuitem_t menu_cal[] = {
//{ MT_SUBMENU,    0, S_RARROW" CALIBRATE", menu_calop         },
  { MT_CALLBACK,   0, S_RARROW" CALIBRATE", menu_calop_init_cb },
  { MT_SUBMENU,    0, S_RARROW" SAVE",      menu_save_cal      },
  { MT_CALLBACK,   0, "RESET",              menu_cal2_cb       },
  { MT_CALLBACK,   0, "ENABLE",             menu_cal2_cb       },
  { MT_CANCEL,   255, S_LARROW" BACK",      NULL               },
  { MT_NONE,     255, NULL,                 NULL               } // sentinel
};

const menuitem_t menu_trace[] = {
  { MT_CALLBACK,   0, "TRACE 0",       menu_trace_cb },
  { MT_CALLBACK,   1, "TRACE 1",       menu_trace_cb },
  { MT_CALLBACK,   2, "TRACE 2",       menu_trace_cb },
  { MT_CALLBACK,   3, "TRACE 3",       menu_trace_cb },
  { MT_CANCEL,   255, S_LARROW" BACK", NULL          },
  { MT_NONE,     255, NULL,            NULL          } // sentinel
};

const menuitem_t menu_format2[] = {
   { MT_CALLBACK, TRC_POLAR,  "POLAR",         menu_format_cb },
   { MT_CALLBACK, TRC_LINEAR, "LINEAR",        menu_format_cb },
   { MT_CALLBACK, TRC_REAL,   "REAL",          menu_format_cb },
   { MT_CALLBACK, TRC_IMAG,   "IMAG",          menu_format_cb },
   { MT_CALLBACK, TRC_R,      "RESISTANCE",    menu_format_cb },
   { MT_CALLBACK, TRC_X,      "REACTANCE",     menu_format_cb },
   { MT_CALLBACK, TRC_Q,      "Q FACTOR",      menu_format_cb },
   { MT_CANCEL,   255,        S_LARROW" BACK", NULL           },
   { MT_NONE,     255,        NULL,            NULL           } // sentinel
};

const menuitem_t menu_format[] = {
   { MT_CALLBACK, TRC_LOGMAG, "LOG MAG",       menu_format_cb },
   { MT_CALLBACK, TRC_PHASE,  "PHASE",         menu_format_cb },
   { MT_CALLBACK, TRC_DELAY,  "DELAY",         menu_format_cb },
   { MT_CALLBACK, TRC_SMITH,  "SMITH",         menu_format_cb },
   { MT_CALLBACK, TRC_SWR,    "SWR",           menu_format_cb },
   { MT_SUBMENU,  255,        S_RARROW" MORE", menu_format2   },
   { MT_CANCEL,   255,        S_LARROW" BACK", NULL           },
   { MT_NONE,     255,        NULL,            NULL           } // sentinel
};

const menuitem_t menu_scale[] = {
  { MT_CALLBACK, KM_SCALE, "SCALE/DIV",              menu_scale_cb },
  { MT_CALLBACK, KM_REFPOS, "\2REFERENCE\0POSITION", menu_scale_cb },
  { MT_CALLBACK, KM_EDELAY, "\2ELECTRICAL\0DELAY",   menu_scale_cb },
  { MT_CANCEL,   255,       S_LARROW" BACK",         NULL          },
  { MT_NONE,     255,       NULL,                    NULL          } // sentinel
};

const menuitem_t menu_channel[] = {
//  { MT_CALLBACK,   0, "CH0 REFLECT",    menu_channel_cb },
//  { MT_CALLBACK,   1, "CH1 THROUGH",    menu_channel_cb },
  { MT_CALLBACK,   0, "CH 0  S11",      menu_channel_cb },
  { MT_CALLBACK,   1, "CH 1  S21",      menu_channel_cb },
  { MT_CANCEL,   255, S_LARROW" BACK",  NULL            },
  { MT_NONE,     255, NULL,             NULL            } // sentinel
};

const menuitem_t menu_transform_window[] = {
  { MT_CALLBACK, TD_WINDOW_MINIMUM, "MINIMUM",       menu_transform_window_cb },
  { MT_CALLBACK, TD_WINDOW_NORMAL,  "NORMAL",        menu_transform_window_cb },
  { MT_CALLBACK, TD_WINDOW_MAXIMUM, "MAXIMUM",       menu_transform_window_cb },
  { MT_CANCEL,   255,               S_LARROW" BACK", NULL                     },
  { MT_NONE,     255,               NULL,            NULL                     } // sentinel
};

const menuitem_t menu_transform[] = {
  { MT_CALLBACK,   0,                     "TRANSFORM ON",     menu_transform_cb        },
  { MT_CALLBACK, TD_FUNC_LOWPASS_IMPULSE, "LOW PASS IMPULSE", menu_transform_filter_cb },
  { MT_CALLBACK, TD_FUNC_LOWPASS_STEP,    "LOW PASS STEP",    menu_transform_filter_cb },
  { MT_CALLBACK, TD_FUNC_BANDPASS,        "BANDPASS",         menu_transform_filter_cb },
  { MT_SUBMENU,    0,                     S_RARROW" WINDOW",  menu_transform_window    },
  { MT_CALLBACK,   0,                     "VELOCITY FACTOR",  menu_velocity_cb         },
  { MT_CANCEL,   255,                     S_LARROW" BACK",    NULL                     },
  { MT_NONE,     255,                     NULL,               NULL                     } // sentinel
};

const menuitem_t menu_bandwidth[] = {
  { MT_CALLBACK, BANDWIDTH_2000, "2 kHz",         menu_bandwidth_cb },
  { MT_CALLBACK, BANDWIDTH_1000, "1 kHz",         menu_bandwidth_cb },
  { MT_CALLBACK, BANDWIDTH_333,  "333 Hz",        menu_bandwidth_cb },
  { MT_CALLBACK, BANDWIDTH_100,  "100 Hz",        menu_bandwidth_cb },
  { MT_CALLBACK, BANDWIDTH_30,   "30 Hz",         menu_bandwidth_cb },
  { MT_CALLBACK, BANDWIDTH_10,   "10 Hz",         menu_bandwidth_cb },
  { MT_CANCEL,   255,            S_LARROW" BACK", NULL              },
  { MT_NONE,     255,            NULL,            NULL              } // sentinel
};

const menuitem_t menu_display2[] = {
   { MT_SUBMENU,      0,                     S_RARROW" TRANSFORM", menu_transform     },
   { MT_SUBMENU,      0,                     S_RARROW" BANDWIDTH", menu_bandwidth     },
   #ifdef USE_INTEGRATOR
      { MT_CALLBACK,  INTEGRATOR_MENU_DATA,  "INTEGRATOR",         menu_integrator_cb },
   #endif
   #ifdef USE_LC_MATCHING
      { MT_CALLBACK,  LC_MATCHING_MENU_DATA, "LC MATCHING",        menu_lc_match_cb   },
   #endif
   { MT_CANCEL,     255,                     S_LARROW" BACK",      NULL               },
   { MT_NONE,       255,                     NULL,                 NULL               } // sentinel
};

const menuitem_t menu_display1[] = {
  { MT_SUBMENU,      0, S_RARROW" TRACE",     menu_trace     },
  { MT_SUBMENU,      0, S_RARROW" FORMAT",    menu_format    },
  { MT_SUBMENU,      0, S_RARROW" SCALE",     menu_scale     },
  { MT_SUBMENU,      0, S_RARROW" CHANNEL",   menu_channel   },
  { MT_SUBMENU,      0, S_RARROW" MORE",      menu_display2  },
  { MT_CANCEL,     255, S_LARROW" BACK",      NULL               },
  { MT_NONE,       255, NULL,                 NULL               } // sentinel
};

#ifdef POINTS_SET_51
   const menuitem_t menu_sweep_points[] = {
      { MT_CALLBACK, POINTS_SET_51,  " 51 POINTS",    menu_points_cb },
      { MT_CALLBACK, POINTS_SET_101, "101 POINTS",    menu_points_cb },
      { MT_CANCEL,   255,            S_LARROW" BACK", NULL           },
      { MT_NONE,     255,            NULL,            NULL           } // sentinel
   };
#endif

const menuitem_t menu_stimulus[] = {
  { MT_CALLBACK,   0, "START",          menu_stimulus_cb },
  { MT_CALLBACK,   0, "STOP",           menu_stimulus_cb },
  { MT_CALLBACK,   0, "CENTER",         menu_stimulus_cb },
  { MT_CALLBACK,   0, "SPAN",           menu_stimulus_cb },
  { MT_CALLBACK,   0, "CW FREQ",        menu_stimulus_cb },
  { MT_CALLBACK,   0, "PAUSE SWEEP",    menu_stimulus_cb },
  { MT_CANCEL,   255, S_LARROW" BACK",  NULL             },
  { MT_NONE,     255, NULL,             NULL             } // sentinel
};

const menuitem_t menu_marker_sel[] = {
  { MT_CALLBACK,   1, "MARKER 1",      menu_marker_sel_cb },
  { MT_CALLBACK,   2, "MARKER 2",      menu_marker_sel_cb },
  { MT_CALLBACK,   3, "MARKER 3",      menu_marker_sel_cb },
  { MT_CALLBACK,   4, "MARKER 4",      menu_marker_sel_cb },
  { MT_CALLBACK,   0, "ALL OFF",       menu_marker_sel_cb },
  { MT_CALLBACK,   0, "DELTA",         menu_marker_sel_cb },
  { MT_CANCEL,   255, S_LARROW" BACK", NULL               },
  { MT_NONE,     255, NULL,            NULL               } // sentinel
};

const menuitem_t menu_marker_ops[] = {
  { MT_CALLBACK, ST_START,  S_RARROW" START",  menu_marker_op_cb },
  { MT_CALLBACK, ST_STOP,   S_RARROW" STOP",   menu_marker_op_cb },
  { MT_CALLBACK, ST_CENTER, S_RARROW" CENTER", menu_marker_op_cb },
  { MT_CALLBACK, ST_SPAN,   S_RARROW" SPAN",   menu_marker_op_cb },
  { MT_CALLBACK,   0,       S_RARROW" EDELAY", menu_marker_op_cb },
  { MT_CANCEL,   255,       S_LARROW" BACK",   NULL              },
  { MT_NONE,     255,       NULL,              NULL              } // sentinel
};

const menuitem_t menu_marker_search[] = {
   { MT_CALLBACK,   0, "NONE",                  menu_marker_search_cb },
   { MT_CALLBACK,   0, "MAXIMUM",               menu_marker_search_cb },
   { MT_CALLBACK,   0, "MINIMUM",               menu_marker_search_cb },
   { MT_CALLBACK,   0, "SEARCH LEFT "S_LARROW,  menu_marker_search_cb },
   { MT_CALLBACK,   0, "SEARCH RIGHT "S_RARROW, menu_marker_search_cb },
   { MT_CALLBACK,   0, "ENABLE TRACKING",       menu_marker_search_cb },
   { MT_CANCEL,   255, S_LARROW" BACK",         NULL                  },
   { MT_NONE,     255, NULL,                    NULL                  } // sentinel
};

const menuitem_t menu_marker_smith[] = {
  { MT_CALLBACK, MS_LIN, "LINEAR",         menu_marker_smith_cb },
  { MT_CALLBACK, MS_LOG, "LOGARITHMIC",    menu_marker_smith_cb },
  { MT_CALLBACK, MS_REIM,"Re+Im",          menu_marker_smith_cb },
  { MT_CALLBACK, MS_RX,  "R+jX",           menu_marker_smith_cb },
  { MT_CALLBACK, MS_RLC, "R+L / R+C",      menu_marker_smith_cb },
  { MT_CANCEL,   255,     S_LARROW" BACK", NULL                 },
  { MT_NONE,     255,     NULL,            NULL                 } // sentinel
};

const menuitem_t menu_marker[] = {
  { MT_SUBMENU, 0,   S_RARROW" SELECT MARKER", menu_marker_sel    },
  { MT_SUBMENU, 0,   S_RARROW" SEARCH",        menu_marker_search },
  { MT_SUBMENU, 0,   S_RARROW" OPERATIONS",    menu_marker_ops    },
  { MT_SUBMENU, 0,   S_RARROW" SMITH VALUE",   menu_marker_smith  },
  { MT_CANCEL,  255, S_LARROW" BACK",          NULL               },
  { MT_NONE,    255, NULL,                     NULL               } // sentinel
};

const menuitem_t menu_recall[] = {
  { MT_CALLBACK,   0, "RECALL 0",      menu_recall_cb },
  { MT_CALLBACK,   1, "RECALL 1",      menu_recall_cb },
  { MT_CALLBACK,   2, "RECALL 2",      menu_recall_cb },
  { MT_CALLBACK,   3, "RECALL 3",      menu_recall_cb },
  { MT_CALLBACK,   4, "RECALL 4",      menu_recall_cb },
  { MT_CANCEL,   255, S_LARROW" BACK", NULL           },
  { MT_NONE,     255, NULL,            NULL           } // sentinel
};

const menuitem_t menu_dfu[] = {
  { MT_CALLBACK,   0, "\2RESET AND\0ENTER DFU", menu_dfu_cb },
  { MT_CANCEL,   255, S_LARROW" BACK",          NULL        },
  { MT_NONE,     255, NULL,                     NULL        } // sentinel
};

const menuitem_t menu_reset_config[] = {
  { MT_CALLBACK,   0, "RESET CONFIG",  menu_reset_config_cb },
  { MT_CANCEL,   255, S_LARROW" BACK", NULL                 },
  { MT_NONE,     255, NULL,            NULL                 } // sentinel
};

const menuitem_t menu_reset_all_cals[] = {
  { MT_CALLBACK,   0, "RESET ALL CAL's", menu_reset_all_cals_cb },
  { MT_CANCEL,   255, S_LARROW" BACK",   NULL                   },
  { MT_NONE,     255, NULL,              NULL                   } // sentinel
};

const menuitem_t menu_reset[] =
{
  { MT_SUBMENU,    0, S_RARROW" RESET CONFIG",    menu_reset_config   },
  { MT_SUBMENU,    0, S_RARROW" RESET ALL CAL's", menu_reset_all_cals },
  { MT_CANCEL,   255, S_LARROW" BACK",            NULL                },
  { MT_NONE,     255, NULL,                       NULL                } // sentinel
};

const menuitem_t menu_touch[] =
{
  { MT_CALLBACK,   0, "TOUCH CAL",     menu_touch_cb },
  { MT_CALLBACK,   0, "TOUCH TEST",    menu_touch_cb },
  { MT_CANCEL,   255, S_LARROW" BACK", NULL          },
  { MT_NONE,     255, NULL,            NULL          } // sentinel
};

const menuitem_t menu_config[] =
{
   { MT_CALLBACK,   0, "ABOUT",                 menu_config_cb      },
   { MT_CALLBACK,   0, "SAVE CONFIG",           menu_config_save_cb },
   #ifdef POINTS_SET_51
      { MT_SUBMENU, 0, S_RARROW" SWEEP POINTS", menu_sweep_points   },
   #endif
   { MT_SUBMENU,    0, S_RARROW" TOUCH",        menu_touch          },
   { MT_SUBMENU,    0, S_RARROW" RESET",        menu_reset          },
   { MT_SUBMENU,    0, S_RARROW" DFU",          menu_dfu            },
   { MT_CANCEL,   255, S_LARROW" BACK",         NULL                },
   { MT_NONE,     255, NULL,                    NULL                } // sentinel
};

const menuitem_t menu_top[] =
{
   { MT_SUBMENU,    0, S_RARROW" DISPLAY",          menu_display1 },
   { MT_SUBMENU,    0, S_RARROW" MARKERS",          menu_marker   },
   { MT_SUBMENU,    0, S_RARROW" STIMULUS",         menu_stimulus },
   { MT_SUBMENU,    0, S_RARROW" CALIBRATE",        menu_cal      },
   { MT_SUBMENU,    0, S_RARROW" LOAD CALIBRATION", menu_recall   },
   #ifdef __USE_SD_CARD__
      { MT_SUBMENU, 0, S_RARROW" SD CARD",          menu_sdcard   },
   #endif
   { MT_SUBMENU,    0, S_RARROW" CONFIG",           menu_config   },
   { MT_NONE,     255, NULL,                        NULL          } // sentinel
};

#define MENU_STACK_DEPTH_MAX 4

const menuitem_t *menu_stack[MENU_STACK_DEPTH_MAX] =
{
   menu_top, NULL, NULL, NULL
};

// this keeps track of the currently displayed menu buttons
struct {
   unsigned int count;            // number of currently displayed menu buttons
   struct
   {
      uint16_t x;         // button left
      uint16_t y;         // button top
      uint16_t w;         // button width
      uint16_t h;         // button height
   } pos[8];
} menu_button;

static void ensure_selection(void)
{
   const menuitem_t *menu = menu_stack[menu_current_level];
   int i;
   for (i = 0; menu[i].type != MT_NONE; i++);
   if (selection >= i)
      selection = i - 1;
}

static void menu_move_back(bool leave_ui, bool redraw_ui)
{
   if (menu_current_level == 0)
      return;

   menu_current_level--;

   if (!redraw_ui)
      return;

   ensure_selection();
   erase_menu_buttons();

   if (leave_ui)
      ui_mode_normal();
   else
      draw_menu();
}

static void menu_push_submenu(const menuitem_t *submenu)
{
   if (menu_current_level < (MENU_STACK_DEPTH_MAX - 1))
      menu_current_level++;

   menu_stack[menu_current_level] = submenu;

   ensure_selection();
   erase_menu_buttons();
   draw_menu();
}

/*
static void menu_move_top(void)
{
   if (menu_current_level == 0)
      return;

   menu_current_level = 0;

   ensure_selection();
   erase_menu_buttons();
   draw_menu();
}
*/

static void menu_invoke(int item)
{
   const menuitem_t *menu = menu_stack[menu_current_level];
   menu = &menu[item];

   menu_button.count = 0;

   switch (menu->type)
   {
      case MT_NONE:
      case MT_BLANK:
      case MT_CLOSE:
         ui_mode_normal();
         break;

      case MT_CANCEL:
         menu_move_back(false, true);
         break;

      case MT_CALLBACK:
         {
            const menuaction_cb_t cb = (menuaction_cb_t)menu->reference;
            if (cb == NULL)
               return;
            (*cb)(item, menu->data);
         }
         break;

      case MT_SUBMENU:
         menu_push_submenu((const menuitem_t *)menu->reference);
         break;
   }
}

typedef struct
{
   uint8_t x:4;
   uint8_t y:4;
   int8_t  c;
} keypads_t;

static const keypads_t *keypads;

static const keypads_t keypads_freq[] =
{
   { 1, 3, KP_PERIOD },
   { 0, 3, KP_0 },
   { 0, 2, KP_1 },
   { 1, 2, KP_2 },
   { 2, 2, KP_3 },
   { 0, 1, KP_4 },
   { 1, 1, KP_5 },
   { 2, 1, KP_6 },
   { 0, 0, KP_7 },
   { 1, 0, KP_8 },
   { 2, 0, KP_9 },
   { 3, 0, KP_G },
   { 3, 1, KP_M },
   { 3, 2, KP_K },
   { 3, 3, KP_X1 },
   { 2, 3, KP_BS },
   { 0, 0, -1 }
};

static const keypads_t keypads_scale[] =
{
   { 3, 0, KP_NONE },
   { 3, 1, KP_NONE },
   { 3, 2, KP_NONE },
   { 1, 3, KP_PERIOD },
   { 0, 3, KP_0 },
   { 0, 2, KP_1 },
   { 1, 2, KP_2 },
   { 2, 2, KP_3 },
   { 0, 1, KP_4 },
   { 1, 1, KP_5 },
   { 2, 1, KP_6 },
   { 0, 0, KP_7 },
   { 1, 0, KP_8 },
   { 2, 0, KP_9 },
   { 3, 3, KP_X1 },
   { 2, 3, KP_BS },
   { 0, 0, -1 }
};

static const keypads_t keypads_integer[] =
{
   { 3, 0, KP_NONE },
   { 3, 1, KP_NONE },
   { 3, 2, KP_NONE },
   { 1, 3, KP_NONE },
   { 0, 3, KP_0    },
   { 0, 2, KP_1    },
   { 1, 2, KP_2    },
   { 2, 2, KP_3    },
   { 0, 1, KP_4    },
   { 1, 1, KP_5    },
   { 2, 1, KP_6    },
   { 0, 0, KP_7    },
   { 1, 0, KP_8    },
   { 2, 0, KP_9    },
   { 3, 3, KP_X1   },
   { 2, 3, KP_BS   },
   { 0, 0, -1      }
};

static const keypads_t keypads_time[] =
{
   { 3, 0, KP_NONE },
   { 1, 3, KP_PERIOD },
   { 0, 3, KP_0 },
   { 0, 2, KP_1 },
   { 1, 2, KP_2 },
   { 2, 2, KP_3 },
   { 0, 1, KP_4 },
   { 1, 1, KP_5 },
   { 2, 1, KP_6 },
   { 0, 0, KP_7 },
   { 1, 0, KP_8 },
   { 2, 0, KP_9 },
   { 3, 1, KP_N },
   { 3, 2, KP_P },
   { 3, 3, KP_MINUS },
   { 2, 3, KP_BS },
   { 0, 0, -1 }
};

static const keypads_t * const keypads_mode_tbl[] =
{
   keypads_freq,  	// start
   keypads_freq,  	// stop
   keypads_freq,  	// center
   keypads_freq,  	// span
   keypads_freq,  	// cw freq
   keypads_scale, 	// scale
   keypads_scale, 	// refpos
   keypads_time,  	// electrical delay
   keypads_scale, 	// velocity factor
   keypads_time,  	// scale of delay
	//keypads_scale	// auto save seconds
	keypads_integer	// auto save seconds
};

static const char * const keypad_mode_label[] =
{
   "START", "STOP", "CENTER", "SPAN", "CW FREQ", "SCALE", "REFPOS", "EDELAY", "VELOCITY%", "DELAY", "SECONDS"
};

static void draw_keypad(void)
{
	// draw the background
	//ili9341_fill(0, 0, LCD_WIDTH, LCD_HEIGHT, DEFAULT_MENU_UNUSED_COLOR);

   int i = 0;
   while (keypads[i].c >= 0)
   {
      const bool highlight = (i == selection && keypads[i].c != KP_NONE) ? true : false;

//    const uint16_t bg = highlight ? config.menu_active_color : config.menu_normal_color;
      const uint16_t bg = highlight ? DEFAULT_MENU_SELECT_COLOR      : DEFAULT_MENU_COLOR;        // test only
      const uint16_t fg = highlight ? DEFAULT_MENU_SELECT_TEXT_COLOR : DEFAULT_MENU_TEXT_COLOR;   // test only

      const int x = KP_GET_X(keypads[i].x);
      const int y = KP_GET_Y(keypads[i].y);

      draw_button(x, y, KP_WIDTH, KP_HEIGHT, 3, highlight, bg);

      if (keypads[i].c != KP_NONE)
      {
      	ili9341_set_foreground(fg);
      	ili9341_set_background(bg);
      	ili9341_drawfont(keypads[i].c, x + (KP_WIDTH - NUM_FONT_GET_WIDTH) / 2, y + (KP_HEIGHT - NUM_FONT_GET_HEIGHT) / 2);
      }

      i++;
   }
}

static void draw_numeric_area_frame(void)
{
// draw_button(0, LCD_HEIGHT - NUM_INPUT_HEIGHT, LCD_WIDTH, NUM_INPUT_HEIGHT, 2, false, config.menu_normal_color);
   draw_button(0, LCD_HEIGHT - NUM_INPUT_HEIGHT, LCD_WIDTH, NUM_INPUT_HEIGHT, 2, false, DEFAULT_MENU_COLOR);      // test only

   ili9341_set_foreground(DEFAULT_NUMERIC_LABEL_COLOR);
// ili9341_set_background(config.menu_normal_color);
   ili9341_set_background(DEFAULT_MENU_COLOR);            // test only
   if (keypad_mode >= 0)
      ili9341_drawstring_size(keypad_mode_label[keypad_mode], 5, LCD_HEIGHT - (FONT_GET_HEIGHT + NUM_INPUT_HEIGHT) / 2 - 4, 2, false);

// ili9341_drawfont(KP_KEYPAD, 300, 216);
}
/*
static void draw_numeric_input(const char *buf)
{
   int i;
   int x         = 5 + (10 * (FONT_WIDTH * 2)) + 2;
   const int y   = LCD_HEIGHT - NUM_INPUT_HEIGHT + 5;
   int focused   = FALSE;
   uint16_t xsim = 0b0010010000000000;

   for (i = 0; i < 10 && buf[i]; i++, xsim <<= 1)
   {
      uint16_t fg = DEFAULT_MENU_TEXT_COLOR;
//    uint16_t bg = config.menu_normal_color;
      uint16_t bg = DEFAULT_MENU_COLOR;   // test only

      int c = buf[i];
      if (c == '.')
         c = KP_PERIOD;
      else
      if (c == '-')
         c = KP_MINUS;
      else
      //if (c >= '0' && c <= '9')
         c -= '0';

      if (ui_mode == UI_NUMERIC && uistat.digit == (8 - i))
      {
         fg = DEFAULT_SPEC_INPUT_COLOR;
//       if (uistat.digit_mode)
//          bg = DEFAULT_MENU_COLOR;

         focused = TRUE;
      }

      ili9341_set_foreground(fg);
      ili9341_set_background(bg);

      const int w = (xsim & 0x8000) ? NUM_FONT_GET_WIDTH + 2 + 8 : NUM_FONT_GET_WIDTH + 2;

      ili9341_fill(x, y, w, NUM_FONT_GET_HEIGHT, bg);

      if (c >= 0)    // c is number
         ili9341_drawfont(c, x, y);
      else
      if (focused)   // c not number, but focused
         ili9341_drawfont(0, x, y);

      x += w;
   }

   {   // erase rest of line
      int w = 0;
      for ( ; i < 10; i++, xsim <<= 1)
         w += (xsim & 0x8000) ? NUM_FONT_GET_WIDTH + 2 + 8 : NUM_FONT_GET_WIDTH + 2;
      //ili9341_fill(x, y, w, NUM_FONT_GET_HEIGHT, config.menu_normal_color);
      ili9341_fill(x, y, w, NUM_FONT_GET_HEIGHT, DEFAULT_MENU_COLOR);   // test only
   }
}
*/

static void draw_numeric_input(const char *buf)
{
   int i;
   int x         = 5 + (10 * (FONT_WIDTH * 2)) + 2;
   const int y   = LCD_HEIGHT - NUM_INPUT_HEIGHT + 5;
   int focused   = FALSE;
/*
   if (ui_mode != UI_NUMERIC)
   {
      uint16_t fg = DEFAULT_MENU_TEXT_COLOR;
//    uint16_t bg = config.menu_normal_color;
      uint16_t bg = DEFAULT_MENU_COLOR;   // test only

      ili9341_set_foreground(fg);
      ili9341_set_background(bg);

      plot_printf(s2, sizeof(s2), "%p", cal->_frequency1);
   }
*/
   for (i = 0; i < 10 && buf[i]; i++)
   {
      uint16_t fg = DEFAULT_MENU_TEXT_COLOR;
//    uint16_t bg = config.menu_normal_color;
      uint16_t bg = DEFAULT_MENU_COLOR;   // test only

      int c = buf[i];
      if (c == '.')
         c = KP_PERIOD;
      else
      if (c == '-')
         c = KP_MINUS;
      else
      //if (c >= '0' && c <= '9')
         c -= '0';

      if (ui_mode == UI_NUMERIC && uistat.digit == (8 - i))
      {
//       bg = DEFAULT_MENU_ACTIVE_COLOR;
//       fg = DEFAULT_MENU_ACTIVE_TEXT_COLOR;
         fg = (uistat.digit_mode) ? DEFAULT_SPEC_INPUT_COLOR : DEFAULT_SPEC_SELECT_COLOR;
         focused = TRUE;
      }

      ili9341_set_foreground(fg);
      ili9341_set_background(bg);

      const int w = NUM_FONT_GET_WIDTH + 2;

      ili9341_fill(x, y, w, NUM_FONT_GET_HEIGHT, bg);

      if (c >= 0)    // c is number
         ili9341_drawfont(c, x, y);
      else
      if (focused)   // c not number, but focused
         ili9341_drawfont(0, x, y);

      x += w;
   }

   // erase rest of line
// ili9341_fill(x, y, LCD_WIDTH - 2 - x, NUM_FONT_GET_HEIGHT, config.menu_normal_color);
   ili9341_fill(x, y, LCD_WIDTH - 2 - x, NUM_FONT_GET_HEIGHT, DEFAULT_MENU_COLOR);   // test only
}

static int menu_number_items(const menuitem_t *menu)
{   // return the number of menu items with MT_CALLBACK's
   int i = 0;
   while (menu[i].data != 255 && menu[i].reference != NULL)
   //while (menu[i].type == MT_CALLBACK)
      i++;
   return i;
}

static bool menu_item_modify_attribute(const menuitem_t *menu, int item, uint16_t *fg, uint16_t *bg)
{
   const int items         = menu_number_items(menu);
   const int current_trace = config.current_trace;
   bool modify             = false;

   if (menu == menu_display2)
   {
		#ifdef USE_INTEGRATOR
      	if (menu[item].data == INTEGRATOR_MENU_DATA && (config.integrator_coeff < 1.0f))
      		modify = true;        // integrator enabled
		#endif
      #ifdef USE_LC_MATCHING
         if (menu[item].data == LC_MATCHING_MENU_DATA && (config.flags & CONFIG_FLAGS_LC_MATCH))
            modify = true;     // LC matching enabled
      #endif
   }

   if (menu == menu_trace && item < TRACES_MAX)
   {
      if (trace[item].enabled)
      {
         *bg = config.trace_color[item];
         *fg = DEFAULT_BG_COLOR;   // test only
         return true;
      }
      return false;
   }

   if (menu == menu_format || menu == menu_format2)
   {
      if (item < items && menu[item].data == trace[current_trace].type)
           modify = true;
   }

   if (menu == menu_channel)
   {
      if (item < items && menu[item].data == trace[current_trace].channel)
           modify = true;
   }

   if (menu == menu_marker_sel)
   {
      if (item < (items - 1))
      {
         if (marker_enabled(item))
            modify = true;
      }
      else
      if (item == (items - 1))   // delta
      {
         if (uistat.marker_delta)
            modify = true;
      }
   }

   if (menu == menu_marker_search)
   {
      if (  (item == 0 && (uistat.lever_mode == LM_MARKER)) ||   // none
            (item == 1 && (get_marker_search() == 0) && (uistat.lever_mode == LM_SEARCH)) ||   // max
            (item == 2 && (get_marker_search() == 1) && (uistat.lever_mode == LM_SEARCH)) ||   // min
            (item == (items - 1) && uistat.marker_tracking))   // tracking
      {
         modify = true;
      }
   }

   if (menu == menu_marker_smith)
   {
      if (marker_smith_format == item)
         modify = true;
   }

   if (menu == menu_calop)
   {
      //if ( (item == 0 && (cal_status & CALSTAT_OPEN))
      //  || (item == 1 && (cal_status & CALSTAT_SHORT))
      //  || (item == 2 && (cal_status & CALSTAT_LOAD))
      //  || (item == 3 && (cal_status & CALSTAT_ISOLN))
      //  || (item == 4 && (cal_status & CALSTAT_THRU)))
      if ( (item == 0 && (cal_status & (CALSTAT_OPEN  | CALSTAT_ER)))
        || (item == 1 && (cal_status & (CALSTAT_SHORT | CALSTAT_ES)))
        || (item == 2 && (cal_status & (CALSTAT_LOAD  | CALSTAT_ED)))
        || (item == 3 && (cal_status & (CALSTAT_ISOLN | CALSTAT_EX)))
        || (item == 4 && (cal_status & (CALSTAT_THRU  | CALSTAT_ET))))
      {
         //*bg = DEFAULT_MENU_ACTIVE_COLOR;
         ////*fg = config.menu_normal_color;
         //*fg = DEFAULT_MENU_ACTIVE_TEXT_COLOR;   // test only
         return true;
      }
   }

   if (menu == menu_stimulus)
   {
      if (item == (items - 1) && !(sweep_mode & SWEEP_ENABLE))
         modify = true;      // sweep paused
   }

   if (menu == menu_cal)
   {
      if (item == (items - 1) && (cal_status & CALSTAT_APPLY))
         modify = true;      // correction enabled
   }

   if (menu == menu_bandwidth)
   {
      if (menu_bandwidth[item].data == config.bandwidth)
         modify = true;
   }

   #ifdef POINTS_SET_51
      if (menu == menu_sweep_points)
      {
         if (menu_sweep_points[item].data == sweep_points)
            modify = true;
      }
   #endif

   if (menu == menu_transform)
   {
      if (     (item == 0 && (domain_mode & DOMAIN_MODE) == DOMAIN_TIME)
            || (item == 1 && (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_IMPULSE)
            || (item == 2 && (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_STEP)
            || (item == 3 && (domain_mode & TD_FUNC) == TD_FUNC_BANDPASS))
      {
         modify = true;
      }
   }

   if (menu == menu_transform_window)
   {
      if ((item == 0 && (domain_mode & TD_WINDOW) == TD_WINDOW_MINIMUM)
       || (item == 1 && (domain_mode & TD_WINDOW) == TD_WINDOW_NORMAL)
       || (item == 2 && (domain_mode & TD_WINDOW) == TD_WINDOW_MAXIMUM))
      {
         modify = true;
      }
   }

   if (menu == menu_recall)
   {
      // if (active_props == &current_props && config.current_id == item)
      if (config.current_id == item)
      {
         //*bg = DEFAULT_MENU_ACTIVE_COLOR;
         ////*fg = config.menu_normal_color;
         //*fg = DEFAULT_MENU_ACTIVE_TEXT_COLOR;   // test only
         return true;
      }
   }

   if (modify)
   {
      *bg = DEFAULT_MENU_ACTIVE_COLOR;
      //*fg = config.menu_normal_color;
      *fg = DEFAULT_MENU_ACTIVE_TEXT_COLOR;   // test only
   }

   return modify;
}

static void draw_menu_buttons(const menuitem_t *menu)
{
   unsigned int i;
   int k;
   char s[3][18];

   const int items = menu_number_items(menu);

   // count how many buttons we're going to draw and where we're going to draw them
   menu_button.count = 0;
   for (i = 0; i < ARRAYSIZE(menu_button.pos); i++)
   {
      if (menu[i].type == MT_NONE)
         break;
      // remember where we are drawing the button
      menu_button.pos[i].x = LCD_WIDTH - MENU_BUTTON_WIDTH;
      menu_button.pos[i].y = 0;
      menu_button.pos[i].w = MENU_BUTTON_WIDTH;
      menu_button.pos[i].h = 0;
      menu_button.count++;
   }

   if (menu_button.count == 0)
      return;

   {   // vertically spread the menu buttons by resizing them
      int y = OFFSETY;
      const int bh = (LCD_HEIGHT - OFFSETY) / menu_button.count;
      for (i = 0; i < ARRAYSIZE(menu_button.pos); i++)
      {
         menu_button.pos[i].y = y;
         menu_button.pos[i].h = bh;
         y += bh;
      }
   }

   for (i = 0; i < menu_button.count; i++)
   {
      const char *label = menu[i].label;
      int text_lines = 0;

      int bx = menu_button.pos[i].x;
      int by = menu_button.pos[i].y;
      int bw = menu_button.pos[i].w;
      int bh = menu_button.pos[i].h;

      if (menu[i].type == MT_BLANK)
         continue;

      const bool highlight = (ui_mode == UI_MENU && (int)i == selection) ? true : false;

      const int memory_index = ((menu == menu_recall || menu == menu_save_cal) && (int)i < items) ? (int)i : -1;

      if (memory_index >= 0)
      {	// it's a calibration memory button
         const properties_t *ref = caldata_ref(memory_index);
         if (ref == NULL)
         {   // slot/memory is empty
//          plot_printf(s[0], sizeof(s[0]), "RECALL %u", i);
            strcpy(s[0], menu[i].label);
            strcpy(s[1], "Empty");
            text_lines = 2;
         }
         else
         {   // show the slot/memory details
            const properties_t *cal = caldata_ref(memory_index);
//            plot_printf(s[0], sizeof(s[0]), "%qHz", cal->_frequency0);
//            plot_printf(s[1], sizeof(s[1]), "%qHz", cal->_frequency1);
            plot_printf(s[0], sizeof(s[0]), "%p", cal->_frequency0);
            plot_printf(s[1], sizeof(s[1]), "%p", cal->_frequency1);

            int k = 0;
            s[2][k++] = (cal->_cal_status & CALSTAT_ER) ? 'R' : '-';
            s[2][k++] = (cal->_cal_status & CALSTAT_ES) ? 'S' : '-';
            s[2][k++] = (cal->_cal_status & CALSTAT_ED) ? 'D' : '-';
            s[2][k++] = (cal->_cal_status & CALSTAT_EX) ? 'X' : '-';
            s[2][k++] = (cal->_cal_status & CALSTAT_ET) ? 'T' : '-';
            s[2][k] = '\0';

            text_lines = 3;
         }
      }
      else
      if ((menu == menu_stimulus) && (int)i < 5)
      {
         strcpy(s[0], menu[i].label);

      	uint32_t value = 0;
      	switch (i)
      	{
      		case 0: value = get_sweep_frequency(ST_START);  break;
      		case 1: value = get_sweep_frequency(ST_STOP);   break;
      		case 2: value = get_sweep_frequency(ST_CENTER); break;
      		case 3: value = get_sweep_frequency(ST_SPAN);   break;
      		case 4: value = get_sweep_frequency(ST_CW);     break;
      		default: break;
      	}
         //plot_printf(s[1], sizeof(s[1]), "%qHz", value);
         plot_printf(s[1], sizeof(s[1]), "%p", value);

         text_lines = 2;
      }
      else
      {
         text_lines = (label[0] == '\3') ? 3 : (label[0] == '\2') ? 2 : 1;
         if (text_lines <= 1)
         {
            strcpy(s[0], label);
         }
         else
         {
            int m = 1;
            for (k = 0; k < text_lines; k++)
            {
               const char *str = &label[m];
               strcpy(s[k], str);
               m += strlen(str) + 1;
            }
         }
      }

//    uint16_t bg = highlight ? config.menu_active_color : config.menu_normal_color;
//    uint16_t fg = DEFAULT_MENU_TEXT_COLOR;
      uint16_t bg = highlight ? DEFAULT_MENU_SELECT_COLOR      : DEFAULT_MENU_COLOR;        // test only
      uint16_t fg = highlight ? DEFAULT_MENU_SELECT_TEXT_COLOR : DEFAULT_MENU_TEXT_COLOR;   // test only

      draw_button(bx, by, bw, bh, 2, highlight, bg);

      const bool modified = menu_item_modify_attribute(menu, i, &fg, &bg);
      bx += 6;
      bw -= 11;

      // button background
      ili9341_fill(bx, by + 4, bw, bh - 9, bg);

      by += (bh - 1 - ((FONT_GET_HEIGHT + 3) * text_lines)) / 2;
      bh  = (FONT_GET_HEIGHT + 3) * text_lines;

      // button background
//      ili9341_fill(bx, by, bw, bh, bg);

      // button text
      ili9341_set_background(bg);
      ili9341_set_foreground(fg);
      for (k = 0; k < text_lines; k++)
         ili9341_drawstring(s[k], bx + 2, by + 2 + ((FONT_GET_HEIGHT + 3) * k), false);

      if (memory_index >= 0)
      {	// it's a calibration memory button
         plot_printf(s[0], sizeof(s[0]), "%d", memory_index);
      	ili9341_drawstring(s[0], bx + 2 + bw - (FONT_MAX_WIDTH * 1), by + 2 + ((FONT_GET_HEIGHT + 3) * (text_lines - 1)), false);
      }

      if (modified)
      {
         if (menu == menu_calop)
         {   // add a tick mark to say it's been done
            //if (text_lines <= 1)
            {
               ili9341_drawstring("Done", bx + 2 + bw - (FONT_MAX_WIDTH * 4), by + 2 + ((FONT_GET_HEIGHT + 3) * (text_lines / 2)), false);
            }
         }
      }
   }
}

static int menu_item_touch(int touch_x, int touch_y)
{   // return the menu selection that is currently being touched
   unsigned int i;
/*
   const menuitem_t *menu = menu_stack[menu_current_level];
   for (i = 0; i < MENU_BUTTON_MAX; i++)
   {
      if (menu[i].type == MT_NONE)
         break;
      if (menu[i].type == MT_BLANK)
         continue;

      const int y = MENU_BUTTON_HEIGHT * i;
      if (touch_y >= y && touch_y < (y + MENU_BUTTON_HEIGHT) && touch_x >= (LCD_WIDTH - MENU_BUTTON_WIDTH))
         return i;   // found the button being touched
   }
*/
   for (i = 0; i < menu_button.count; i++)
   {
      const uint16_t bx = menu_button.pos[i].x;
      const uint16_t by = menu_button.pos[i].y;
      const uint16_t bw = menu_button.pos[i].w;
      const uint16_t bh = menu_button.pos[i].h;
      if (   touch_x >= bx && touch_x < (bx + bw) && touch_y >= by && touch_y < (by + bh))
         return i;   // found the button being touched
   }

   // no button being touched
   return -1;
}

// ****************************************************************************************

static void draw_menu(void)
{
   draw_menu_buttons(menu_stack[menu_current_level]);
}

static void erase_menu_buttons(void)
{
//   ili9341_fill(LCD_WIDTH - MENU_BUTTON_WIDTH, 0, MENU_BUTTON_WIDTH, MENU_BUTTON_HEIGHT * MENU_BUTTON_MAX, DEFAULT_BG_COLOR);
//   ili9341_fill(LCD_WIDTH - MENU_BUTTON_WIDTH, 0, MENU_BUTTON_WIDTH, LCD_HEIGHT, DEFAULT_MENU_BACK_COLOR);
   ili9341_fill(LCD_WIDTH - MENU_BUTTON_WIDTH, 0, MENU_BUTTON_WIDTH, LCD_HEIGHT, DEFAULT_BG_COLOR);
}

static void erase_numeric_input(void)
{
   ili9341_fill(0, LCD_HEIGHT - NUM_INPUT_HEIGHT, LCD_WIDTH, NUM_INPUT_HEIGHT, DEFAULT_BG_COLOR);
}

static void leave_ui_mode()
{
   if (ui_mode == UI_MENU)
   {
      request_to_draw_cells_behind_menu();
      erase_menu_buttons();
   }
   else
   if (ui_mode == UI_NUMERIC)
   {
      request_to_draw_cells_behind_numeric_input();
      erase_numeric_input();
   }
   menu_button.count = 0;
   draw_frequencies();
}

static void fetch_numeric_target(void)
{
   const int current_trace = config.current_trace;

   switch (keypad_mode)
   {
      default:
      case KM_NONE:            return;
      case KM_START:           uistat.value = get_sweep_frequency(ST_START);          break;
      case KM_STOP:            uistat.value = get_sweep_frequency(ST_STOP);           break;
      case KM_CENTER:          uistat.value = get_sweep_frequency(ST_CENTER);         break;
      case KM_SPAN:            uistat.value = get_sweep_frequency(ST_SPAN);           break;
      case KM_CW:              uistat.value = get_sweep_frequency(ST_CW);             break;
      case KM_SCALE:           uistat.value = get_trace_scale(current_trace)  * 1000; break;
      case KM_REFPOS:          uistat.value = get_trace_refpos(current_trace) * 1000; break;
      case KM_EDELAY:          uistat.value = get_electrical_delay();                 break;
      case KM_VELOCITY_FACTOR: uistat.value = (int32_t)(velocity_factor * 1000);      break;
      case KM_SCALEDELAY:      uistat.value = get_trace_scale(current_trace) * 1e12;  break;
		#ifdef __USE_SD_CARD_AUTO_SAVE_PERIOD__
      	case KM_AUTO_SAVE_SECS:  uistat.value = auto_save_secs;                      break;
		#endif
   }

   int32_t x = uistat.value;
   int n = 0;
   for (; (x >= 10 || x <= -10) && n < 9; n++)
      x /= 10;

   uistat.digit          = n;
   uistat.digit_mode     = true;
   uistat.original_value = uistat.value;
}

static void set_numeric_value(void)
{
   const int current_trace = config.current_trace;

   switch (keypad_mode)
   {
      default:
      case KM_NONE:               break;
      case KM_START:              set_sweep_frequency(ST_START, uistat.value);     break;
      case KM_STOP:               set_sweep_frequency(ST_STOP, uistat.value);      break;
      case KM_CENTER:             set_sweep_frequency(ST_CENTER, uistat.value);    break;
      case KM_SPAN:               set_sweep_frequency(ST_SPAN, uistat.value);      break;
      case KM_CW:                 set_sweep_frequency(ST_CW, uistat.value);        break;
      case KM_SCALE:              set_trace_scale( current_trace, (float)uistat.value / 1000.0f);  break;
      case KM_REFPOS:             set_trace_refpos(current_trace, (float)uistat.value / 1000.0f);  break;
      case KM_EDELAY:             set_electrical_delay(uistat.value);              break;
      case KM_VELOCITY_FACTOR:    velocity_factor = (float)uistat.value / 1000.0f; break;
		#ifdef __USE_SD_CARD_AUTO_SAVE_PERIOD__
      	case KM_AUTO_SAVE_SECS:  auto_save_secs = uistat.value;                   break;
		#endif
   }
}

static void draw_numeric_area(void)
{
   char buf[10];
   plot_printf(buf, sizeof buf, "%9d", uistat.value);
   draw_numeric_input(buf);
}

static void show_original_keypad_value(void)
{
   int x         = 5 + (10 * (FONT_WIDTH * 2)) + 2;
   const int y   = LCD_HEIGHT - NUM_INPUT_HEIGHT + 5;
   char buf[16];

   // wipe all previous text
// ili9341_fill(x, y, LCD_WIDTH - 2 - x, NUM_FONT_GET_HEIGHT, config.menu_normal_color);
   ili9341_fill(x, y, LCD_WIDTH - 2 - x, NUM_FONT_GET_HEIGHT, DEFAULT_MENU_COLOR);   // test only

   switch (keypad_mode)
   {
      case KM_START:
      case KM_STOP:
      case KM_CENTER:
      case KM_SPAN:
      case KM_CW:
         plot_printf(buf, sizeof(buf), "%qHz", uistat.original_value);
         break;
      case KM_SCALE:
      case KM_REFPOS:
         plot_printf(buf, sizeof(buf), "%0.1f", (float)uistat.original_value / 1000.0f);
         break;
      case KM_EDELAY:
         plot_printf(buf, sizeof(buf), "%d ps", uistat.original_value);
         break;
      case KM_SCALEDELAY:
         plot_printf(buf, sizeof(buf), "%d ps", uistat.original_value);
         break;
      case KM_VELOCITY_FACTOR:
         plot_printf(buf, sizeof(buf), "%0.1f %%", (float)uistat.original_value / 10.0f);
         break;
      case KM_AUTO_SAVE_SECS:
      	if (uistat.original_value == 0)
      		plot_printf(buf, sizeof(buf), "disabled");
      	else
      		plot_printf(buf, sizeof(buf), "%d", uistat.original_value);
         break;
   }

// ili9341_set_background(config.menu_normal_color);
   ili9341_set_background(DEFAULT_MENU_COLOR);   // test only
   ili9341_set_foreground(DEFAULT_MENU_TEXT_DIM_COLOR);
   ili9341_drawstring_size(buf, x + 10, LCD_HEIGHT - NUM_INPUT_HEIGHT + 9, 2, false);
}

static void ui_mode_menu(void)
{
   if (ui_mode == UI_MENU)
      return;

   ui_mode           = UI_MENU;
   menu_button.count = 0;
   area_width        = AREA_WIDTH_NORMAL - MENU_BUTTON_WIDTH;
   area_height       = AREA_HEIGHT_NORMAL;

   ensure_selection();
   erase_menu_buttons();
   draw_menu();
}

static void ui_mode_numeric(int _keypad_mode)
{
   if (_keypad_mode < 0)
      return;

   if (ui_mode == UI_NUMERIC)
      return;

   leave_ui_mode();

   // keypads array
   keypad_mode       = _keypad_mode;

   ui_mode           = UI_NUMERIC;
   menu_button.count = 0;
   area_width        = AREA_WIDTH_NORMAL;
   area_height       = LCD_HEIGHT - NUM_INPUT_HEIGHT;

   fetch_numeric_target();

   draw_numeric_area_frame();
   draw_numeric_area();
}

static void ui_mode_keypad(int _keypad_mode)
{
   if (_keypad_mode < 0)
      return;

   if (ui_mode == UI_KEYPAD)
      return;

   // keypads array
   keypad_mode        = _keypad_mode;
   keypads            = keypads_mode_tbl[_keypad_mode];
   keypads_last_index = 0;
   while (keypads[keypads_last_index + 1].c >= 0)
      keypads_last_index++;

   ui_mode           = UI_KEYPAD;
   menu_button.count = 0;
   area_width        = AREA_WIDTH_NORMAL - MENU_BUTTON_WIDTH;
   area_height       = LCD_HEIGHT - NUM_INPUT_HEIGHT;

   //draw_menu();

   //ili9341_set_foreground(config.menu_normal_color);
   ili9341_set_foreground(DEFAULT_MENU_COLOR);   // test only
   ili9341_set_background(DEFAULT_BG_COLOR);
   ili9341_clear_screen();

   draw_keypad();

   fetch_numeric_target();

   draw_numeric_area_frame();
   draw_numeric_input("");
   show_original_keypad_value();
}

static void ui_mode_normal(void)
{
   if (ui_mode == UI_NORMAL)
      return;

   area_width  = AREA_WIDTH_NORMAL;
   area_height = AREA_HEIGHT_NORMAL;

   leave_ui_mode();

   ui_mode = UI_NORMAL;
}

static void lever_move_marker(uint16_t status)
{
   do
   {
      const int marker = active_marker;
      if (marker >= 0 && marker < MARKERS_MAX)
      {
         if (marker_enabled(marker))
         {
            int index = marker_index(marker);

            if ((status & EVT_DOWN) && index > 0)
               index--;

            if ((status & EVT_UP) && index < (sweep_points - 1))
               index++;

            if (index != marker_index(marker))
            {
               set_marker_index(marker, index);
               redraw_marker(marker);
            }
         }
      }

      status = btn_wait_release();

   } while (status != 0);
}

static void lever_search_marker(uint16_t status)
{
   const int marker = active_marker;
   if (marker >= 0 && marker < MARKERS_MAX)
   {
      const int index = marker_index(marker);

      int i = -1;

      if (status & EVT_DOWN)
         i = marker_search_left(index);
      else
      if (status & EVT_UP)
         i = marker_search_right(index);

      if (i >= 0)
      {
         set_marker_index(marker, i);
         redraw_marker(marker);
      }
   }
}

// ex. 10942 -> 10000
//      6791 ->  5000
//       341 ->   200
static uint32_t step_round(uint32_t v)
{
   uint32_t x = 1;

   // decade step
   for (x = 1; (x * 10) < v; x *= 10);

   // 1-2-5 step
   if ((x * 2) > v)
      return x;

   if ((x * 5) > v)
      return x * 2;

   return x * 5;
}

static void lever_zoom_span(uint16_t status)
{
   uint32_t span = get_sweep_frequency(ST_SPAN);
   if (status & EVT_UP)
   {
      span = step_round(span - 1);
   }
   else
   if (status & EVT_DOWN)
   {
      span = step_round(span + 1);
      span = step_round(span * 3);
   }
   set_sweep_frequency(ST_SPAN, span);
}

static void lever_move(int status, int mode)
{
   const uint32_t center = get_sweep_frequency(mode);
   uint32_t span = get_sweep_frequency(ST_SPAN);
   span = step_round(span / 3);
   if (status & EVT_UP)
      set_sweep_frequency(mode, center + span);
   else
   if (status & EVT_DOWN)
      set_sweep_frequency(mode, center - span);
}

#define STEPRATIO 0.2f

static void lever_edelay(uint16_t status)
{
   float value = get_electrical_delay();

   float ratio = STEPRATIO;
   if (value < 0.0f)
      ratio = -ratio;

   if (status & EVT_UP)
      value = (1.0f - ratio) * value;
   else
   if (status & EVT_DOWN)
      value = (1.0f + ratio) * value;

   set_electrical_delay(value);
}

static void ui_process_normal(void)
{
   const uint16_t status = btn_check();
   if (status == 0)
      return;

   if (status & EVT_BUTTON_SINGLE_CLICK)
   {
      ui_mode_menu();
      return;
   }

   switch (uistat.lever_mode)
   {
      case LM_MARKER:
         lever_move_marker(status);
         break;

      case LM_SEARCH:
         lever_search_marker(status);
         break;

      case LM_CENTER:
         lever_move(status, FREQ_IS_STARTSTOP() ? ST_START : ST_CENTER);
         break;

      case LM_SPAN:
         if (FREQ_IS_STARTSTOP())
            lever_move(status, ST_STOP);
         else
            lever_zoom_span(status);
         break;

      case LM_EDELAY:
         lever_edelay(status);
         break;
   }
}

static void ui_process_menu(void)
{
   uint8_t status = btn_check();
   if (status == 0)
      return;

   if (status & EVT_BUTTON_SINGLE_CLICK)
   {
      menu_invoke(selection);
      return;
   }

   do
   {
      if (status & EVT_UP)
      {
         // close menu if next item is sentinel
         if (menu_stack[menu_current_level][selection + 1].type == MT_NONE)
         {
            ui_mode_normal();
            return;
         }
         selection++;
      }

      if (status & EVT_DOWN)
      {
         if (selection == 0)
         {
            ui_mode_normal();
            return;
         }
         selection--;
      }

      draw_menu();

      chThdSleepMilliseconds(80);   // key repeat speed

      status = btn_wait_release();
   } while (status != 0);
}

static int keypad_click(int key)
{
   const int current_trace = config.current_trace;

   const int c = keypads[key].c;

   if (keypads[key].c == KP_NONE)
   	return KP_CONTINUE;

   if ((c >= KP_X1 && c <= KP_G) || c == KP_N || c == KP_P)
   {
      if (kp_buf[0] == '\0')
         return KP_DONE;

      int32_t scale = 1;
      if (c >= KP_X1 && c <= KP_G)
      {
         int n = c - KP_X1;
         while (n-- > 0)
            scale *= 1000;
      }
      else
      if (c == KP_N)
         scale *= 1000;

      // numeric input done
      double value = my_atof(kp_buf) * scale;

      switch (keypad_mode)
      {
         case KM_NONE:
         default:
            break;
         case KM_START:
            if (value > 0)
               set_sweep_frequency(ST_START, value);
            break;
         case KM_STOP:
            if (value > 0)
               set_sweep_frequency(ST_STOP, value);
            break;
         case KM_CENTER:
            if (value > 0)
               set_sweep_frequency(ST_CENTER, value);
            break;
         case KM_SPAN:
            if (value > 0)
               set_sweep_frequency(ST_SPAN, value);
            break;
         case KM_CW:
            if (value > 0)
               set_sweep_frequency(ST_CW, value);
            break;
         case KM_SCALE:
            if (value > 0)
               set_trace_scale(current_trace, value);
            break;
         case KM_REFPOS:
            set_trace_refpos(current_trace, value);
            break;
         case KM_EDELAY:
            set_electrical_delay(value); // pico seconds
            break;
         case KM_VELOCITY_FACTOR:
            if (value > 0)
            {   // assume it's a fraction if they started with a '.', otherwise assume it's a percentage value
               if (value > 100)
                  value = 100;
               velocity_factor = (kp_buf[0] == '.') ? value : value / 100;
            }
            break;
         case KM_SCALEDELAY:
            set_trace_scale(current_trace, value * 1e-12); // pico second
            break;
			#ifdef __USE_SD_CARD_AUTO_SAVE_PERIOD__
         	case KM_AUTO_SAVE_SECS:
         		{
         			int secs  = (int)value;
         			if (secs <= 0)
         			{
         				secs = 0;
         			}
         			else
         			{
         				if (secs < 3) secs = 3;                // min 3 seconds
         				else
         				if (secs > (60 * 20)) secs = 60 * 20;  // max 20 minutes
         			}
         			auto_save_secs = secs;
         			break;
         		}
			#endif
      }

      return KP_DONE;
   }

   if (c <= 9 && kp_index < NUMINPUT_LEN)
   {
      kp_buf[kp_index++] = '0' + c;
   }
   else
   if (c == KP_PERIOD && kp_index < NUMINPUT_LEN)
   {
      // check period in former input
      int j;
      for (j = 0; j < kp_index && kp_buf[j] != '.'; j++);

      // append period if there are no period
      if (kp_index == j)
         kp_buf[kp_index++] = '.';
   }
   else
   if (c == KP_MINUS)
   {
      if (kp_index == 0)
         kp_buf[kp_index++] = '-';
   }
   else
   if (c == KP_BS)
   {
      if (kp_index == 0)
         return KP_CANCEL;
      kp_index--;
   }

   // null terminate
   kp_buf[kp_index] = '\0';

   if (kp_index == 0 && ui_mode == UI_KEYPAD)
      show_original_keypad_value();
   else
      draw_numeric_input(kp_buf);

   return KP_CONTINUE;
}

static void numeric_apply_touch(int touch_x, int touch_y)
{
   if (touch_x < 64)
   {
      ui_mode_normal();
      return;
   }

   if (touch_x > (64 + (9 * 20) + 8 + 8))
   {
      ui_mode_keypad(keypad_mode);
      ui_process_keypad();
      return;
   }

   if (touch_y > LCD_HEIGHT - 40)
   {
      const int n = 9 - (touch_x - 64) / 20;
      uistat.digit = n;
      uistat.digit_mode = true;
   }
   else
   {
      int n;
      int step = (touch_y < 100) ? 1 : -1;
      for (n = uistat.digit; n > 0; n--)
         step *= 10;
      uistat.value += step;
   }

   draw_numeric_area();

   touch_wait_release();

   uistat.digit_mode = false;
   draw_numeric_area();
}

static void ui_process_numeric(void)
{
   uint8_t status = btn_check();
   if (status == 0)
      return;

   if (status == EVT_BUTTON_SINGLE_CLICK)
   {
      status = btn_wait_release();
      if (status & EVT_BUTTON_DOWN_LONG)
      {
         set_numeric_value();
         ui_mode_normal();
      }
      else
      if (status & EVT_BUTTON_SINGLE_CLICK)
      {
         uistat.digit_mode = !uistat.digit_mode;
         draw_numeric_area();
      }
      return;
   }

   do
   {
      bool updated = false;

      if (uistat.digit_mode)
      {
         if ((status & EVT_DOWN) && uistat.digit < 8)
         {
              uistat.digit++;
              updated = true;
         }
         else
         if ((status & EVT_UP) && uistat.digit > 0)
         {
              uistat.digit--;
              updated = true;
         }
      }
      else
      {
         int n;
         int32_t step = 1;
         for (n = uistat.digit; n > 0; n--)
            step *= 10;
         if (status & EVT_DOWN)
         {
            uistat.value += step;
              updated = true;
         }
         else
         if (status & EVT_UP)
         {
            uistat.value -= step;
              updated = true;
         }
      }

      if (updated)
         draw_numeric_area();

      status = btn_wait_release();
   } while (status != 0);
}

static void ui_process_keypad(void)
{
   adc_stop();

   kp_index = 0;

   while (TRUE)
   {
      const uint16_t status = btn_check();

      if (status & (EVT_UP | EVT_DOWN))
      {
         uint16_t s = status;
         do
         {
            if (s & EVT_UP)
               if (--selection < 0)
                  selection = keypads_last_index;
            if (s & EVT_DOWN)
               if (++selection > keypads_last_index)
                  selection = 0;
            draw_keypad();

            s = btn_wait_release();
         } while (s != 0);
      }

      if (status == EVT_BUTTON_SINGLE_CLICK)
      {
         const int s = keypad_click(selection);
         btn_wait_release();
         if (s)
            break;   // exit loop on done or cancel
      }

      const uint16_t tc = touch_check();
      if (tc == EVT_TOUCH_PRESSED || tc == EVT_TOUCH_DOWN)
      {   // touching/dragging
         int touch_x;
         int touch_y;
         touch_position(&touch_x, &touch_y);
         int i = 0;
         while (keypads[i].c >= 0)
         {
            const int x = KP_GET_X(keypads[i].x);
            const int y = KP_GET_Y(keypads[i].y);
            if (touch_x >= x && touch_x < (x + KP_WIDTH) && touch_y >= y && touch_y < (y + KP_HEIGHT))
            {
               if (selection != i)
               {   // draw focus
                  selection = i;
                  draw_keypad();
               }
               break;
            }
            i++;
         }
         if (i > keypads_last_index || keypads[i].c < 0)
         {
            if (selection >= 0)
            {
               selection = -1;
               draw_keypad();
            }
         }
      }
      else
      if (tc == EVT_TOUCH_RELEASED)
      {
         if (selection >= 0)
         {
            const int s = keypad_click(selection);
            btn_wait_release();
            if (s)
               break;
         }
         selection = -1;
         draw_keypad();
      }
   }

   redraw_frame();
   request_to_redraw_grid();

   ui_mode_normal();

   touch_start_watchdog();
}

#ifdef __USE_SD_CARD__
   //*****************************************************************************
   // Bitmap file header for 320x240 image 16bpp (v4 format allow set RGB mask)
   //*****************************************************************************

   #define BITMAP_FILEHEADER_SIZE  (14)
   #define BITMAP_INFOv4_SIZE      (56)
   #define BITMAP_SIZE             (LCD_WIDTH * LCD_HEIGHT * 2)
   #define BITMAP_BITS_PER_PIXEL   (16)
   #define BITMAP_NUMBER_OF_PLANES (1)
   #define BITMAP_BI_BITFIELDS     (3)

   static const uint8_t bmp_header_v4[BITMAP_FILEHEADER_SIZE + BITMAP_INFOv4_SIZE] =
   {
      // *******************
      // BITMAPFILEHEADER (14 byte size)

      'B', 'M',                                              // BM signature

      ((sizeof(bmp_header_v4) + BITMAP_SIZE) >>  0) & 0xff,  // File size = sizeof(bmp_header) + sizeof(bitmap image)
      ((sizeof(bmp_header_v4) + BITMAP_SIZE) >>  8) & 0xff,  //   "
      ((sizeof(bmp_header_v4) + BITMAP_SIZE) >> 16) & 0xff,  //   "
      ((sizeof(bmp_header_v4) + BITMAP_SIZE) >> 24) & 0xff,  //   "

      0x00, 0x00,                                            // reserved

      0x00, 0x00,                                            // reserved

      (sizeof(bmp_header_v4) >>  0) & 0xff,                  // Size of all headers = 14 + 56
      (sizeof(bmp_header_v4) >>  8) & 0xff,                  //   "
      (sizeof(bmp_header_v4) >> 16) & 0xff,                  //   "
      (sizeof(bmp_header_v4) >> 24) & 0xff,                  //   "

      // *******************
      // BITMAPINFOv4 (56 byte size)

      ((sizeof(bmp_header_v4) - BITMAP_FILEHEADER_SIZE) >>  0) & 0xff,    // data offset after this point (56)
      ((sizeof(bmp_header_v4) - BITMAP_FILEHEADER_SIZE) >>  8) & 0xff,    //  "
      ((sizeof(bmp_header_v4) - BITMAP_FILEHEADER_SIZE) >> 16) & 0xff,    //  "
      ((sizeof(bmp_header_v4) - BITMAP_FILEHEADER_SIZE) >> 24) & 0xff,    //  "

      (LCD_WIDTH  >>  0) & 0xff,                      // width
      (LCD_WIDTH  >>  8) & 0xff,                      //  "
      (LCD_WIDTH  >> 16) & 0xff,                      //  "
      (LCD_WIDTH  >> 24) & 0xff,                      //  "

      (LCD_HEIGHT >>  0) & 0xff,                      // height
      (LCD_HEIGHT >>  8) & 0xff,                      //  "
      (LCD_HEIGHT >> 16) & 0xff,                      //  "
      (LCD_HEIGHT >> 24) & 0xff,                      //  "

      (BITMAP_NUMBER_OF_PLANES >> 0) & 0xff,          // number of planes
      (BITMAP_NUMBER_OF_PLANES >> 8) & 0xff,          //   "

      (BITMAP_BITS_PER_PIXEL >> 0) & 0xff,            // bits per pixel
      (BITMAP_BITS_PER_PIXEL >> 8) & 0xff,            //   "

      (BITMAP_BI_BITFIELDS >>  0) & 0xff,             // Compression
      (BITMAP_BI_BITFIELDS >>  8) & 0xff,             //   "
      (BITMAP_BI_BITFIELDS >> 16) & 0xff,             //   "
      (BITMAP_BI_BITFIELDS >> 24) & 0xff,             //   "

      (BITMAP_SIZE >>  0) & 0xff,                     // Bitmap size = 320*240*2
      (BITMAP_SIZE >>  8) & 0xff,                     //   "
      (BITMAP_SIZE >> 16) & 0xff,                     //   "
      (BITMAP_SIZE >> 24) & 0xff,                     //   "

      ((3780) >>  0) & 0xff,                          // x Resolution (96 DPI = 96 * 39.3701 inches)
      ((3780) >>  8) & 0xff,                          //   "
      ((3780) >> 16) & 0xff,                          //   "
      ((3780) >> 24) & 0xff,                          //   "

      ((3780) >>  0) & 0xff,                          // y Resolution (96 DPI = 96 * 39.3701 inches)
      ((3780) >>  8) & 0xff,                          //   "
      ((3780) >> 16) & 0xff,                          //   "
      ((3780) >> 24) & 0xff,                          //   "

      0x00, 0x00, 0x00, 0x00,                         // Palette size
      0x00, 0x00, 0x00, 0x00,                         // Palette used

      // Extend v4 header data (color mask for RGB565)
      0b00000000, 0b11111000, 0b00000000, 0b00000000, // R mask = 0b11111000 00000000
      0b11100000, 0b00000111, 0b00000000, 0b00000000, // G mask = 0b00000111 11100000
      0b00011111, 0b00000000, 0b00000000, 0b00000000, // B mask = 0b00000000 00011111
      0b00000000, 0b00000000, 0b00000000, 0b00000000  // A mask = 0b00000000 00000000

      // *******************
   };

   static int made_screenshot(int touch_x, int touch_y)
   {
//    if (touch_y < HEIGHT || touch_x < FREQUENCIES_XPOS3 || touch_x > FREQUENCIES_XPOS2)
      if (touch_x >= OFFSETX && touch_y < LCD_HEIGHT - (FONT_STR_HEIGHT * 2))
         return FALSE;

      //uint32_t time = chVTGetSystemTimeX();
      //shell_printf("Screenshot\r\n");

      #if FF_USE_LFN >= 1
         const uint32_t tr = rtc_get_tr_bcd(); // TR read first
         const uint32_t dr = rtc_get_dr_bcd(); // DR read second
         plot_printf(fs_filename, FF_LFN_BUF, "VNA_%06X_%06X.bmp", dr, tr);
      #else
         plot_printf(fs_filename, FF_LFN_BUF, "%08X.bmp", rtc_get_FAT());
      #endif

      ili9341_set_foreground(DEFAULT_MENU_TEXT_COLOR);
      ili9341_set_background(DEFAULT_MENU_COLOR);   // test only

      FRESULT res = f_mount(fs_volume, "", 1);
      //shell_printf("Mount = %d\r\n", res);
      if (res == FR_OK)
      {
         // fs_volume, fs_file and fs_filename stored at end of spi_buffer!!!!!
			uint16_t *buf = (uint16_t *)spi_buffer;

         res = f_open(fs_file, fs_filename, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
         //shell_printf("Open %s, result = %d\r\n", fs_filename, res);
         if (res == FR_OK)
         {
            int y;
            UINT size;

            res = f_write(fs_file, bmp_header_v4, sizeof(bmp_header_v4), &size);
            for (y = LCD_HEIGHT - 1; y >= 0 && res == FR_OK; y--)
            {
               int i;
               ili9341_read_memory(0, y, LCD_WIDTH, 1, LCD_WIDTH, buf);
               for (i = 0; i < LCD_WIDTH; i++)
                  buf[i] = __REVSH(buf[i]); // swap byte order (example 0x10FF to 0xFF10)
               res = f_write(fs_file, buf, LCD_WIDTH * sizeof(uint16_t), &size);
            }

            if (res != FR_OK)
            {
            	res = f_close(fs_file);

					#if (FF_FS_MINIMIZE == 0)
        				// delete the file
						f_unlink(fs_filename);
					#endif
            }
            else
            	res = f_close(fs_file);
            //shell_printf("Close %d\r\n", res);

            //testLog();
         }

         ///time = chVTGetSystemTimeX() - time;
         //shell_printf("Total time: %dms (write %d byte/sec)\r\n", time/10, (LCD_WIDTH*LCD_HEIGHT*sizeof(uint16_t)+sizeof(bmp_header_v4))*10000/time);


         menu_show_message((res == FR_OK) ? "SCREEN SAVED    " : "Write failed  ", fs_filename);
      }
      else
      {
         menu_show_message("CARD MOUNT", "failed");
      }

      draw_frequencies();
      request_to_redraw_grid();

      touch_wait_release();

      chThdSleepMilliseconds(200);

      return TRUE;
   }
#endif

static void ui_process_lever(void)
{
   if (!(operation_requested & OP_LEVER))
      return;

   switch (ui_mode)
   {
      case UI_NORMAL:
         ui_process_normal();
         break;
      case UI_MENU:
         ui_process_menu();
         break;
      case UI_NUMERIC:
         ui_process_numeric();
         break;
      case UI_KEYPAD:
         ui_process_keypad();
         break;
   }
}

static void drag_marker(int t, int m)
{
   // wait touch release
   do
   {
      int touch_x;
      int touch_y;
      int index;
      touch_position(&touch_x, &touch_y);
      touch_x -= OFFSETX;
      touch_y -= OFFSETY;
      index = search_nearest_index(touch_x, touch_y, t);
      if (index >= 0)
      {
         set_marker_index(m, index);
         redraw_marker(m);
      }
   } while (touch_check() != EVT_TOUCH_RELEASED);
}

static int touch_pickup_marker(int touch_x, int touch_y)
{
   int m;

   touch_x -= OFFSETX;
   touch_y -= OFFSETY;

   for (m = 0; m < MARKERS_MAX; m++)
   {
      int t;

      if (!marker_enabled(m))
         continue;

      for (t = 0; t < TRACES_MAX; t++)
      {
         if (!trace[t].enabled)
            continue;

         int x;
         int y;
         marker_position(m, t, &x, &y);
         x -= touch_x;
         y -= touch_y;

         if (((x * x) + (y * y)) < (20 * 20))
         {
            if (active_marker != m)
            {
               previous_marker = active_marker;
               set_active_marker(m);
               redraw_marker(active_marker);
            }

            // select trace
            config.current_trace = t;
            select_lever_mode(LM_MARKER);

            // drag marker until release
            drag_marker(t, m);

            return TRUE;
         }
      }
   }

   return FALSE;
}

static int touch_lever_mode_select(int touch_x, int touch_y)
{
   if (touch_y > HEIGHT)
   {
      select_lever_mode(touch_x < FREQUENCIES_XPOS2 ? LM_CENTER : LM_SPAN);
      return 1;
   }

   if (touch_y < 25)
   {
      if (touch_x < FREQUENCIES_XPOS2 && get_electrical_delay() != 0.0)
         select_lever_mode(LM_EDELAY);
      else
         select_lever_mode(LM_MARKER);
      return 1;
   }

   return 0;
}

#ifdef SHOW_FINGER
   static const uint8_t finger_bitmap[] =
   {
      0b00111000,
      0b01000010,
      0b10011001,
      0b10100101,
      0b10100101,
      0b10100101,
      0b11111101,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10111101,
      0b10000001,
      0b10011001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b10000001,
      0b01111110
   };
#endif

static void ui_process_touch(void)
{
   int touch_x;
   int touch_y;

    const uint16_t status_touch = last_touch_status;

   if (!(operation_requested & OP_TOUCH) && status_touch == EVT_TOUCH_NONE)
      return;

   adc_stop();

   const uint16_t status = touch_check();

   if (status == EVT_TOUCH_PRESSED || status == EVT_TOUCH_DOWN)
   {   // touch started

      if (!ui_menu_user_present)
      {   // touch just started
         ui_menu_user_present = true;

         #ifdef SHOW_FINGER
            // show the touch indicator
            ili9341_set_background(DEFAULT_FINGER_COLOR);
            ili9341_set_foreground(DEFAULT_FG_COLOR);
            blit8BitWidthBitmap(1, 50, 8, sizeof(finger_bitmap), finger_bitmap);
         #endif
      }

      touch_position(&touch_x, &touch_y);

      switch (ui_mode)
      {
         case UI_NORMAL:
            if (touch_pickup_marker(touch_x, touch_y))
               break;      // marker selected

            #ifdef __USE_SD_CARD__
               if (made_screenshot(touch_x, touch_y))
                  break;
            #endif

            {
               const int lever = touch_lever_mode_select(touch_x, touch_y);
               if (lever)
               {
                  touch_wait_release();
                  break;      // lever mode (top and bottom screen)
               }
            }

            // pop-up the menu
            selection = -1;
            ui_mode_menu();

            touch_wait_release();
            break;

         case UI_MENU:
            {
               const int current_selection = menu_item_touch(touch_x, touch_y);

               if (selection != current_selection)
               {   // moved onto a different button .. follow the finger!
                  selection = current_selection;
                  draw_menu();
               }
            }
            return;

         case UI_NUMERIC:
            numeric_apply_touch(touch_x, touch_y);
            break;
      }
   }
   else
   if (status == EVT_TOUCH_RELEASED)
   {   // touch released

      if (selection >= 0)
      {   // released over a button
         const int i = selection;
           selection = -1;

           // select the menu option
          menu_invoke(i);
      }
      else
      {   // close the menu
          ui_mode_normal();
      }
   }

   if (ui_menu_user_present)
   {
      ui_menu_user_present = false;
      #ifdef SHOW_FINGER
         ili9341_fill(1, 50, 8, sizeof(finger_bitmap), DEFAULT_BG_COLOR);   // hide the touch indicator
      #endif
   }

   touch_start_watchdog();
}

void ui_process(void)
{
   ui_process_lever();
   ui_process_touch();
   operation_requested = OP_NONE;
}

// Triggered when the button is pressed or released. The LED4 is set to ON.
static void extcb1(EXTDriver *extp, expchannel_t channel)
{
   (void)extp;
   (void)channel;
   operation_requested |= OP_LEVER;
//   op_lever = true;
   //cur_button = READ_BUTTONS();
}

static const EXTConfig extcfg =
{
  {
    {EXT_CH_MODE_DISABLED, NULL},

    // no longer use button interrupts, noisy buttons often cause interrupt hell for CPU's
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
//  {EXT_CH_MODE_DISABLED, NULL},
//  {EXT_CH_MODE_DISABLED, NULL},
//  {EXT_CH_MODE_DISABLED, NULL},

    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

// this is called once every 5 millisecond
static void gpt_callback(GPTDriver *gpt_ptr)
{
  (void)gpt_ptr;
/*
   const uint16_t cur_button     = READ_BUTTONS();

   const uint16_t button_changed = last_button ^ cur_button;

   if (button_changed)
   {   // button state(s) have just changed state .. reset counter
      button_changed    = last_button ^ cur_button;
      last_button       = cur_button;
      timer_tick_button = timer_tick;
   }

   if ((timer_tick - timer_tick_button) >= (BUTTON_DEBOUNCE_MS / GPT_TIMER_MS))
   {  // buttons have been stable for long enough

      uint16_t status = 0;
      if (button_changed & BIT_PUSH)  status |= EVT_BUTTON_SINGLE_CLICK;
      if (button_changed & BIT_UP1)   status |= EVT_UP;
      if (button_changed & BIT_DOWN1) status |= EVT_DOWN;

//    if ((timer_tick - timer_tick_button) >= (BUTTON_DOWN_LONG_MS / GPT_TIMER_MS) && (cur_button & BIT_PUSH))
//       button_event |= EVT_BUTTON_DOWN_LONG;

//      EVT_BUTTON_SINGLE_CLICK
//      EVT_BUTTON_DOUBLE_CLICK
//      EVT_BUTTON_DOWN_LONG
//      EVT_UP
//      EVT_DOWN
//      EVT_REPEAT


      if (button_status != status)
      {
         button_status     = status;
         button_status_new = true;

         //operation_requested |= OP_LEVER;
         op_lever = true;
      }
   }
*/

   // ******************
/*
   const uint32_t debounce_tick = BUTTON_DEBOUNCE_MS / GPT_TIMER_MS;
   const uint32_t ticks         = timer_tick;
   const uint8_t  cur_button    = READ_BUTTONS();
   const uint8_t  changed       = last_button ^ cur_button;
   uint8_t        status        = 0;

   if (changed & BIT_PUSH)
   {
      if ((ticks - timer_tick_button) >= debounce_tick)
      {
         if (cur_button & BIT_PUSH)
         {  // button released
            status |= EVT_BUTTON_SINGLE_CLICK;
            if (inhibit_until_release)
            {
               inhibit_until_release = false;
               status = 0;
            }
         }
         timer_tick_button = ticks;
      }
   }

   if (changed & BIT_UP1)
   {
      if ((cur_button & BIT_UP1) && (ticks >= (timer_tick_button + debounce_tick)))
         status |= EVT_UP;
      timer_tick_button = ticks;
   }

   if (changed & BIT_DOWN1)
   {
      if ((cur_button & BIT_DOWN1) && (ticks >= timer_tick_button + debounce_tick))
         status |= EVT_DOWN;
      timer_tick_button = ticks;
   }

   last_button = cur_button;

   button_status = status;

   // ******************

   timer_tick++;
*/
}

// Touch panel timer check (check press frequency 20Hz)
static const GPTConfig gpt3cfg =
{
// 20,        // 20Hz timer clock.
   100,       // 100Hz timer clock.
// 1000,      // 1kHz timer clock
   gpt_callback,
   0x0020, // CR2:MMS=02 to output TRGO
   0
};

#if 0
   static void test_touch(uint16_T *x, uint16_t *y)
   {
      adc_stop(ADC1);
      *x = touch_measure_x();
      *y = touch_measure_y();
      touch_start_watchdog();
   }
#endif

void handle_touch_interrupt(void)
{
   operation_requested |= OP_TOUCH;
//   op_touch = true;
}

void ui_init()
{
   adc_init();

   // Activates the EXT driver 1
   extStart(&EXTD1, &extcfg);

   // setup a timer
   gptStart(&GPTD3, &gpt3cfg);
//   gptPolledDelay(    &GPTD3, 10);   // Small delay
   gptStartContinuous(&GPTD3, 10);
//   gptPolledDelay(    &GPTD3, (         100 * gpt3cfg.frequency) / 1000);  // 100ms delay
//   gptStartContinuous(&GPTD3, (GPT_TIMER_MS * gpt3cfg.frequency) / 1000);   // 'GPT_TIMER_MS'ms timer callback

  touch_start_watchdog();
}
