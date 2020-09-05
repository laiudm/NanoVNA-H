/*
 * Copyright (c) 2016-2017, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
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

#include <string.h>

#include <arm_math.h>
//#include <math.h>

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "si5351.h"
#include "nanovna.h"
#include "fft.h"
#ifdef USE_LC_MATCHING
   #include "lc_matching.h"
#endif
#ifdef  __USE_SD_CARD__
   #include "../FatFs/ff.h"
   #include "../FatFs/diskio.h"
#endif

#include <chprintf.h>

#ifndef VERSION
   #define VERSION "2020.Sep.04-1 by OneOfEleven from DiSlord 0.9.3.4"
#endif

#ifdef  __USE_SD_CARD__
   #if (SPI_BUFFER_SIZE < 2048)
      #error "SPI_BUFFER_SIZE for SD card support need size = 2048"
   #else
      // Fat file system work area (at the end of spi_buffer)
      FATFS *fs_volume   = (FATFS *)(((uint8_t*)(&spi_buffer[SPI_BUFFER_SIZE])) - sizeof(FATFS));
      // FatFS file object (at the end of spi_buffer)
      FIL   *fs_file     = (   FIL*)(((uint8_t*)(&spi_buffer[SPI_BUFFER_SIZE])) - sizeof(FATFS) - sizeof(FIL));
      // Filename object (at the end of spi_buffer)
      char  *fs_filename = (  char*)(((uint8_t*)(&spi_buffer[SPI_BUFFER_SIZE])) - sizeof(FATFS) - sizeof(FIL) - FF_LFN_BUF - 4);
   #endif

   #ifdef __USE_SD_CARD_AUTO_SAVE_PERIOD__
      uint16_t auto_save_secs  = 0;	// '0' = disabled, otherwise s2P files are saved every 'n' seconds to SD card
      systime_t auto_save_time = 0;
      #endif
#endif

 /*
 *  Shell settings
 */
// If need run shell as thread (use more amount of memory for stack), after
// enable this need reduce spi_buffer size, by default shell run in main thread
// #define VNA_SHELL_THREAD

static BaseSequentialStream *shell_stream = (BaseSequentialStream *)&SDU1;

#define VNA_SHELL_NEWLINE_STR    "\r\n"
#define VNA_SHELL_PROMPT_STR     "ch> "
#define VNA_SHELL_MAX_ARGUMENTS   4
#define VNA_SHELL_MAX_LENGTH     48      // max line length

// Shell command functions prototypes
typedef void (*vna_shellcmd_t)(int argc, char *argv[]);
#define VNA_SHELL_FUNCTION(command_name) static void command_name(int argc, char *argv[])

// Shell command line buffer, args, nargs, and function ptr
static char                    shell_line[VNA_SHELL_MAX_LENGTH];
static char                    *shell_args[VNA_SHELL_MAX_ARGUMENTS + 1];
static uint16_t                shell_nargs;
static volatile vna_shellcmd_t shell_function = 0;

//#define ENABLED_DUMP

// Allow get threads debug info
//#define ENABLE_THREADS_COMMAND

//#define ENABLE_GAMMA_COMMAND

// Info about NanoVNA, need fore soft
#define ENABLE_INFO_COMMAND

// Enable color command, allow change config color for traces, grid, menu
#define ENABLE_COLOR_COMMAND

// Enable I2C command for send data to AIC3204, used for debug
//#define ENABLE_I2C_COMMAND

//#define ENABLE_STAT_COMMAND

//#define ENABLE_TEST_COMMAND

//#define ENABLE_PORT_COMMAND

//#define ENABLE_TOUCH_COMMAND

static void apply_CH0_error_term_at(float gamma[2], int i);
static void apply_CH1_error_term_at(float gamma[2], int i);
static void apply_edelay(void);

static uint16_t get_sweep_mode(void);
static void     cal_interpolate(int s);
static void     update_frequencies(bool update_marker_indexes);
static void     set_frequencies(uint32_t start, uint32_t stop, uint16_t points);
static bool     sweep(bool break_on_operation, uint16_t sweep_mode, bool no_cal);
static void     transform_domain(void);
static  int32_t my_atoi(const char *p);
static uint32_t my_atoui(const char *p);

// Obsolete, always use interpolate
#define cal_auto_interpolate  TRUE

static uint8_t   drive_strength = SI5351_CLK_DRIVE_STRENGTH_8MA;
int8_t           sweep_mode     = SWEEP_ENABLE;
volatile uint8_t redraw_request = 0; // contains REDRAW_XXX flags

// sweep operation variables
volatile uint16_t wait_count = 0;

static uint16_t p_sweep = 0;
// ChibiOS i2s buffer must be 2x size (for process one while next buffer filled by DMA)
static int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];
// Sweep measured data
float measured[2][POINTS_COUNT][2];
uint32_t frequencies[POINTS_COUNT];

// Version text, displayed in Config->Version menu, also send by info command
const char *info_about[] =
{
  "Board: " BOARD_NAME,
  "2016-2020 Copyright @edy555",
  "Licensed under GPL. See: https://github.com/ttrftech/NanoVNA",
  "Version: " VERSION " [p:"define_to_STR(POINTS_COUNT)", IF:"define_to_STR(FREQUENCY_IF_FREQ_k)"k, ADC:"define_to_STR(AUDIO_ADC_FREQ_k)"k, Lcd:"define_to_STR(LCD_WIDTH)"x"define_to_STR(LCD_HEIGHT)"]",
  "Build Time: " __DATE__ " - " __TIME__,
  "Kernel: " CH_KERNEL_VERSION,
  "Compiler: " PORT_COMPILER_NAME,
  "Architecture: " PORT_ARCHITECTURE_NAME " Core Variant: " PORT_CORE_VARIANT_NAME,
  "Port Info: " PORT_INFO,
  "Platform: " PLATFORM_NAME,
  0
};

#define DSP_START(delay)     {wait_count = delay + config.bandwidth;}
#define DSP_WAIT_READY       while (wait_count) { if (operation_requested && break_on_operation) return false; __WFI(); }
#define DSP_WAIT             while (wait_count) {__WFI();}
#define RESET_SWEEP          {p_sweep = 0;}
#define DELAY_CHANNEL_CHANGE 2

#define SWEEP_CH0_MEASURE    1
#define SWEEP_CH1_MEASURE    2

static THD_WORKING_AREA(waThread1, 768);

static THD_FUNCTION(Thread1, arg)
{
   (void)arg;

   chRegSetThreadName("sweep");

   while (1)
   {
      bool completed = false;
      if ((sweep_mode & (SWEEP_ENABLE | SWEEP_ONCE)) && !ui_menu_user_present)   // 'ui_menu_user_present' so as to give full priority to the users input
      {
         completed = sweep(true, get_sweep_mode(), false);
         sweep_mode &= ~SWEEP_ONCE;
      }
      else
      {
         if (ui_menu_user_present)
            RESET_SWEEP;

         __WFI();
      }

      // Run Shell command in sweep thread
      if (shell_function)
      {
         shell_function(shell_nargs - 1, &shell_args[1]);
         shell_function = 0;
         osalThreadSleepMilliseconds(10);
         continue;
      }

      // Process UI inputs
      ui_process();

      #ifdef USE_LC_MATCHING
         if (completed)
            if (lc_match_process() >= 0)
               request_to_redraw_grid();
      #endif

      // Process collected data, calculate trace coordinates and plot only if scan completed
      if ((sweep_mode & SWEEP_ENABLE) && completed)
      {
         if (electrical_delay != 0)
            apply_edelay();

         if ((domain_mode & DOMAIN_MODE) == DOMAIN_TIME)
            transform_domain();

			#ifdef __USE_SD_CARD_AUTO_SAVE_PERIOD__
        		if (auto_save_secs > 0)
        		{
        			const systime_t now   = chVTGetSystemTime();
        			const systime_t delta = now - auto_save_time;
        			if (delta >= S2ST(auto_save_secs))
        			{	// save the current measurement to SD card
        				auto_save_time = now;
        				sd_save_sparam_file(SAVE_S2P_FILE, false);
        			}
        		}
			#endif

         // Prepare draw graphics, cache all lines, mark screen cells for redraw
         plot_into_index(measured);
         redraw_request |= REDRAW_CELLS | REDRAW_BATTERY;

         if (uistat.marker_tracking)
         {
            const int mi = marker_search();
            if (mi >= 0 && active_marker < MARKERS_MAX)
            {
               set_marker_index(active_marker, mi);
               redraw_request |= REDRAW_MARKER;
            }
         }
      }

      // plot trace and other indications as raster
      draw_all(completed);  // flush markmap only if scan completed to prevent remaining traces
   }
}

static inline void pause_sweep(void)
{
   sweep_mode &= ~SWEEP_ENABLE;
}

static inline void resume_sweep(void)
{
   sweep_mode |= SWEEP_ENABLE;
}

void toggle_sweep(void)
{
   sweep_mode ^= SWEEP_ENABLE;
}

static float bessel0(float x)
{
   const float eps = 0.0001f;

   float ret  = 0.0f;
   float term = 1.0f;
   float m    = 0.0f;

   while (term > (eps * ret))
   {
      ret += term;
      ++m;
      term *= (x * x) / (4.0f * m * m);
   }

   return ret;
}

static float kaiser_window(int k, int n, float beta)
{
   if (beta == 0.0f || n <= 1)
      return 1.0f;
   const float r = ((float)(2 * k) / (n - 1)) - 1.0f;
   return bessel0(beta * sqrtf(1.0f - (r * r))) / bessel0(beta);
}

static void transform_domain(void)
{
   int ch;
   int i;

   // use spi_buffer as temporary buffer
   // and calculate ifft for time domain
   float *tmp = (float *)spi_buffer;

   uint16_t window_size = POINTS_COUNT;
   uint16_t offset      = 0;
   uint8_t is_lowpass   = FALSE;
   switch (domain_mode & TD_FUNC)
   {
      case TD_FUNC_BANDPASS:
         offset      = 0;
         window_size = POINTS_COUNT;
         break;
      case TD_FUNC_LOWPASS_IMPULSE:
      case TD_FUNC_LOWPASS_STEP:
         is_lowpass  = TRUE;
         offset      = POINTS_COUNT;
         window_size = POINTS_COUNT * 2;
         break;
   }

   float beta = 0.0f;
   switch (domain_mode & TD_WINDOW)
   {
      case TD_WINDOW_MINIMUM:
//       beta = 0.0f;  // this is rectangular
         break;
      case TD_WINDOW_NORMAL:
         beta = 5.0f;
//       beta = 6.0f;
         break;
      case TD_WINDOW_MAXIMUM:
         beta = 13.0f;
         break;
   }

   uint16_t ch_mask = get_sweep_mode();

   for (ch = 0; ch < 2; ch++, ch_mask >>= 1)
   {
      if ((ch_mask & 1) == 0)
         continue;

      memcpy(tmp, measured[ch], sizeof(measured[0]));

      for (i = 0; i < POINTS_COUNT; i++)
      {
         const float w = kaiser_window(i + offset, window_size, beta);
         tmp[i * 2 + 0] *= w;
         tmp[i * 2 + 1] *= w;
      }

      for (i = POINTS_COUNT; i < FFT_SIZE; i++)
      {
         tmp[i * 2 + 0] = 0.0f;
         tmp[i * 2 + 1] = 0.0f;
      }

      if (is_lowpass)
      {
         for (i = 1; i < POINTS_COUNT; i++)
         {
            tmp[(FFT_SIZE - i) * 2 + 0] =  tmp[i * 2 + 0];
            tmp[(FFT_SIZE - i) * 2 + 1] = -tmp[i * 2 + 1];
         }
      }

      fft256_inverse((float(*)[2])tmp);

      memcpy(measured[ch], tmp, sizeof(measured[0]));

      for (i = 0; i < POINTS_COUNT; i++)
      {
         measured[ch][i][0] *= 1.0f / FFT_SIZE;
         if (is_lowpass)
            measured[ch][i][1]  = 0.0f;
         else
            measured[ch][i][1] *= 1.0f / FFT_SIZE;
      }

      if ((domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_STEP)
      {
         for (i = 1; i < POINTS_COUNT; i++)
            measured[ch][i - 0][0] += measured[ch][i - 1][0];
      }
   }
}

void streamWriteLoop(void *buffer, int size)
{
	const uint8_t *buf = (const uint8_t *)buffer;

	int max_loops = 100;

	int total_sent = 0;

	while (total_sent < size)
   {
		const int sent = (int)streamWrite(shell_stream, (void *)&buf[total_sent], size - total_sent);
		if (sent > 0)
      	total_sent += sent;
     	if (--max_loops <= 0)
     		break;
   }
}

// Shell commands output
int shell_printf(const char *fmt, ...)
{
   va_list ap;
   int formatted_bytes;
   va_start(ap, fmt);
      formatted_bytes = chvprintf(shell_stream, fmt, ap);
   va_end(ap);
   return formatted_bytes;
}

#if defined(__USE_SD_CARD__) && defined(__USE_SD_CARD_CMDS__)

   FRESULT cmd_sd_card_mount(void)
   {
      const FRESULT res = f_mount(fs_volume, "", 1);
      if (res != FR_OK)
         shell_printf("error: card not mounted\r\n");
        return res;
   }

   VNA_SHELL_FUNCTION(cmd_sd_list)
   {
      (void)argc;
      (void)argv;

      DIR dj;
      FILINFO fno;
      FRESULT res;

      shell_printf("sd_list:\r\n");

      res = cmd_sd_card_mount();
      if (res != FR_OK)
         return;

        res = f_findfirst(&dj, &fno, "", "*.*");
        while (res == FR_OK && fno.fname[0])
        {
           shell_printf("%s %u\r\n", fno.fname, fno.fsize);
           res = f_findnext(&dj, &fno);
        }
        f_closedir(&dj);
   }

   VNA_SHELL_FUNCTION(cmd_sd_readfile)
   {
      FRESULT res;
      char *buf = (char *)spi_buffer;

      if (argc < 1)
      {
         shell_printf("usage: sd_readfile {filename}\r\n");
         return;
      }

      const char *filename = argv[0];

      shell_printf("sd_readfile: %s\r\n", filename);

      res = cmd_sd_card_mount();
      if (res != FR_OK)
         return;

      res = f_open(fs_file, filename, FA_OPEN_EXISTING | FA_READ);
      if (res != FR_OK)
      {
         shell_printf("error: %s not opened\r\n", filename);
         return;
      }

      {	// number of bytes to follow (file size)
      	const uint32_t filesize = f_size(fs_file);
      	streamWriteLoop((void *)&filesize, 4);
      }

      // file data
      while (!f_eof(fs_file))
      {
         UINT size = 0;
         res = f_read(fs_file, buf, 128, &size);
         if (res != FR_OK || size == 0)
            break;
         //streamWrite(shell_stream, (void *)buf, size);
         streamWriteLoop((void *)buf, size);
      }

      res = f_close(fs_file);
   }
#endif

VNA_SHELL_FUNCTION(cmd_pause)
{
   (void)argc;
   (void)argv;
   pause_sweep();
}

VNA_SHELL_FUNCTION(cmd_resume)
{
   (void)argc;
   (void)argv;

   // restore frequencies array and cal
   update_frequencies(true);
   if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY))
      cal_interpolate(config.current_id);

   resume_sweep();
}

VNA_SHELL_FUNCTION(cmd_reset)
{
   (void)argc;
   (void)argv;

   if (argc >= 1)
   {
      if (strcmp(argv[0], "dfu") == 0)
      {
         shell_printf("Performing reset to DFU mode\r\n");
         enter_dfu();
         return;
      }
   }

   shell_printf("Performing reset\r\n");

   rccEnableWWDG(FALSE);
   WWDG->CFR = 0x60;
   WWDG->CR  = 0xff;

   // wait forever
   while (1);
}

static const uint8_t gain_table[][2] = {
    {  0,  0 },     // 1st:    0 ~  300MHz
    { 50, 50 },     // 2nd:  300 ~  900MHz
    { 75, 75 },     // 3th:  900 ~ 1500MHz
    { 85, 85 },     // 4th: 1500 ~ 2100MHz
    { 95, 95 },     // 5th: 2100 ~ 2700MHz
};

#define DELAY_GAIN_CHANGE 4

static int adjust_gain(uint32_t newfreq)
{
   int new_order = si5351_get_harmonic_lvl(newfreq);
   int old_order = si5351_get_harmonic_lvl(si5351_get_frequency());
   if (new_order != old_order)
   {
      tlv320aic3204_set_gain(gain_table[new_order][0], gain_table[new_order][1]);
      return DELAY_GAIN_CHANGE;
   }
   return 0;
}

static int set_frequency(uint32_t freq)
{
   int delay = adjust_gain(freq);
   uint8_t ds = drive_strength;
   delay += si5351_set_frequency(freq, ds);
   return delay;
}

//static int set_frequency(uint32_t freq)
//{
//	return si5351_set_frequency(freq, current_props._power);
//}

// Use macro, std isdigit more big
#define _isdigit(c) (c >= '0' && c <= '9')
// Rewrite universal standart str to value functions to more compact
//
// Convert string to int32
static int32_t my_atoi(const char *p)
{
  int32_t value = 0;
  uint32_t c;
  bool neg = false;

  if (*p == '-') {neg = true; p++;}
  if (*p == '+') p++;
  while ((c = *p++ - '0') < 10)
    value = value * 10 + c;
  return neg ? -value : value;
}

// Convert string to uint32
//  0x - for hex radix
//  0o - for oct radix
//  0b - for bin radix
//  default dec radix
static uint32_t my_atoui(const char *p)
{
   uint32_t value = 0;
   uint32_t radix = 10;
   uint32_t c;

   if (*p == '+')
      p++;

   if (*p == '0')
   {
      switch (p[1])
      {
         case 'x': radix = 16; break;
         case 'o': radix =  8; break;
         case 'b': radix =  2; break;
         default: goto calculate;
      }
      p+=2;
   }

calculate:
   while (1)
   {
      c = *p++ - '0';
      // c = to_upper(*p) - 'A' + 10
      if (c >= 'A' - '0')
         c = (c&(~0x20)) - ('A' - '0') + 10;
      if (c >= radix)
         return value;
      value = (value * radix) + c;
   }
}

double
my_atof(const char *p)
{
  int neg = FALSE;
  if (*p == '-')
    neg = TRUE;
  if (*p == '-' || *p == '+')
    p++;
  double x = my_atoi(p);
  while (_isdigit((int)*p))
    p++;
  if (*p == '.') {
    double d = 1.0f;
    p++;
    while (_isdigit((int)*p)) {
      d /= 10;
      x += d * (*p - '0');
      p++;
    }
  }
  if (*p == 'e' || *p == 'E') {
    p++;
    int exp = my_atoi(p);
    while (exp > 0) {
      x *= 10;
      exp--;
    }
    while (exp < 0) {
      x /= 10;
      exp++;
    }
  }
  if (neg)
    x = -x;
  return x;
}

//
// Function used for search substring v in list
// Example need search parameter "center" in "start|stop|center|span|cw" getStringIndex return 2
// If not found return -1
// Used for easy parse command arguments
static int get_str_index(char *v, const char *list)
{
  int i = 0;
  while (1) {
    char *p = v;
    while (1) {
      char c = *list;
      if (c == '|') c = 0;
      if (c == *p++) {
        // Found, return index
        if (c == 0) return i;
        list++;    // Compare next symbol
        continue;
      }
      break;  // Not equal, break
    }
    // Set new substring ptr
    while (1) {
      // End of string, not found
      if (*list == 0) return -1;
      if (*list++ == '|') break;
    }
    i++;
  }
  return -1;
}

VNA_SHELL_FUNCTION(cmd_offset)
{
  if (argc < 1)
  {
    shell_printf("usage: offset {frequency offset(Hz)}\r\n");
    return;
  }
  int32_t offset = my_atoi(argv[0]);
#ifdef USE_VARIABLE_OFFSET
  generate_DSP_Table(offset);
#endif
  si5351_set_frequency_offset(offset);
}

VNA_SHELL_FUNCTION(cmd_freq)
{
  if (argc < 1)
  {
    goto usage;
  }
  uint32_t freq = my_atoui(argv[0]);

  pause_sweep();
  set_frequency(freq);
  return;
usage:
  shell_printf("usage: freq {frequency(Hz)}\r\n");
}

VNA_SHELL_FUNCTION(cmd_power)
{
  if (argc < 1)
  {
    shell_printf("usage: power {0-3}\r\n");
    return;
  }
  drive_strength = my_atoui(argv[0]);
//  set_frequency(frequency);
}

#ifdef __USE_RTC__
   VNA_SHELL_FUNCTION(cmd_time)
   {
      (void)argc;
      (void)argv;

      uint32_t  dt_buf[2];

      dt_buf[0] = rtc_get_tr_bcd(); // TR should be read first for sync
      dt_buf[1] = rtc_get_dr_bcd(); // DR should be read second

      static const uint8_t idx_to_time[] = {6,5,4,2,  1,  0};
      static const char       time_cmd[] = "y|m|d|h|min|sec";
      //            0    1   2       4      5     6
      //time[]      = {sec, min, hr, 0, day, month, year, 0}
      uint8_t *time = (uint8_t *)dt_buf;

      if (argc < 1)
         goto showdt;

      if (argc < 2)
         goto usage;

      const int idx      = get_str_index(argv[0], time_cmd);

      const uint32_t val = my_atoui(argv[1]);
      if (idx < 0 || val > 99)
         goto usage;

      // Write byte value in struct
      time[idx_to_time[idx]] = ((val / 10) << 4) | (val % 10); // value in bcd format
      rtc_set_time(dt_buf[1], dt_buf[0]);
      return;

   usage:
      shell_printf("usage: time [%s] 0-99\r\n", time_cmd);
   showdt:
      shell_printf("20%02X/%02X/%02X %02X:%02X:%02X\r\n", time[6], time[5], time[4], time[2], time[1], time[0]);
   }
#endif

VNA_SHELL_FUNCTION(cmd_dac)
{
	int value;
	if (argc < 1)
	{
		shell_printf("usage: dac {value(0-4095)}\r\n"\
                 "current value: %d\r\n", config.dac_value);
		return;
	}
	value = my_atoui(argv[0]);
	config.dac_value = value;
	dacPutChannelX(&DACD2, 0, value);
}

VNA_SHELL_FUNCTION(cmd_threshold)
{
	uint32_t value;
	if (argc < 1)
	{
		shell_printf("usage: threshold {frequency in harmonic mode}\r\n"\
                 "current: %d\r\n", config.harmonic_freq_threshold);
		return;
	}
	value = my_atoui(argv[0]);
	config.harmonic_freq_threshold = value;
}

VNA_SHELL_FUNCTION(cmd_saveconfig)
{
	(void)argc;
	(void)argv;
	config_save();
	shell_printf("Config saved.\r\n");
}

VNA_SHELL_FUNCTION(cmd_clearconfig)
{
	if (argc < 1)
	{
		shell_printf("usage: clearconfig {protection key}\r\n");
		return;
	}

	if (strcmp(argv[0], "1234") != 0)
	{
		shell_printf("Key unmatched.\r\n");
		return;
	}

	clear_all_config_prop_data();
	shell_printf("Config and all cal data cleared.\r\n"\
               "Do reset manually to take effect. Then do touch cal and save.\r\n");
}

static struct
{
   int16_t rms[2];
   int16_t ave[2];
   int callback_count;
   #if 0
      int32_t last_counter_value;
      int32_t interval_cycles;
      int32_t busy_cycles;
   #endif
} stat;

VNA_SHELL_FUNCTION(cmd_data)
{
   int i;
   int sel = 0;
   float (*array)[2];

   if (argc >= 1)
      sel = my_atoi(argv[0]);

   if (sel == 0 || sel == 1)
      array = measured[sel];
   else
   if (sel >= 2 && sel < (2 + SAVEAREA_MAX))
      array = cal_data[sel - 2];
   else
   {
      shell_printf("usage: data [0, 1 or 2..%d]\r\n", 2 + SAVEAREA_MAX - 1);
      return;
   }

   for (i = 0; i < sweep_points; i++)
      shell_printf("%+f %+f\r\n", array[i][0], array[i][1]);
}

#ifdef ENABLED_DUMP
   VNA_SHELL_FUNCTION(cmd_dump)
   {
      int i;
      int j;
      int len;

      if (argc >= 1)
         dump_selection = my_atoi(argv[0]);

      dsp_start(3);
      dsp_wait();

      len = AUDIO_BUFFER_LEN;
      if (dump_selection == 1 || dump_selection == 2)
         len /= 2;
      for (i = 0; i < len; )
      {
         for (j = 0; j < 16; j++, i++)
            shell_printf("%04x ", 0xffff & (int)dump_buffer[i]);
         shell_printf("\r\n");
      }
   }
#endif

VNA_SHELL_FUNCTION(cmd_capture)
{
   // read pixel count at one time (PART*2 bytes required for read buffer)

   (void)argc;
   (void)argv;

   int y;

   #if SPI_BUFFER_SIZE < ((3 * LCD_WIDTH) + 1)
      #error "Low size of spi_buffer for cmd_capture"
   #endif

   // read 2 row pixel time (read buffer limit by 2/3 + 1 from spi_buffer size)
   for (y = 0; y < LCD_HEIGHT; y += 2)
   {
      // use uint16_t spi_buffer[2048] (defined in ili9341) for read buffer
      ili9341_read_memory(0, y, LCD_WIDTH, 2, 2 * LCD_WIDTH, spi_buffer);
      //streamWrite(shell_stream, (void *)spi_buffer, 2 * LCD_WIDTH * sizeof(uint16_t));
      streamWriteLoop(spi_buffer, 2 * LCD_WIDTH * sizeof(uint16_t));
   }
}

#ifdef ENABLE_GAMMA_COMMAND
   VNA_SHELL_FUNCTION(cmd_gamma)
   {
      float gamma[2];
      (void)argc;
      (void)argv;

      pause_sweep();
      chMtxLock(&mutex);
      wait_dsp(4);
      calculate_gamma(gamma);
      chMtxUnlock(&mutex);

      shell_printf("%d %d\r\n", gamma[0], gamma[1]);
   }
#endif

static void (*sample_func)(float *gamma) = calculate_gamma;

VNA_SHELL_FUNCTION(cmd_sample)
{
  static const char cmd_sample_list[] = "gamma|ampl|ref";

  if (argc < 1)
	  goto usage;

  switch (get_str_index(argv[0], cmd_sample_list))
  {
	  case 0:
		  sample_func = calculate_gamma;
		  return;
	  case 1:
		  sample_func = fetch_amplitude;
		  return;
	  case 2:
		  sample_func = fetch_amplitude_ref;
		  return;
	  default:
		  break;
  }

usage:
	shell_printf("usage: sample {%s}\r\n", cmd_sample_list);
}

#ifdef USE_INTEGRATOR
	VNA_SHELL_FUNCTION(cmd_integrator)
	{
		static const char cmd_value_list[] = "off|on";

		if (argc >= 1)
		{
			switch (get_str_index(argv[0], cmd_value_list))
			{
				case 0:	// off
					config.integrator_coeff = 1.0f;
					break;
				case 1:	// on
					config.integrator_coeff = INTEGRATOR_COEFF;
					break;
				default:
					break;
			}
		}

		shell_printf("integrator %s\r\n", (config.integrator_coeff >= 1.0f) ? "off" : "on");
		shell_printf("usage: integrator {%s}\r\n", cmd_value_list);
	}
#endif

#if 1
   config_t config;  // we fill it in at boot-up
#else
   config_t config =
   {
      .magic                   = CONFIG_MAGIC,
      .dac_value               = DEFAULT_DAC_VALUE,
      .grid_color              = DEFAULT_GRID_COLOR,
      .menu_normal_color       = DEFAULT_MENU_COLOR,
      .menu_active_color       = DEFAULT_MENU_ACTIVE_COLOR,
      .trace_color             = { DEFAULT_TRACE_1_COLOR, DEFAULT_TRACE_2_COLOR, DEFAULT_TRACE_3_COLOR, DEFAULT_TRACE_4_COLOR },
   // .touch_cal               = { 693, 605, 124, 171 },  // 2.4 inch LCD panel
      .touch_cal               = { 357, 563, 159, 198 },  // 2.8 inch LCD panel
   // .touch_cal               = { 252, 450, 111, 150 },  // 4.0" LCD
      .harmonic_freq_threshold = DEFAULT_FREQUENCY_THRESHOLD,
      .vbat_offset_mv          = DEFAULT_BATTERY_OFFSET_MV,
      .bandwidth               = BANDWIDTH_2000,
      .current_id              = 0,
      .current_trace           = 0,
		.integrator_coeff        = 1.0f,
      .flags                   = 0
   };
#endif

properties_t current_props;
properties_t *active_props = &current_props;

// default trace settings
static const trace_t def_trace[TRACES_MAX] =
{ // enable, type,       channel, reserved, scale, refpos
  {  1,      TRC_LOGMAG, 0,       0,        10.0,  NGRIDY - 1 },
  {  1,      TRC_LOGMAG, 1,       0,        10.0,  NGRIDY - 1 },
  {  1,      TRC_SMITH,  0,       0,         1.0,  0          },
  {  1,      TRC_PHASE,  1,       0,        90.0,  NGRIDY / 2 }
};

// Load propeties default settings
void load_default_properties(void)
{
   int i;

   // Magic add on caldata_save
   // current_props.magic             = CONFIG_MAGIC;

   current_props._frequency0          =     50000;    // start =  50kHz
   current_props._frequency1          = 900000000;    // end   = 900MHz

   current_props._sweep_points        = POINTS_SET_101; // Set default 101 points
   current_props._cal_status          = 0;

   // This data not loaded by default
   // current_props._cal_data[5][POINTS_COUNT][2];
   // =============================================
   current_props._electrical_delay    = 0.0;

   memcpy(current_props._trace, def_trace, sizeof(def_trace));

   for (i = 0; i < MARKERS_MAX; i++)
      current_props._marker_index[i]  = (current_props._sweep_points / 2) + (i * 10);

   current_props._marker_status.active  = 0;         // active marker = 0
   current_props._marker_status.spare   = 0;         //
   current_props._marker_status.enabled = 0b00001;   // marker-0 enabled, rest disabled

   current_props._velocity_factor     = 0.66;

   current_props._domain_mode         = 0;

   current_props._marker_smith_format = MS_RLC;

   current_props._freq_mode           = FREQ_MODE_START_STOP;

   // Checksum add on caldata_save
   // current_props.checksum          = 0;
}

int load_properties(uint32_t id)
{
   const int r = caldata_recall(id, true);
   update_frequencies(false);
   return r;
}

void
ensure_edit_config(void)
{
  if (active_props == &current_props)
    return;

  //memcpy(&current_props, active_props, sizeof(config_t));
  active_props = &current_props;
  // move to uncal state
  cal_status = 0;
}

#ifdef ENABLED_DUMP
   int16_t dump_buffer[AUDIO_BUFFER_LEN];
   int16_t dump_selection = 0;

   static void duplicate_buffer_to_dump(int16_t *p)
   {
      if (dump_selection == 1)
         p = samp_buf;
      else
      if (dump_selection == 2)
         p = ref_buf;
      memcpy(dump_buffer, p, sizeof dump_buffer);
   }
#endif

//
// DMA i2s callback function, called on get 'half' and 'full' buffer size data
// need for process data, while DMA fill next buffer
void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
   int16_t *p = &rx_buffer[offset];
   (void)i2sp;
   if (wait_count > 0)
   {
      if (wait_count <= config.bandwidth + 1)
      {
         if (wait_count == config.bandwidth + 1)
            reset_dsp_accumerator();
         dsp_process(p, n);
      }
      #ifdef ENABLED_DUMP
         duplicate_buffer_to_dump(p);
      #endif
      --wait_count;
   }
   stat.callback_count++;
}

static const I2SConfig i2sconfig = {
  NULL,                   // TX Buffer
  rx_buffer,              // RX Buffer
  AUDIO_BUFFER_LEN * 2,   // RX Buffer size
  NULL,                   // tx callback
  i2s_end_callback,       // rx callback
  0,                      // i2scfgr
  0                       // i2spr
};

static uint16_t get_sweep_mode(void){
  uint16_t sweep_mode = 0;
  int t;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    if (trace[t].channel == 0) sweep_mode|=SWEEP_CH0_MEASURE;
    if (trace[t].channel == 1) sweep_mode|=SWEEP_CH1_MEASURE;
  }
  return sweep_mode;
}

// main loop for measurement
bool sweep(bool break_on_operation, uint16_t sweep_mode, bool no_cal)
{
   if (p_sweep >= sweep_points || break_on_operation == false)
      RESET_SWEEP;

   if (break_on_operation && sweep_mode == 0)
      return false;

   // blink LED while scanning
   palClearPad(GPIOC, GPIOC_LED);

   // Power stabilization after LED off, before measure
   int st_delay = 3;

	#ifdef USE_INTEGRATOR
   	//const float integrator_coeff = (config.integrator_coeff > 0.0f && config.integrator_coeff < 1.0f) ? config.integrator_coeff : 1.0f;		// 0.0 to 1.0
   	const float integrator_coeff = config.integrator_coeff;
	#endif

   for (; p_sweep < sweep_points; p_sweep++)
   {   // 5300
   	float gamma[2];

      if (frequencies[p_sweep] == 0)
         break;

      int delay = set_frequency(frequencies[p_sweep]);

      if (sweep_mode)
      {
         if (SWEEP_CH0_MEASURE)
         {
         	float *gamma_array = measured[0][p_sweep];

            tlv320aic3204_select(0);                   // CH0:REFLECTION, reset and begin measure

            DSP_START(delay + st_delay);

            delay = DELAY_CHANNEL_CHANGE;

            //================================================
            // Place some code thats need execute while delay
            //================================================

            DSP_WAIT_READY;

            (*sample_func)(gamma);      // calculate reflection coefficient

            if (cal_status & CALSTAT_APPLY && !no_cal)
               apply_CH0_error_term_at(gamma, p_sweep);

				#ifdef USE_INTEGRATOR
            {	// averager .. simple IIR filter
           		gamma_array[0] = ((1.0f - integrator_coeff) * gamma_array[0]) + (integrator_coeff * gamma[0]);
            	gamma_array[1] = ((1.0f - integrator_coeff) * gamma_array[1]) + (integrator_coeff * gamma[1]);
				}
            #else
            {
            	gamma_array[0] = gamma[0];
            	gamma_array[1] = gamma[1];
            }
				#endif
         }

         if (SWEEP_CH1_MEASURE)
         {
         	float *gamma_array = measured[1][p_sweep];

         	tlv320aic3204_select(1);                   // CH1:TRANSMISSION, reset and begin measure

            DSP_START(delay + st_delay);

            delay = DELAY_CHANNEL_CHANGE;

            //================================================
            // Place some code thats need execute while delay
            //================================================

            DSP_WAIT_READY;

            (*sample_func)(gamma);      // calculate transmission coefficient

            if (cal_status & CALSTAT_APPLY && !no_cal)
               apply_CH1_error_term_at(gamma, p_sweep);

				#ifdef USE_INTEGRATOR
            {	// averager .. simple IIR filter
					gamma_array[0] = ((1.0f - integrator_coeff) * gamma_array[0]) + (integrator_coeff * gamma[0]);
					gamma_array[1] = ((1.0f - integrator_coeff) * gamma_array[1]) + (integrator_coeff * gamma[1]);
				}
            #else
            {
            	gamma_array[0] = gamma[0];
					gamma_array[1] = gamma[1];
            }
				#endif
         }
      }

      st_delay = 0;

      // Display SPI made noise on measurement (can see in CW mode)
      // ili9341_fill(OFFSETX + CELLOFFSETX, OFFSETY, (p_sweep * WIDTH)/(sweep_points - 1), 1, RGB565(0, 0, 255));
   }

   // blink LED while scanning
   palSetPad(GPIOC, GPIOC_LED);

   return true;
}

#define MAX_BANDWIDTH      ( AUDIO_ADC_FREQ / AUDIO_SAMPLES_COUNT)
#define MIN_BANDWIDTH      ((AUDIO_ADC_FREQ / AUDIO_SAMPLES_COUNT) / 512 + 1)

uint32_t get_bandwidth_frequency(uint16_t bw_freq)
{
	return (AUDIO_ADC_FREQ / AUDIO_SAMPLES_COUNT) / (bw_freq + 1);
}

void set_bandwidth(uint16_t bw_count)
{
	config.bandwidth = bw_count & 0x1FF;
	redraw_request |= REDRAW_FREQUENCY;
}

VNA_SHELL_FUNCTION(cmd_bandwidth)
{
	int user_bw;
	if (argc == 1)
		user_bw = my_atoui(argv[0]);
	else
	if (argc >= 2)
	{
		int f = my_atoui(argv[0]);
		if (f > MAX_BANDWIDTH) user_bw = 0;
		else
		if (f < MIN_BANDWIDTH) user_bw = 511;
		else
			user_bw = ((AUDIO_ADC_FREQ + AUDIO_SAMPLES_COUNT / 2) / AUDIO_SAMPLES_COUNT) / f - 1;
	}
	else
		goto result;

	set_bandwidth(user_bw);

result:
	shell_printf("bandwidth %d (%uHz)\r\n", config.bandwidth, get_bandwidth_frequency(config.bandwidth));
}

#ifdef POINTS_SET_51
   void set_sweep_points(uint16_t points)
   {
      if (points == sweep_points || points > POINTS_COUNT)
         return;

      sweep_points = points;

      update_frequencies(true);

      if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY))
         cal_interpolate(config.current_id);
   }
#endif

VNA_SHELL_FUNCTION(cmd_scan_bin)
{
   int i;
   uint32_t start;
   uint32_t stop;
   uint16_t points = sweep_points;

   if (argc < 2)
   {
      shell_printf("usage: scan_bin {start(Hz)} {stop(Hz)} [points] [outmask]\r\n");
      return;
   }

   start = my_atoui(argv[0]);
   stop  = my_atoui(argv[1]);
   if (start == 0 || stop == 0 || start > stop)
   {
      shell_printf("frequency range is invalid\r\n");
      return;
   }

   if (argc >= 3)
   {
      points = my_atoui(argv[2]);
      if (points == 0 || points > POINTS_COUNT)
      {
         shell_printf("sweep points exceeds range "define_to_STR(POINTS_COUNT)"\r\n");
         return;
      }
      sweep_points = points;
   }

   uint16_t mask = 7;   // default to sending all 3 fields
   if (argc >= 4)
      mask = my_atoui(argv[3]);

   uint16_t sweep_mode = SWEEP_CH0_MEASURE | SWEEP_CH1_MEASURE;
   sweep_mode = (mask >> 1) & 3;

   // Output data after if set (faster data receive)
   if (mask)
   {
      set_frequencies(start, stop, points);

      if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY))
         cal_interpolate(config.current_id);

      pause_sweep();

      const bool no_sweep_cal = (mask & 8) ? true : false;

      sweep(false, sweep_mode, no_sweep_cal);

      // send the bianry data LS-Byte first (little endian)

      // 2-bytes .. mask
      streamWrite(shell_stream, (void *)&mask, 2);

      // 2 bytes .. number of scatter points to follow
      streamWrite(shell_stream, (void *)&points, 2);

      // frequencies and s-params
      for (i = 0; i < points; i++)
      {
         if (mask & 1)  // 4 bytes .. frequency
            streamWrite(shell_stream, (void *)&frequencies[i], 4);
         if (mask & 2)  // 8 bytes .. S11 real/imag
            streamWrite(shell_stream, (void *)&measured[0][i][0], 4 * 2);
         if (mask & 4)  // 8 bytes .. S21 real/imag
            streamWrite(shell_stream, (void *)&measured[1][i][0], 4 * 2);
      }
   }
}

VNA_SHELL_FUNCTION(cmd_scan)
{
   uint32_t start;
   uint32_t stop;
   uint16_t points = sweep_points;
   int i;

   if (argc < 2)
   {
      shell_printf("usage: scan {start(Hz)} {stop(Hz)} [points] [outmask]\r\n");
      return;
   }

   start = my_atoui(argv[0]);
   stop  = my_atoui(argv[1]);
   if (start == 0 || stop == 0 || start > stop)
   {
      shell_printf("frequency range is invalid\r\n");
      return;
   }

   if (argc >= 3)
   {
      points = my_atoui(argv[2]);
      if (points == 0 || points > POINTS_COUNT)
      {
         shell_printf("sweep points exceeds range "define_to_STR(POINTS_COUNT)"\r\n");
         return;
      }
      sweep_points = points;
   }

   uint16_t mask = 0;
   uint16_t sweep_mode = SWEEP_CH0_MEASURE | SWEEP_CH1_MEASURE;
   bool bin_mode = false;
   if (argc >= 4)
   {
      mask       = my_atoui(argv[3]);
      sweep_mode = (mask >> 1) & 3;
      bin_mode   = ((mask >> 7) & 1) ? true : false;
   }

   set_frequencies(start, stop, points);

   if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY))
      cal_interpolate(config.current_id);

   pause_sweep();

   const bool no_sweep_cal = (mask & 8) ? true : false;

   sweep(false, sweep_mode, no_sweep_cal);

   if (!bin_mode)
   {	// Output data after if set (faster data recive)
   	if (mask)
   	{
   		for (i = 0; i < points; i++)
   		{
   			if (mask & 1) shell_printf("%u ", frequencies[i]);
   			if (mask & 2) shell_printf("%+f %+f ", measured[0][i][0], measured[0][i][1]);
   			if (mask & 4) shell_printf("%+f %+f ", measured[1][i][0], measured[1][i][1]);
   			shell_printf("\r\n");
   		}
   	}
   }
   else
   {	// binary mode

   	// 2-bytes .. mask
      streamWrite(shell_stream, (void *)&mask, 2);

      // 2 bytes .. number of scatter points to follow
      streamWrite(shell_stream, (void *)&points, 2);

      // frequencies and s-params
      for (i = 0; i < points; i++)
      {
         if (mask & 1)  // 4 bytes .. frequency
            streamWrite(shell_stream, (void *)&frequencies[i], 4);
         if (mask & 2)  // 8 bytes .. S11 real/imag
            streamWrite(shell_stream, (void *)&measured[0][i][0], 4 * 2);
         if (mask & 4)  // 8 bytes .. S21 real/imag
            streamWrite(shell_stream, (void *)&measured[1][i][0], 4 * 2);
      }
   }
}

static void set_frequencies(uint32_t start, uint32_t stop, uint16_t points)
{
   uint32_t i;
   uint32_t step  = points - 1;
   uint32_t span  = stop - start;
   uint32_t delta = span / step;
   uint32_t error = span % step;
   uint32_t f     = start;
   uint32_t df    = step >> 1;

   for (i = 0; i <= step; i++, f += delta)
   {
      frequencies[i] = f;
      df += error;
      if (df >=step)
      {
         f++;
         df -= step;
      }
   }

   // disable at out of sweep range
   for (; i < POINTS_COUNT; i++)
      frequencies[i] = 0;
}

uint32_t get_marker_frequency(int marker)
{
   if (marker < 0 || marker >= MARKERS_MAX)
      return 0;

   if (!marker_enabled(marker))
      return 0;

   const int index = marker_index(marker);

   return (index >= 0 && index < sweep_points) ? frequencies[index] : 0;
}

static void update_frequencies(bool update_marker_indexes)
{
   const uint32_t fstart = get_sweep_frequency(ST_START);
   const uint32_t fstop  = get_sweep_frequency(ST_STOP);

   // fetch the current marker frequencies
   uint32_t marker_frequencies[MARKERS_MAX];
   if (update_marker_indexes)
   {
      int m;
      for (m = 0; m < MARKERS_MAX; m++)
         marker_frequencies[m] = marker_enabled(m) ? get_marker_frequency(m) : 0;
   }

   set_frequencies(fstart, fstop, sweep_points);

   //operation_requested |= OP_FREQCHANGE;

   // ***********************
   // update marker indexes to try and keep them on the same frequencies they were on before the frequency changes

   if (update_marker_indexes)
   {
      int m;
      for (m = 0; m < MARKERS_MAX; m++)
      {
         const uint32_t fmark = marker_frequencies[m];
         uint8_t index        = sweep_points / 2;   // default to center index

         if (!marker_enabled(m) || fmark == 0)
            continue;

         if (fmark <= fstart)
         {
            index = 0;
         }
         else
           if (fmark >= fstop)
           {
              index = sweep_points - 1;
           }
           else
           {
              int i;
              for (i = 0; i < sweep_points - 1; i++)
              {
                 const uint32_t f0 = frequencies[i + 0];
                 const uint32_t f1 = frequencies[i + 1];
                 if (fmark >= f0 && fmark < f1)
                 {
                    index = (fmark < ((f0 + f1) / 2)) ? i + 0 : i + 1;
                    break;
                 }
            }
         }

         // save new marker index
         set_marker_index(m, index);
      }
   }

   // ***********************

   // set grid layout
   update_grid();

   RESET_SWEEP;
}

void set_sweep_frequency(int type, uint32_t freq)
{
   const int cal_applied = cal_status & CALSTAT_APPLY;

   // Check frequency for out of bounds (minimum SPAN can be any value)
   if (type != ST_SPAN && freq < START_MIN)
      freq = START_MIN;
   if (freq > STOP_MAX)
      freq = STOP_MAX;

   ensure_edit_config();

   switch (type)
   {
      case ST_START:
         freq_mode &= ~FREQ_MODE_CENTER_SPAN;
         if (frequency0 != freq)
         {
            frequency0 = freq;
            // if start > stop then make start = stop
            if (frequency1 < freq)
                frequency1 = freq;
         }
         break;
      case ST_STOP:
         freq_mode &= ~FREQ_MODE_CENTER_SPAN;
         if (frequency1 != freq)
         {
            frequency1 = freq;
            // if start > stop then make start = stop
            if (frequency0 > freq)
                frequency0 = freq;
         }
         break;
      case ST_CENTER:
         freq_mode |= FREQ_MODE_CENTER_SPAN;
         const uint32_t center = (frequency0 / 2) + (frequency1 / 2);
         if (center != freq)
         {
            uint32_t span = frequency1 - frequency0;
            if (freq < (START_MIN + (span / 2)))
               span = (freq - START_MIN) * 2;
            if (freq > (STOP_MAX - (span / 2)))
               span = (STOP_MAX - freq) * 2;
            frequency0 = freq - (span / 2);
            frequency1 = freq + (span / 2);
         }
         break;
      case ST_SPAN:
         freq_mode |= FREQ_MODE_CENTER_SPAN;
         if (frequency1 - frequency0 != freq)
         {
            uint32_t center = (frequency0 / 2) + (frequency1 / 2);
            if (center < START_MIN + (freq / 2))
                center = START_MIN + (freq / 2);
            if (center > STOP_MAX - (freq / 2))
                center = STOP_MAX - (freq / 2);
            frequency0 = center - (freq / 2);
            frequency1 = center + (freq / 2);
         }
         break;
      case ST_CW:
         freq_mode |= FREQ_MODE_CENTER_SPAN;
         if (frequency0 != freq || frequency1 != freq)
         {
            frequency0 = freq;
            frequency1 = freq;
         }
         break;
   }

   update_frequencies(true);

   if (cal_auto_interpolate && cal_applied)
      cal_interpolate(config.current_id);
}

uint32_t
get_sweep_frequency(int type)
{
  // Obsolete, ensure correct start/stop, start always must be < stop
  if (frequency0 > frequency1) {
    uint32_t t = frequency0;
    frequency0 = frequency1;
    frequency1 = t;
  }
  switch (type) {
    case ST_START:  return frequency0;
    case ST_STOP:   return frequency1;
    case ST_CENTER: return frequency0/2 + frequency1/2;
    case ST_SPAN:   return frequency1 - frequency0;
    case ST_CW:     return frequency0;
  }
  return 0;
}

VNA_SHELL_FUNCTION(cmd_sweep)
{
   if (argc < 1)
   {
      shell_printf("%u %u %d\r\n", get_sweep_frequency(ST_START), get_sweep_frequency(ST_STOP), sweep_points);
      return;
   }

   if (argc > 3)
      goto usage;

   uint32_t value0 = (argc >= 1) ? my_atoui(argv[0]) : 0;
   uint32_t value1 = (argc >= 2) ? my_atoui(argv[1]) : 0;
   #ifdef POINTS_SET_51
      uint32_t value2 = (argc >= 3) ? my_atoui(argv[2]) : 0;
   #endif

   #if MAX_FREQ_TYPE != 5
      #error "Sweep mode possibly changed, check cmd_sweep function"
   #endif

   // Parse sweep {start|stop|center|span|cw} {freq(Hz)}
   // get enum ST_START, ST_STOP, ST_CENTER, ST_SPAN, ST_CW
   static const char sweep_cmd[] = "start|stop|center|span|cw";
   if (argc == 2 && value0 == 0)
   {
      int type = get_str_index(argv[0], sweep_cmd);
      if (type == -1)
         goto usage;
      set_sweep_frequency(type, value1);
      return;
   }

   //  Parse sweep {start(Hz)} [stop(Hz)]
   if (value0)
      set_sweep_frequency(ST_START, value0);

   if (value1)
      set_sweep_frequency(ST_STOP, value1);

   #ifdef POINTS_SET_51
       if (value2)
          set_sweep_points(value2);
   #endif

     return;

usage:
   #ifdef POINTS_SET_51
      shell_printf("usage: sweep {start(Hz)} [stop(Hz)] [points]\r\n"\
                   "\tsweep {%s} {freq(Hz)}\r\n", sweep_cmd);
   #else
      shell_printf("usage: sweep {start(Hz)} [stop(Hz)]\r\n"\
                   "\tsweep {%s} {freq(Hz)}\r\n", sweep_cmd);
   #endif
}


static void eterm_set(int term, float re, float im)
{
   unsigned int i;
   for (i = 0; i < sweep_points; i++)
   {
      cal_data[term][i][0] = re;
      cal_data[term][i][1] = im;
   }
}

static void eterm_copy(int dst, int src)
{
   memcpy(cal_data[dst], cal_data[src], sizeof cal_data[dst]);
}

#if 0
const struct open_model {
  float c0;
  float c1;
  float c2;
  float c3;
} open_model = { 50, 0, -300, 27 };
#endif

#if 0
static void adjust_ed(void)
{
  unsigned int i;
  for (i = 0; i < sweep_points; i++)
  {
    // z=1/(jwc*z0) = 1/(2*pi*f*c*z0)  Note: normalized with Z0
    // s11ao = (z-1)/(z+1) = (1-1/z)/(1+1/z) = (1-jwcz0)/(1+jwcz0)
    // prepare 1/s11ao to avoid dividing complex
    float c = 1000e-15f;
    float z0 = 50.0f;
    //float z = 2 * VNA_PI * frequencies[i] * c * z0;
    float z = 0.02f;
    cal_data[ETERM_ED][i][0] += z;
  }
}
#endif

static void eterm_calc_es(void)
{
  int i;
  for (i = 0; i < sweep_points; i++)
  {
    // z=1/(jwc*z0) = 1/(2*pi*f*c*z0)  Note: normalized with Z0
    // s11ao = (z-1)/(z+1) = (1-1/z)/(1+1/z) = (1-jwcz0)/(1+jwcz0)
    // prepare 1/s11ao for effeiciency

    const float c      = 50e-15f;
    //const float c    = 1.707e-12;
    const float z0     = 50.0f;
    const float z      = (2 * VNA_PI) * frequencies[i] * c * z0;
    float sq           =  1 + (z * z);
    const float s11aor = (1 - (z * z)) / sq;
    const float s11aoi = (2 * z) / sq;

    // S11mo’= S11mo - Ed
    // S11ms’= S11ms - Ed

    const float s11or = cal_data[CAL_OPEN ][i][0] - cal_data[ETERM_ED][i][0];
    const float s11oi = cal_data[CAL_OPEN ][i][1] - cal_data[ETERM_ED][i][1];
    const float s11sr = cal_data[CAL_SHORT][i][0] - cal_data[ETERM_ED][i][0];
    const float s11si = cal_data[CAL_SHORT][i][1] - cal_data[ETERM_ED][i][1];

    // Es = (S11mo'/s11ao + S11ms’)/(S11mo' - S11ms’)

    const float numr = s11sr + (s11or * s11aor) - (s11oi * s11aoi);
    const float numi = s11si + (s11oi * s11aor) + (s11or * s11aoi);

    const float denomr = s11or - s11sr;
    const float denomi = s11oi - s11si;

    sq = (denomr * denomr) + (denomi * denomi);

    cal_data[ETERM_ES][i][0] = ((numr * denomr) + (numi * denomi)) / sq;
    cal_data[ETERM_ES][i][1] = ((numi * denomr) - (numr * denomi)) / sq;
  }

  cal_status &= ~CALSTAT_OPEN;
  cal_status |=  CALSTAT_ES;
}

static void eterm_calc_er(int sign)
{
  int i;
  for (i = 0; i < sweep_points; i++)
  {
    // Er = sign*(1-sign*Es)S11ms'
    const float s11sr = cal_data[CAL_SHORT][i][0] - cal_data[ETERM_ED][i][0];
    const float s11si = cal_data[CAL_SHORT][i][1] - cal_data[ETERM_ED][i][1];
          float esr   = cal_data[ETERM_ES][i][0];
          float esi   = cal_data[ETERM_ES][i][1];
    if (sign > 0)
    {
      esr = -esr;
      esi = -esi;
    }
    esr = 1.0f + esr;
    float err = (esr * s11sr) - (esi * s11si);
    float eri = (esr * s11si) + (esi * s11sr);
    if (sign < 0)
    {
      err = -err;
      eri = -eri;
    }
    cal_data[ETERM_ER][i][0] = err;
    cal_data[ETERM_ER][i][1] = eri;
  }

  cal_status &= ~CALSTAT_SHORT;
  cal_status |=  CALSTAT_ER;
}

// CAUTION: Et is inversed for efficiency
static void eterm_calc_et(void)
{
  int i;
  for (i = 0; i < sweep_points; i++)
  {
    // Et = 1/(S21mt - Ex)
    const float etr = cal_data[CAL_THRU][i][0] - cal_data[CAL_ISOLN][i][0];
    const float eti = cal_data[CAL_THRU][i][1] - cal_data[CAL_ISOLN][i][1];
    const float sq = (etr * etr) + (eti * eti);
    const float invr =  etr / sq;
    const float invi = -eti / sq;
    cal_data[ETERM_ET][i][0] = invr;
    cal_data[ETERM_ET][i][1] = invi;
  }

  cal_status &= ~CALSTAT_THRU;
  cal_status |=  CALSTAT_ET;
}

#if 0
void apply_error_term(void)
{
  int i;
  for (i = 0; i < sweep_points; i++)
  {
    // S11m' = S11m - Ed
    // S11a = S11m' / (Er + Es S11m')
    float s11mr = measured[0][i][0] - cal_data[ETERM_ED][i][0];
    float s11mi = measured[0][i][1] - cal_data[ETERM_ED][i][1];
    float err = cal_data[ETERM_ER][i][0] + s11mr * cal_data[ETERM_ES][i][0] - s11mi * cal_data[ETERM_ES][i][1];
    float eri = cal_data[ETERM_ER][i][1] + s11mr * cal_data[ETERM_ES][i][1] + s11mi * cal_data[ETERM_ES][i][0];
    float sq = err*err + eri*eri;
    float s11ar = (s11mr * err + s11mi * eri) / sq;
    float s11ai = (s11mi * err - s11mr * eri) / sq;
    measured[0][i][0] = s11ar;
    measured[0][i][1] = s11ai;

    // CAUTION: Et is inversed for efficiency
    // S21m' = S21m - Ex
    // S21a = S21m' (1-EsS11a)Et
    float s21mr = measured[1][i][0] - cal_data[ETERM_EX][i][0];
    float s21mi = measured[1][i][1] - cal_data[ETERM_EX][i][1];
    float esr = 1 - (cal_data[ETERM_ES][i][0] * s11ar - cal_data[ETERM_ES][i][1] * s11ai);
    float esi = - (cal_data[ETERM_ES][i][1] * s11ar + cal_data[ETERM_ES][i][0] * s11ai);
    float etr = esr * cal_data[ETERM_ET][i][0] - esi * cal_data[ETERM_ET][i][1];
    float eti = esr * cal_data[ETERM_ET][i][1] + esi * cal_data[ETERM_ET][i][0];
    float s21ar = s21mr * etr - s21mi * eti;
    float s21ai = s21mi * etr + s21mr * eti;
    measured[1][i][0] = s21ar;
    measured[1][i][1] = s21ai;
  }
}

static void apply_error_term_at(int i)
{
    // S11m' = S11m - Ed
    // S11a = S11m' / (Er + Es S11m')
    float s11mr = measured[0][i][0] - cal_data[ETERM_ED][i][0];
    float s11mi = measured[0][i][1] - cal_data[ETERM_ED][i][1];
    float err = cal_data[ETERM_ER][i][0] + s11mr * cal_data[ETERM_ES][i][0] - s11mi * cal_data[ETERM_ES][i][1];
    float eri = cal_data[ETERM_ER][i][1] + s11mr * cal_data[ETERM_ES][i][1] + s11mi * cal_data[ETERM_ES][i][0];
    float sq = err*err + eri*eri;
    float s11ar = (s11mr * err + s11mi * eri) / sq;
    float s11ai = (s11mi * err - s11mr * eri) / sq;
    measured[0][i][0] = s11ar;
    measured[0][i][1] = s11ai;

    // CAUTION: Et is inversed for efficiency
    // S21m' = S21m - Ex
    // S21a = S21m' (1-EsS11a)Et
    float s21mr = measured[1][i][0] - cal_data[ETERM_EX][i][0];
    float s21mi = measured[1][i][1] - cal_data[ETERM_EX][i][1];
#if 0
    float esr = 1 - (cal_data[ETERM_ES][i][0] * s11ar - cal_data[ETERM_ES][i][1] * s11ai);
    float esi = 0 - (cal_data[ETERM_ES][i][1] * s11ar + cal_data[ETERM_ES][i][0] * s11ai);
    float etr = esr * cal_data[ETERM_ET][i][0] - esi * cal_data[ETERM_ET][i][1];
    float eti = esr * cal_data[ETERM_ET][i][1] + esi * cal_data[ETERM_ET][i][0];
    float s21ar = s21mr * etr - s21mi * eti;
    float s21ai = s21mi * etr + s21mr * eti;
#else
    // Not made CH1 correction by CH0 data
    float s21ar = s21mr * cal_data[ETERM_ET][i][0] - s21mi * cal_data[ETERM_ET][i][1];
    float s21ai = s21mi * cal_data[ETERM_ET][i][0] + s21mr * cal_data[ETERM_ET][i][1];
#endif
    measured[1][i][0] = s21ar;
    measured[1][i][1] = s21ai;
}
#endif

//static void apply_CH0_error_term_at(int i)
static void apply_CH0_error_term_at(float gamma[2], int i)
{
//	float *gamma   = measured[0][i];
   // S11m' = S11m - Ed
   // S11a = S11m' / (Er + Es S11m')
   const float mr = gamma[0] - cal_data[ETERM_ED][i][0];
   const float mi = gamma[1] - cal_data[ETERM_ED][i][1];
   const float cr = cal_data[ETERM_ES][i][0];
   const float ci = cal_data[ETERM_ES][i][1];
   const float er = cal_data[ETERM_ER][i][0] + (mr * cr) - (mi * ci);
   const float ei = cal_data[ETERM_ER][i][1] + (mr * ci) + (mi * cr);
   const float sq =  (er * er) + (ei * ei);
   gamma[0]       = ((mr * er) + (mi * ei)) / sq;
   gamma[1]       = ((mi * er) - (mr * ei)) / sq;
}

//static void apply_CH1_error_term_at(int i)
static void apply_CH1_error_term_at(float gamma[2], int i)
{
//	float *gamma   = measured[1][i];
   // CAUTION: Et is inversed for efficiency
   // S21a = (S21m - Ex) * Et
   const float mr = gamma[0] - cal_data[ETERM_EX][i][0];
   const float mi = gamma[1] - cal_data[ETERM_EX][i][1];
   // Not made CH1 correction by CH0 data
	const float cr = cal_data[ETERM_ET][i][0];
	const float ci = cal_data[ETERM_ET][i][1];
   gamma[0]       = (mr * cr) - (mi * ci);
   gamma[1]       = (mi * cr) + (mr * ci);
}

static void apply_edelay(void)
{
  const uint16_t sweep_mode = get_sweep_mode();
  int i;
  for (i = 0; i < sweep_points; i++)
  {
    const float w = (float)(2 * VNA_PI * 1e-12) * electrical_delay * frequencies[i];

    //const float s = sinf(w);
    //const float c = cosf(w);
    float s, c;
    arm_sin_cos_f32(w, &s, &c);

    if (sweep_mode & SWEEP_CH0_MEASURE)
    {
      const float real = measured[0][i][0];
      const float imag = measured[0][i][1];
      measured[0][i][0] = (real * c) - (imag * s);
      measured[0][i][1] = (imag * c) + (real * s);
    }
    if (sweep_mode & SWEEP_CH1_MEASURE)
    {
      const float real = measured[1][i][0];
      const float imag = measured[1][i][1];
      measured[1][i][0] = (real * c) - (imag * s);
      measured[1][i][1] = (imag * c) + (real * s);
    }
  }
}

void cal_collect(int type)
{
  int dst;
  int src;

  // ensure_edit_config();
  active_props = &current_props;

  switch (type)
  {
    case CAL_LOAD:  cal_status|= CALSTAT_LOAD;  dst = CAL_LOAD;  src = 0; break;
    case CAL_OPEN:  cal_status|= CALSTAT_OPEN;  dst = CAL_OPEN;  src = 0; cal_status &= ~(CALSTAT_ES); break;
    case CAL_SHORT: cal_status|= CALSTAT_SHORT; dst = CAL_SHORT; src = 0; cal_status &= ~(CALSTAT_ER); break;
    case CAL_THRU:  cal_status|= CALSTAT_THRU;  dst = CAL_THRU;  src = 1; break;
    case CAL_ISOLN: cal_status|= CALSTAT_ISOLN; dst = CAL_ISOLN; src = 1; break;
    default: return;
  }

  // Run sweep for collect data (use minimum BANDWIDTH_30, or bigger if set)
  const uint8_t bw = config.bandwidth;  // store current setting
  const uint16_t status = cal_status;
  if (bw < BANDWIDTH_30)
    config.bandwidth = BANDWIDTH_30;

  cal_status &= ~(CALSTAT_APPLY);

  // Set MAX settings for sweep_points on calibrate
//  if (sweep_points != POINTS_COUNT)
//    set_sweep_points(POINTS_COUNT);

  sweep(false, src == 0 ? SWEEP_CH0_MEASURE : SWEEP_CH1_MEASURE, false);
  config.bandwidth = bw;          // restore
  cal_status = status;

  // Copy calibration data
  memcpy(cal_data[dst], measured[src], sizeof measured[0]);

  redraw_request |= REDRAW_CAL_STATUS;
}

void cal_done(void)
{
  ensure_edit_config();

  if (!(cal_status & CALSTAT_LOAD))
    eterm_set(ETERM_ED, 0.0, 0.0);

  //adjust_ed();

  if ((cal_status & CALSTAT_SHORT) && (cal_status & CALSTAT_OPEN))
  {
    eterm_calc_es();
    eterm_calc_er(-1);
  }
  else
  if (cal_status & CALSTAT_OPEN)
  {
    eterm_copy(CAL_SHORT, CAL_OPEN);
    eterm_set(ETERM_ES, 0.0, 0.0);
    eterm_calc_er(1);
  }
  else
  if (cal_status & CALSTAT_SHORT)
  {
    eterm_set(ETERM_ES, 0.0, 0.0);
    cal_status &= ~CALSTAT_SHORT;
    eterm_calc_er(-1);
  }
  else
  if (!(cal_status & CALSTAT_ER))
    eterm_set(ETERM_ER, 1.0, 0.0);
  else
  if (!(cal_status & CALSTAT_ES))
    eterm_set(ETERM_ES, 0.0, 0.0);

  if (!(cal_status & CALSTAT_ISOLN))
    eterm_set(ETERM_EX, 0.0, 0.0);

  if (cal_status & CALSTAT_THRU)
    eterm_calc_et();
  else
  if (!(cal_status & CALSTAT_ET))
    eterm_set(ETERM_ET, 1.0, 0.0);

  cal_status |= CALSTAT_APPLY;
  redraw_request |= REDRAW_CAL_STATUS;
}

static void cal_interpolate(int s)
{
  const properties_t *src = caldata_ref(s);
  uint32_t i;
  uint32_t j;
  int eterm;

  if (src == NULL)
    return;

  ensure_edit_config();

  uint32_t src_f = src->_frequency0;

  // lower than start freq of src range
  for (i = 0; i < sweep_points; i++)
  {
    if (frequencies[i] >= src_f)
      break;

    // fill cal_data at head of src range
    for (eterm = 0; eterm < 5; eterm++)
    {
      cal_data[eterm][i][0] = src->_cal_data[eterm][0][0];
      cal_data[eterm][i][1] = src->_cal_data[eterm][0][1];
    }
  }

  // ReBuild src freq list
  const uint32_t src_points = (src->_sweep_points - 1);
  const uint32_t span       = src->_frequency1 - src->_frequency0;
  const uint32_t delta      = span / src_points;
  const uint32_t error      = span % src_points;
        uint32_t df         = src_points >> 1;

  j = 0;
  for (; i < sweep_points; i++)
  {
    const uint32_t f = frequencies[i];
    if (f == 0)
       goto interpolate_finish;

    for (; j < src_points; j++)
    {
      if (src_f <= f && f < src_f + delta)
      {
        // found f between freqs at j and j+1
        float k1 = (delta == 0) ? 0.0f : (float)(f - src_f) / delta;

        // avoid glitch between freqs in different harmonics mode
        uint16_t idx = j;
        if (si5351_get_harmonic_lvl(src_f) != si5351_get_harmonic_lvl(src_f+delta))
        {
          // f in prev harmonic, need extrapolate from prev 2 points
          if (si5351_get_harmonic_lvl(f) == si5351_get_harmonic_lvl(src_f))
          {
            if (idx >= 1)
            {
              idx--;
              k1+= 1.0f;
            }
            else // point limit
              k1 = 0.0f;
          }
          else  // f in next harmonic, need extrapolate from next 2 points
          {
            if (idx < src_points)
            {
              idx++;
              k1 -= 1.0f;
            }
            else // point limit
              k1 = 1.0f;
          }
        }

        const float k0 = 1.0f - k1;
        for (eterm = 0; eterm < 5; eterm++)
        {
          cal_data[eterm][i][0] = (src->_cal_data[eterm][idx][0] * k0) + (src->_cal_data[eterm][idx+1][0] * k1);
          cal_data[eterm][i][1] = (src->_cal_data[eterm][idx][1] * k0) + (src->_cal_data[eterm][idx+1][1] * k1);
        }

        break;
      }

      df += error;
      if (df >= src_points)
      {
         src_f++;
         df -= src_points;
      }
      src_f += delta;
    }

    if (j == src_points)
      break;
  }

  // upper than end freq of src range
  for (; i < sweep_points; i++)
  {
    // fill cal_data at tail of src
    for (eterm = 0; eterm < 5; eterm++)
    {
      cal_data[eterm][i][0] = src->_cal_data[eterm][src_points][0];
      cal_data[eterm][i][1] = src->_cal_data[eterm][src_points][1];
    }
  }

interpolate_finish:
  cal_status     |= src->_cal_status | CALSTAT_APPLY | CALSTAT_INTERPOLATED;
  redraw_request |= REDRAW_CAL_STATUS;
}

VNA_SHELL_FUNCTION(cmd_cal)
{
  static const char *items[] = { "load", "open", "short", "thru", "isoln", "Es", "Er", "Et", "cal'ed" };

  if (argc < 1)
  {
    int i;
    for (i = 0; i < 9; i++)
    {
      if (cal_status & (1 << i))
        shell_printf("%s ", items[i]);
    }
    shell_printf("\r\n");
    shell_printf((cal_status & CALSTAT_APPLY) ? "on\r\n" : "off\r\n");
    return;
  }

  redraw_request |= REDRAW_CAL_STATUS;

  //                                     0    1     2    3     4    5  6   7     8    9 10
  static const char cmd_cal_list[] = "load|open|short|thru|isoln|done|on|off|reset|data|in";
  switch (get_str_index(argv[0], cmd_cal_list))
  {
    case 0:
      cal_collect(CAL_LOAD);
      return;
    case 1:
      cal_collect(CAL_OPEN);
      return;
    case 2:
      cal_collect(CAL_SHORT);
      return;
    case 3:
      cal_collect(CAL_THRU);
      return;
    case 4:
      cal_collect(CAL_ISOLN);
      return;
    case 5:
      cal_done();
      return;
    case 6:
      cal_status |= CALSTAT_APPLY;
      return;
    case 7:
      cal_status &= ~CALSTAT_APPLY;
      return;
    case 8:
      cal_status = 0;
      return;
    case 9:
      shell_printf("%+f %+f\r\n", cal_data[CAL_LOAD][0][0], cal_data[CAL_LOAD][0][1]);
      shell_printf("%+f %+f\r\n", cal_data[CAL_OPEN][0][0], cal_data[CAL_OPEN][0][1]);
      shell_printf("%+f %+f\r\n", cal_data[CAL_SHORT][0][0], cal_data[CAL_SHORT][0][1]);
      shell_printf("%+f %+f\r\n", cal_data[CAL_THRU][0][0], cal_data[CAL_THRU][0][1]);
      shell_printf("%+f %+f\r\n", cal_data[CAL_ISOLN][0][0], cal_data[CAL_ISOLN][0][1]);
      return;
    case 10:
      cal_interpolate((argc > 1) ? my_atoi(argv[1]) : 0);
      return;
    default:
      break;
  }

  shell_printf("usage: cal [%s]\r\n", cmd_cal_list);
}

VNA_SHELL_FUNCTION(cmd_save)
{
  if (argc < 1)
    goto usage;

  int id = my_atoi(argv[0]);
  if (id < 0 || id >= SAVEAREA_MAX)
    goto usage;
  caldata_save(id, true);
  redraw_request |= REDRAW_CAL_STATUS;
  return;

 usage:
  shell_printf("save {id}\r\n");
}

VNA_SHELL_FUNCTION(cmd_recall)
{
   if (argc < 1)
      goto usage;

   const int id = my_atoi(argv[0]);
   if (id < 0 || id >= SAVEAREA_MAX)
      goto usage;

   // Check for success
   if (load_properties(id) == -1)
      shell_printf("Err, default load\r\n");

   redraw_request |= REDRAW_CAL_STATUS;
   return;

usage:
    shell_printf("recall {id}\r\n");
}

static const struct {
  const char *name;
  uint16_t refpos;
  float scale_unit;
} trace_info[MAX_TRACE_TYPE] = {
  { "LOGMAG", NGRIDY-1,  10.0 },
  { "PHASE",  NGRIDY/2,  90.0 },
  { "DELAY",  NGRIDY/2,  1e-9 },
  { "SMITH",         0,  1.00 },
  { "POLAR",         0,  1.00 },
  { "LINEAR",        0,  0.125},
  { "SWR",           0,  0.25 },
  { "REAL",   NGRIDY/2,  0.25 },
  { "IMAG",   NGRIDY/2,  0.25 },
  { "R",      NGRIDY/2, 100.0 },
  { "X",      NGRIDY/2, 100.0 },
  { "Q",             0,  10.0 }
};

static const char * const trc_channel_name[] = {
  "CH0", "CH1"
};

const char *get_trace_typename(int t)
{
  return trace_info[trace[t].type].name;
}

void set_trace_type(int t, int type)
{
  int enabled = type != TRC_OFF;
  int force = FALSE;

  if (trace[t].enabled != enabled) {
    trace[t].enabled = enabled;
    force = TRUE;
  }
  if (trace[t].type != type) {
    trace[t].type = type;
    // Set default trace refpos
    trace[t].refpos = trace_info[type].refpos;
    // Set default trace scale
    trace[t].scale  = trace_info[type].scale_unit;
    force = TRUE;
  }
  if (force) {
    plot_into_index(measured);
//    force_set_markmap();
  }
}

void set_trace_channel(int t, int channel)
{
  if (trace[t].channel != channel) {
    trace[t].channel = channel;
//    force_set_markmap();
  }
}

void set_trace_scale(int t, float scale)
{
  if (trace[t].scale != scale) {
    trace[t].scale = scale;
//    force_set_markmap();
  }
}

float get_trace_scale(int t)
{
  return trace[t].scale;
}

void set_trace_refpos(int t, float refpos)
{
  if (trace[t].refpos != refpos) {
    trace[t].refpos = refpos;
 //   force_set_markmap();
  }
}

float get_trace_refpos(int t)
{
  return trace[t].refpos;
}

VNA_SHELL_FUNCTION(cmd_trace)
{
  int t;
  if (argc <= 0)
  {
    for (t = 0; t < TRACES_MAX; t++) {
      if (trace[t].enabled) {
        const char *type = get_trace_typename(t);
        const char *channel = trc_channel_name[trace[t].channel];
        float scale = get_trace_scale(t);
        float refpos = get_trace_refpos(t);
        shell_printf("%d %s %s %+f %+f\r\n", t, type, channel, scale, refpos);
      }
    }
    return;
  }

  if (strcmp(argv[0], "all") == 0 &&
      argc > 1 && strcmp(argv[1], "off") == 0) {
  for (t = 0; t < TRACES_MAX; t++)
      set_trace_type(t, TRC_OFF);
    goto exit;
  }

  t = my_atoi(argv[0]);
  if (t < 0 || t >= TRACES_MAX)
    goto usage;

  if (argc == 1) {
    const char *type = get_trace_typename(t);
    const char *channel = trc_channel_name[trace[t].channel];
    shell_printf("%d %s %s\r\n", t, type, channel);
    return;
  }
#if MAX_TRACE_TYPE != 12
#error "Trace type enum possibly changed, check cmd_trace function"
#endif
  // enum TRC_LOGMAG, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_Q, TRC_OFF
  static const char cmd_type_list[] = "logmag|phase|delay|smith|polar|linear|swr|real|imag|r|x|q|off";
  int type = get_str_index(argv[1], cmd_type_list);
  if (type >= 0) {
    set_trace_type(t, type);
    goto check_ch_num;
  }
  //                                            0      1
  static const char cmd_scale_ref_list[] = "scale|refpos";
  if (argc >= 3) {
    switch (get_str_index(argv[1], cmd_scale_ref_list)) {
      case 0:
        //trace[t].scale = my_atof(argv[2]);
        set_trace_scale(t, my_atof(argv[2]));
        goto exit;
      case 1:
        //trace[t].refpos = my_atof(argv[2]);
        set_trace_refpos(t, my_atof(argv[2]));
        goto exit;
      default:
        goto usage;
    }
  }
check_ch_num:
  if (argc > 2) {
    int src = my_atoi(argv[2]);
    if (src != 0 && src != 1)
      goto usage;
    trace[t].channel = src;
  }
exit:
  return;
usage:
  shell_printf("trace {0|1|2|3|all} [%s] [src]\r\n"\
               "trace {0|1|2|3} {%s} {value}\r\n", cmd_type_list, cmd_scale_ref_list);
}


void set_electrical_delay(float picoseconds)
{
  if (electrical_delay != picoseconds)
  {
    electrical_delay = picoseconds;
    force_set_markmap();
  }
  redraw_request |= REDRAW_MARKER;
}

float get_electrical_delay(void)
{
  return electrical_delay;
}

VNA_SHELL_FUNCTION(cmd_edelay)
{
  if (argc < 1) {
    shell_printf("%+f\r\n", electrical_delay);
    return;
  }
  set_electrical_delay(my_atof(argv[0]));
}

VNA_SHELL_FUNCTION(cmd_marker)
{
   static const char cmd_marker_list[] = "on|off";

   int marker;

   if (argc < 1)
   {
      for (marker = 0; marker < MARKERS_MAX; marker++)
      {
         if (marker_enabled(marker))
         {
            const int index = marker_index(marker);
            shell_printf("%d %d %d\r\n", marker + 1, index, frequencies[index]);
         }
      }
      return;
   }

   redraw_request |= REDRAW_MARKER;

   if (strcmp(argv[0], "off") == 0)
   {
      set_no_active_marker;
      for (marker = 0; marker < MARKERS_MAX; marker++)
         disable_marker(marker);
      return;
   }

   marker = my_atoi(argv[0]) - 1;
   if (marker < 0 || marker >= MARKERS_MAX)
   {
      shell_printf("marker [n] [%s|{index}]\r\n", cmd_marker_list);
      return;
   }

   if (argc >= 1)
   {
      const int index = marker_index(marker);
      shell_printf("%d %d %d\r\n", marker + 1, index, frequencies[index]);
      enable_marker(marker);
      set_active_marker(marker);
      return;
   }

   switch (get_str_index(argv[1], cmd_marker_list))
   {
      case 0:   // enable marker
         enable_marker(marker);
         set_active_marker(marker);
         break;

      case 1:   // disable marker
         disable_marker(marker);
         if (active_marker == marker)
            set_no_active_marker;
         break;

      default:
         {   // select active marker and move to index
            enable_marker(marker);
            const int index = my_atoi(argv[1]);
            if (index >= 0 && index < POINTS_COUNT)
            {
               set_marker_index(marker, index);
               set_active_marker(marker);
            }
         }
         break;
   }
}

#ifdef ENABLE_TOUCH_COMMAND
	VNA_SHELL_FUNCTION(cmd_touchcal)
	{
		(void)argc;
		(void)argv;
		//extern int16_t touch_cal[4];
		int i;

		shell_printf("first touch upper left, then lower right...");
		touch_cal_exec();
		shell_printf("done\r\n");

		shell_printf("touch cal params: ");
		for (i = 0; i < 4; i++)
			shell_printf("%d ", config.touch_cal[i]);
		shell_printf("\r\n");
	}

	VNA_SHELL_FUNCTION(cmd_touchtest)
	{
		(void)argc;
		(void)argv;
		do touch_draw_test();
		while (argc);
	}
#endif

VNA_SHELL_FUNCTION(cmd_frequencies)
{
  int i;
  (void)argc;
  (void)argv;
  for (i = 0; i < sweep_points; i++) {
    if (frequencies[i] != 0)
      shell_printf("%u\r\n", frequencies[i]);
  }
}

static void
set_domain_mode(uint8_t mode) // accept DOMAIN_FREQ or DOMAIN_TIME
{
  if (mode != (domain_mode & DOMAIN_MODE)) {
    domain_mode = (domain_mode & ~DOMAIN_MODE) | (mode & DOMAIN_MODE);
    redraw_request |= REDRAW_FREQUENCY;
    uistat.lever_mode = LM_MARKER;
  }
}

static void
set_timedomain_func(int func) // accept TD_FUNC_LOWPASS_IMPULSE, TD_FUNC_LOWPASS_STEP or TD_FUNC_BANDPASS
{
  domain_mode = (domain_mode & ~TD_FUNC) | (func & TD_FUNC);
}

static void
set_timedomain_window(int func) // accept TD_WINDOW_MINIMUM/TD_WINDOW_NORMAL/TD_WINDOW_MAXIMUM
{
  domain_mode = (domain_mode & ~TD_WINDOW) | (func & TD_WINDOW);
}

VNA_SHELL_FUNCTION(cmd_transform)
{
  int i;
  if (argc <= 0) {
    goto usage;
  }
  //                                         0   1       2    3        4       5      6       7
  static const char cmd_transform_list[] = "on|off|impulse|step|bandpass|minimum|normal|maximum";
  for (i = 0; i < argc; i++) {
    switch (get_str_index(argv[i], cmd_transform_list)) {
      case 0:
        set_domain_mode(DOMAIN_TIME);
        return;
      case 1:
        set_domain_mode(DOMAIN_FREQ);
        return;
      case 2:
        set_timedomain_func(TD_FUNC_LOWPASS_IMPULSE);
        return;
      case 3:
        set_timedomain_func(TD_FUNC_LOWPASS_STEP);
        return;
      case 4:
        set_timedomain_func(TD_FUNC_BANDPASS);
        return;
      case 5:
        set_timedomain_window(TD_WINDOW_MINIMUM);
        return;
      case 6:
        set_timedomain_window(TD_WINDOW_NORMAL);
        return;
      case 7:
        set_timedomain_window(TD_WINDOW_MAXIMUM);
        return;
      default:
        goto usage;
    }
  }
  return;
usage:
  shell_printf("usage: transform {%s} [...]\r\n", cmd_transform_list);
}

#ifdef ENABLE_TEST_COMMAND
   VNA_SHELL_FUNCTION(cmd_test)
   {
      (void)argc;
      (void)argv;

      #if 0
         int i;
         for (i = 0; i < 100; i++)
         {
            palClearPad(GPIOC, GPIOC_LED);
            set_frequency(10000000);
            palSetPad(GPIOC, GPIOC_LED);
            chThdSleepMilliseconds(50);

            palClearPad(GPIOC, GPIOC_LED);
            set_frequency(90000000);
            palSetPad(GPIOC, GPIOC_LED);
            chThdSleepMilliseconds(50);
         }
      #endif

      #if 0
         int i;
         int mode = 0;

         if (argc >= 1)
            mode = my_atoi(argv[0]);

         for (i = 0; i < 20; i++)
         {
            palClearPad(GPIOC, GPIOC_LED);
            ili9341_test(mode);
            palSetPad(GPIOC, GPIOC_LED);
            chThdSleepMilliseconds(50);
         }
      #endif

      #if 0
         //extern adcsample_t adc_samples[2];
         //shell_printf("adc: %d %d\r\n", adc_samples[0], adc_samples[1]);
         int i;
         int x, y;
         for (i = 0; i < 50; i++)
         {
            test_touch(&x, &y);
            shell_printf("adc: %d %d\r\n", x, y);
            chThdSleepMilliseconds(200);
         }
         //extern int touch_x, touch_y;
         //shell_printf("adc: %d %d\r\n", touch_x, touch_y);
      #endif

      #if 0
         while (argc > 1)
         {
            int16_t x, y;
            touch_position(&x, &y);
            shell_printf("touch: %d %d\r\n", x, y);
            chThdSleepMilliseconds(200);
         }
      #endif
   }
#endif

VNA_SHELL_FUNCTION(cmd_gain)
{
   int rvalue = 0;
   int lvalue;
   //int idx;

//   if (argc < 1 && argc > 3)
   if (argc < 1)
   {
      shell_printf("usage: gain idx {lgain(0-95)} [rgain(0-95)]\r\n");
      return;
   }

   //idx = my_atoui(argv[0]);
   lvalue = rvalue = my_atoui(argv[1]);
   if (argc >= 3)
      rvalue = my_atoui(argv[2]);

   tlv320aic3204_set_gain(lvalue, rvalue);

   //gain_table[idx][0] = lvalue;
   //gain_table[idx][1] = rvalue;
}

#ifdef ENABLE_PORT_COMMAND
   VNA_SHELL_FUNCTION(cmd_port)
   {
      int port;
      if (argc < 1)
      {
         shell_printf("usage: port {0:TX 1:RX}\r\n");
         return;
      }
      port = my_atoi(argv[0]);
      tlv320aic3204_select(port);
   }
#endif

#ifdef ENABLE_STAT_COMMAND
   VNA_SHELL_FUNCTION(cmd_stat)
   {
      int ch;
      int16_t *p = &rx_buffer[0];
      int32_t acc0, acc1;
      int32_t ave0, ave1;
      //float sample[2], ref[2];
      //minr, maxr,  mins, maxs;
      int32_t count = AUDIO_BUFFER_LEN;
      int i;
      (void)argc;
      (void)argv;
      for (ch=0;ch<2;ch++)
      {
         tlv320aic3204_select(ch);
         DSP_START(4);
         DSP_WAIT;
         //reset_dsp_accumerator();
         //dsp_process(&p[               0], AUDIO_BUFFER_LEN);
         //dsp_process(&p[AUDIO_BUFFER_LEN], AUDIO_BUFFER_LEN);

         acc0 = acc1 = 0;
         for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2)
         {
            acc0 += p[i  ];
            acc1 += p[i+1];
         }
         ave0 = acc0 / count;
         ave1 = acc1 / count;
         acc0 = acc1 = 0;
         //minr  = maxr = 0;
         //mins  = maxs = 0;
         for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2)
         {
            acc0 += (p[i  ] - ave0)*(p[i  ] - ave0);
            acc1 += (p[i+1] - ave1)*(p[i+1] - ave1);
            //if (minr < p[i  ]) minr = p[i  ];
            //if (maxr > p[i  ]) maxr = p[i  ];
            //if (mins < p[i+1]) mins = p[i+1];
            //if (maxs > p[i+1]) maxs = p[i+1];
         }

         stat.rms[0] = (int)sqrtf(acc0 / count);
         stat.rms[1] = (int)sqrtf(acc1 / count);

         stat.ave[0] = ave0;
         stat.ave[1] = ave1;
         shell_printf("Ch: %d\r\n", ch);
         shell_printf("average:   r: %6d s: %6d\r\n", stat.ave[0], stat.ave[1]);
         shell_printf("rms:       r: %6d s: %6d\r\n", stat.rms[0], stat.rms[1]);
         //shell_printf("min:     ref %6d ch %6d\r\n", minr, mins);
         //shell_printf("max:     ref %6d ch %6d\r\n", maxr, maxs);
      }

      //shell_printf("callback count: %d\r\n", stat.callback_count);
      //shell_printf("interval cycle: %d\r\n", stat.interval_cycles);
      //shell_printf("busy cycle: %d\r\n", stat.busy_cycles);
      //shell_printf("load: %d\r\n", stat.busy_cycles * 100 / stat.interval_cycles);
      //  extern int awd_count;
      //  shell_printf("awd: %d\r\n", awd_count);
   }
#endif

const char NANOVNA_VERSION[] = VERSION;

VNA_SHELL_FUNCTION(cmd_version)
{
  (void)argc;
  (void)argv;
  shell_printf("%s\r\n", NANOVNA_VERSION);
}

VNA_SHELL_FUNCTION(cmd_vbat)
{
  (void)argc;
  (void)argv;
  shell_printf("%d mV\r\n", adc_vbat_read());
}

VNA_SHELL_FUNCTION(cmd_vbat_offset)
{
   if (argc < 1)
   {
      shell_printf("%d\r\n", config.vbat_offset_mv);
      return;
   }
   config.vbat_offset_mv = (int16_t)my_atoi(argv[0]);
}

#ifdef ENABLE_INFO_COMMAND
   VNA_SHELL_FUNCTION(cmd_info)
   {
      (void)argc;
      (void)argv;
      int i = 0;
      while (info_about[i])
         shell_printf("%s\r\n", info_about[i++]);
   }
#endif

#ifdef ENABLE_COLOR_COMMAND
   VNA_SHELL_FUNCTION(cmd_color)
   {
      uint32_t color;
      int i;

      if (argc < 2)
      {
         shell_printf("usage: color {id} {rgb24}\r\n");

         for (i = -3; i < TRACES_MAX; i++)
         {
            #if 0
               switch (i)
               {
                  case -3: color = config.grid_color; break;
                  case -2: color = config.menu_normal_color; break;
                  case -1: color = config.menu_active_color; break;
                  default: color = config.trace_color[i];break;
               }
            #else
               // WARNING!!! Dirty hack for size, depend from config struct
               color = config.trace_color[i];
            #endif
            color =  ((color >>  3) & 0x001c00) |
                     ((color >>  5) & 0x0000f8) |
                     ((color << 16) & 0xf80000) |
                     ((color << 13) & 0x00e000);
            //color = (color>>8)|(color<<8);
            //color = ((color<<8)&0xF80000)|((color<<5)&0x00FC00)|((color<<3)&0x0000F8);
            shell_printf("   %d: 0x%06x\r\n", i, color);
         }
         return;
      }

      i = my_atoi(argv[0]);
      if (i < -3 && i >= TRACES_MAX)
         return;

      color = RGBHEX(my_atoui(argv[1]));

      #if 0
         switch (i)
         {
            case -3: config.grid_color = color; break;
            case -2: config.menu_normal_color = color; break;
            case -1: config.menu_active_color = color; break;
            default: config.trace_color[i] = color;break;
         }
      #else
         // WARNING!!! Dirty hack for size, depend from config struct
         config.trace_color[i] = color;
      #endif

      // Redraw all
      redraw_request|= REDRAW_AREA;
   }
#endif

#ifdef ENABLE_I2C_COMMAND
   VNA_SHELL_FUNCTION(cmd_i2c)
   {
      if (argc < 3)
      {
         shell_printf("usage: i2c page reg data\r\n");
         return;
      }

      const uint8_t page = my_atoui(argv[0]);
      const uint8_t reg  = my_atoui(argv[1]);
      const uint8_t data = my_atoui(argv[2]);
      tlv320aic3204_write_reg(page, reg, data);
   }
#endif

#ifdef ENABLE_THREADS_COMMAND
   #if CH_CFG_USE_REGISTRY == FALSE
      #error "Threads Requite enabled CH_CFG_USE_REGISTRY in chconf.h"
   #endif
   VNA_SHELL_FUNCTION(cmd_threads)
   {
      static const char *states[] = {CH_STATE_NAMES};
      thread_t *tp;
      (void)argc;
      (void)argv;
      shell_printf("stklimit|   stack|stk free|    addr|refs|prio|    state|        name"VNA_SHELL_NEWLINE_STR);

      tp = chRegFirstThread();
      do {
         uint32_t max_stack_use = 0U;
         #if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
            uint32_t stklimit = (uint32_t)tp->wabase;
            #if CH_DBG_FILL_THREADS == TRUE
               uint8_t *p = (uint8_t *)tp->wabase; while(p[max_stack_use]==CH_DBG_STACK_FILL_VALUE) max_stack_use++;
            #endif
         #else
            uint32_t stklimit = 0U;
         #endif

         shell_printf("%08x|%08x|%08x|%08x|%4u|%4u|%9s|%12s"VNA_SHELL_NEWLINE_STR,
             stklimit, (uint32_t)tp->ctx.sp, max_stack_use, (uint32_t)tp,
             (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
             tp->name == NULL ? "" : tp->name);
         tp = chRegNextThread(tp);
      } while (tp != NULL);
   }
#endif

//=============================================================================
VNA_SHELL_FUNCTION(cmd_help);

#pragma pack(push, 2)
   typedef struct
   {
      const char     *sc_name;
      vna_shellcmd_t sc_function;
      uint16_t       flags;
   } VNAShellCommand;
#pragma pack(pop)

// Some commands can executed only in sweep thread, not in main cycle
#define CMD_WAIT_MUTEX  1
static const VNAShellCommand commands[] =
{
   {"version"          , cmd_version     , 0},
   {"reset"            , cmd_reset       , 0},
   {"freq"             , cmd_freq        , CMD_WAIT_MUTEX},
   {"offset"           , cmd_offset      , CMD_WAIT_MUTEX},
   {"bandwidth"        , cmd_bandwidth   , 0},
   #ifdef __USE_RTC__
      {"time"          , cmd_time        , 0},
   #endif
   {"dac"              , cmd_dac         , 0},
   {"saveconfig"       , cmd_saveconfig  , 0},
   {"clearconfig"      , cmd_clearconfig , 0},
   {"data"             , cmd_data        , CMD_WAIT_MUTEX},
   #ifdef ENABLED_DUMP
      {"dump"          , cmd_dump        , 0},
   #endif
   {"frequencies"      , cmd_frequencies , 0},
   #ifdef ENABLE_PORT_COMMAND
      {"port"          , cmd_port        , 0},
   #endif
   #ifdef ENABLE_STAT_COMMAND
      {"stat"          , cmd_stat        , CMD_WAIT_MUTEX},
   #endif
   {"gain"             , cmd_gain        , CMD_WAIT_MUTEX},
   {"power"            , cmd_power       , 0},
   {"sample"           , cmd_sample      , 0},
   #ifdef USE_INTEGRATOR
		{"integrator"    , cmd_integrator  , 0},
   #endif
	#ifdef ENABLE_GAMMA_COMMAND
      {"gamma"         , cmd_gamma       , 0},
   #endif
   {"scan_bin"         , cmd_scan_bin    , CMD_WAIT_MUTEX},
   {"scan"             , cmd_scan        , CMD_WAIT_MUTEX},
   {"sweep"            , cmd_sweep       , 0},
   #ifdef ENABLE_TEST_COMMAND
      {"test"          , cmd_test        , 0},
   #endif
   #ifdef ENABLE_TOUCH_COMMAND
		{"touchcal"      , cmd_touchcal    , CMD_WAIT_MUTEX},
   	{"touchtest"     , cmd_touchtest   , CMD_WAIT_MUTEX},
	#endif
	{"pause"            , cmd_pause       , 0},
   {"resume"           , cmd_resume      , 0},
   {"cal"              , cmd_cal         , CMD_WAIT_MUTEX},
   {"save"             , cmd_save        , 0},
   {"recall"           , cmd_recall      , CMD_WAIT_MUTEX},
   {"trace"            , cmd_trace       , 0},
   {"marker"           , cmd_marker      , 0},
   {"edelay"           , cmd_edelay      , 0},
   {"capture"          , cmd_capture     , CMD_WAIT_MUTEX},
   {"vbat"             , cmd_vbat        , 0},
   {"vbat_offset"      , cmd_vbat_offset , 0},
   {"transform"        , cmd_transform   , 0},
   {"threshold"        , cmd_threshold   , 0},
   #if defined(__USE_SD_CARD__) && defined(__USE_SD_CARD_CMDS__)
      {"sd_list"       , cmd_sd_list     , CMD_WAIT_MUTEX},
      {"sd_readfile"   , cmd_sd_readfile , CMD_WAIT_MUTEX},
   #endif
   {"help"             , cmd_help        , 0},
   #ifdef ENABLE_INFO_COMMAND
      {"info"          , cmd_info        , 0},
   #endif
   #ifdef ENABLE_COLOR_COMMAND
      {"color"         , cmd_color       , 0},
   #endif
   #ifdef ENABLE_I2C_COMMAND
      {"i2c"           , cmd_i2c         , CMD_WAIT_MUTEX},
   #endif
   #ifdef ENABLE_THREADS_COMMAND
      {"threads"       , cmd_threads     , 0},
   #endif
   {NULL               , NULL            , 0}
};

VNA_SHELL_FUNCTION(cmd_help)
{
   (void)argc;
   (void)argv;
   const VNAShellCommand *scp = commands;
   shell_printf("Commands:");
   while (scp->sc_name != NULL)
   {
      shell_printf(" %s", scp->sc_name);
      scp++;
   }
   shell_printf(VNA_SHELL_NEWLINE_STR);
   return;
}

/*
 * VNA shell functions
 */

//
// Read command line from shell_stream
//
static int VNAShell_readLine(char *line, int max_size)
{
  // Read line from input stream
  uint8_t c;
  char *ptr = line;
  while (1) {
    // Return 0 only if stream not active
    if (streamRead(shell_stream, &c, 1) == 0)
      return 0;
    // Backspace or Delete
    if (c == 8 || c == 0x7f) {
      if (ptr != line) {
        static const char backspace[] = {0x08, 0x20, 0x08, 0x00};
        shell_printf(backspace);
        ptr--;
      }
      continue;
    }
    // New line (Enter)
    if (c == '\r') {
      shell_printf(VNA_SHELL_NEWLINE_STR);
      *ptr = 0;
      return 1;
    }
    // Others (skip)
    if (c < 0x20)
      continue;
    // Store
    if (ptr < line + max_size - 1) {
      streamPut(shell_stream, c); // Echo
      *ptr++ = (char)c;
    }
  }
  return 0;
}

//
// Parse and run command line
//
static void VNAShell_executeLine(char *line)
{
  // Parse and execute line
  char *lp = line, *ep;
  shell_nargs = 0;
  while (*lp != 0) {
    // Skipping white space and tabs at string begin.
    while (*lp == ' ' || *lp == '\t') lp++;
    // If an argument starts with a double quote then its delimiter is another quote, else
    // delimiter is white space.
    ep = (*lp == '"') ? strpbrk(++lp, "\"") : strpbrk(lp, " \t");
    // Store in args string
    shell_args[shell_nargs++] = lp;
    // Stop, end of input string
    if ((lp = ep) == NULL) break;
    // Argument limits check
    if (shell_nargs > VNA_SHELL_MAX_ARGUMENTS) {
      shell_printf("too many arguments, max " define_to_STR(
          VNA_SHELL_MAX_ARGUMENTS) "" VNA_SHELL_NEWLINE_STR);
      return;
    }
    // Set zero at the end of string and continue check
    *lp++ = 0;
  }
  if (shell_nargs == 0) return;
  // Execute line
  const VNAShellCommand *scp;
  for (scp = commands; scp->sc_name != NULL; scp++) {
    if (strcmp(scp->sc_name, shell_args[0]) == 0) {
      if (scp->flags & CMD_WAIT_MUTEX) {
        shell_function = scp->sc_function;
        // Wait execute command in sweep thread
        do {
          osalThreadSleepMilliseconds(100);
        } while (shell_function);
      } else {
        scp->sc_function(shell_nargs - 1, &shell_args[1]);
      }
      return;
    }
  }
  shell_printf("%s?" VNA_SHELL_NEWLINE_STR, shell_args[0]);
}

#ifdef VNA_SHELL_THREAD
static THD_WORKING_AREA(waThread2, /* cmd_* max stack size + alpha */442);
THD_FUNCTION(myshellThread, p)
{
  (void)p;
  chRegSetThreadName("shell");
  shell_printf(VNA_SHELL_NEWLINE_STR"NanoVNA Shell"VNA_SHELL_NEWLINE_STR);
  while (true) {
    shell_printf(VNA_SHELL_PROMPT_STR);
    if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
      VNAShell_executeLine(shell_line);
    else // Putting a delay in order to avoid an endless loop trying to read an unavailable stream.
      osalThreadSleepMilliseconds(100);
  }
}
#endif

// I2C clock bus setting: depend from STM32_I2C1SW in mcuconf.h
static const I2CConfig i2ccfg = {
  .timingr =     // TIMINGR register initialization. (use I2C timing configuration tool for STM32F3xx and STM32F0xx microcontrollers (AN4235))
#if STM32_I2C1SW == STM32_I2C1SW_HSI
  // STM32_I2C1SW == STM32_I2C1SW_HSI     (HSI=8MHz)
  // 400kHz @ HSI 8MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
  STM32_TIMINGR_PRESC(0U)  |
  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(1U) |
  STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U),
  // Old values voodoo magic 400kHz @ HSI 8MHz
  //0x00300506,
#elif  STM32_I2C1SW == STM32_I2C1SW_SYSCLK
  // STM32_I2C1SW == STM32_I2C1SW_SYSCLK  (SYSCLK = 48MHz)
  // 400kHz @ SYSCLK 48MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
//  STM32_TIMINGR_PRESC(5U)  |
//  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
//  STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U),
  // 600kHz @ SYSCLK 48MHz, manually get values, x1.5 I2C speed, but need calc timings
  STM32_TIMINGR_PRESC(3U)  |
  STM32_TIMINGR_SCLDEL(2U) | STM32_TIMINGR_SDADEL(2U) |
  STM32_TIMINGR_SCLH(4U)   | STM32_TIMINGR_SCLL(4U),
#else
#error "Need Define STM32_I2C1SW and set correct TIMINGR settings"
#endif
  .cr1 = 0,     // CR1 register initialization.
  .cr2 = 0      // CR2 register initialization.
};

static DACConfig dac1cfg1 = {
   init:     DEFAULT_DAC_VALUE,
   datamode: DAC_DHRM_12BIT_RIGHT
};

void config_reset(void)
{
   memset(&config, 0xff, sizeof(config));   // default flash byte value

   config.magic                   = CONFIG_MAGIC;

   config.dac_value               = DEFAULT_DAC_VALUE;

   config.grid_color              = DEFAULT_GRID_COLOR;
   config.menu_normal_color       = DEFAULT_MENU_COLOR;
   config.menu_active_color       = DEFAULT_MENU_ACTIVE_COLOR;

   config.trace_color[0]          = DEFAULT_TRACE_1_COLOR;
   config.trace_color[1]          = DEFAULT_TRACE_2_COLOR;
   config.trace_color[2]          = DEFAULT_TRACE_3_COLOR;
   config.trace_color[3]          = DEFAULT_TRACE_4_COLOR;

   // 2.8 inch LCD panel
   config.touch_cal[0]            = 357;
   config.touch_cal[1]            = 563;
   config.touch_cal[2]            = 159;
   config.touch_cal[3]            = 198;

   // 4.0" LCD
// config.touch_cal[0]            = 252;
// config.touch_cal[1]            = 450;
// config.touch_cal[2]            = 111;
// config.touch_cal[3]            = 150;

   config.harmonic_freq_threshold = DEFAULT_FREQUENCY_THRESHOLD;

   config.vbat_offset_mv          = DEFAULT_BATTERY_OFFSET_MV;

   config.bandwidth               = BANDWIDTH_2000;

   config.current_id              = 0;

   config.current_trace           = 0;

   config.flags                   = 0;

   config.integrator_coeff        = 1.0f;

   config.checksum                = 0;
}

// Main thread stack size defined in makefile USE_PROCESS_STACKSIZE = 0x200
// Profile stack usage (enable threads command by def ENABLE_THREADS_COMMAND) show:
// Stack maximum usage = 472 bytes (need test more and run all commands), free stack = 40 bytes
//
int main(void)
{
   halInit();
   chSysInit();

   config_reset();

   #ifdef __USE_RTC__
      rtc_init();
   #endif

   ili9341_init();

   // splash/logo screen
   ili9341_set_foreground(DEFAULT_FG_COLOR);
   ili9341_set_background(DEFAULT_BG_COLOR);
   ili9341_clear_screen();
   #if !defined(NANOVNA_H4)
      ili9341_drawstring_size("NanoVNA-H", (LCD_WIDTH - (FONT_MAX_WIDTH * 3 * 8)) / 2, (LCD_HEIGHT - (FONT_GET_HEIGHT * 3)) / 3, 3, false);
   #else
      ili9341_drawstring_size("NanoVNA-H4", (LCD_WIDTH - (FONT_MAX_WIDTH * 3 * 8)) / 2, (LCD_HEIGHT - (FONT_GET_HEIGHT * 3)) / 3, 3, false);
   #endif

   #ifdef USE_VARIABLE_OFFSET
      generate_DSP_Table(FREQUENCY_OFFSET);
   #endif

   // restore config
   bool config_resetted = false;
   if (config_recall() >= 0)
   {   // a bit of simple config error checking
      uint16_t errors = 0;
      if (config.vbat_offset_mv >  2000)                                        errors |= 1u << 0;
      if (config.bandwidth      == 0xffff)                                      errors |= 1u << 1;
      if (config.current_id     == 0xff)                                        errors |= 1u << 2;
      if (config.current_trace  <  -1    || config.current_trace >= TRACES_MAX) errors |= 1u << 3;
      if (config.integrator_coeff <= 0.0f || config.integrator_coeff > 1.0f)    errors |= 1u << 4;
      if (errors)
      {   // configuration error .. reset it
         config_reset();
         config_save();
         config_resetted = true;
         ili9341_drawstring_size("Main config was reset due to"
                                 "\nerror or config reset required",
                                 OFFSETX + 5, (LCD_HEIGHT - (FONT_GET_HEIGHT * 3)) / 3 + 50 + ((FONT_GET_HEIGHT + 5) * 0), 2, false);

         char str[22];
         plot_printf(str, sizeof(str), "error 0x%04x", errors);
         //plot_printf(str, sizeof(str), "error 0b%10b", errors);
         ili9341_drawstring_size(str, OFFSETX + 5, (LCD_HEIGHT - (FONT_GET_HEIGHT * 3)) / 3 + 50 + ((FONT_GET_HEIGHT + 5) * 4), 2, false);
      }
   }

   HPF_DISABLE;   // disable the new HPF .. for now

   //palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(1) | PAL_STM32_OTYPE_OPENDRAIN);
   //palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(1) | PAL_STM32_OTYPE_OPENDRAIN);
   i2cStart(&I2CD1, &i2ccfg);

   si5351_init();

   // MCO on PA8
   //palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(0));

   /*
    * Initializes a serial-over-USB CDC driver.
    */
   sduObjectInit(&SDU1);
   sduStart(&SDU1, &serusbcfg);

   /*
    * Activates the USB driver and then the USB bus pull-up on D+.
    * Note, a delay is inserted in order to not have to disconnect the cable after a reset.
    */
   usbDisconnectBus(serusbcfg.usbp);
   chThdSleepMilliseconds(100);
   usbStart(serusbcfg.usbp, &usbcfg);
   usbConnectBus(serusbcfg.usbp);

   // test only
   //const int cal_size = sizeof(properties_t);

   // restore frequencies and calibration 0 slot properties from flash memory
   load_properties((config.current_id < SAVEAREA_MAX) ? config.current_id : 0);

   dac1cfg1.init = config.dac_value;
   /*
    * Starting DAC1 driver, setting up the output pin as analog as suggested by the Reference Manual.
    */
   dacStart(&DACD2, &dac1cfg1);

   /*
    * I2S Initialize
    */
   tlv320aic3204_init();
   i2sInit();
   i2sObjectInit(&I2SD2);
   i2sStart(&I2SD2, &i2sconfig);
   i2sStartExchange(&I2SD2);

   ui_init();

   {   // let the splash/logo stay on screen for a short while
      // .. also, while we're sat here doing nothing we can auto adjust the touch threshold
      adc_stop();

      const int count = config_resetted ? 2500 : 500;   // leave splash screen on for a little longer to inform the user of a config reset
      uint16_t peak = 0;
      int i;
      for (i = 0; i < count; i++)
      {
         const int16_t value = adc_single_read(ADC_TOUCH_Y);
         if (value > 0)
            if (peak < value)
                peak = value;
         chThdSleepMilliseconds(1);
      }
      if (peak > 0)
      {
         peak *= 2;
         if (peak < TOUCH_THRESHOLD_MIN)     peak = TOUCH_THRESHOLD_MIN;
         else
            if (peak > TOUCH_THRESHOLD_DEFAULT) peak = TOUCH_THRESHOLD_DEFAULT;
         touch_threshold = peak;
      }
   }

   // Initialize graph plotting
   plot_init();
   redraw_frame();

   chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO - 1, Thread1, NULL);

   touch_start_watchdog();

   while (1)
   {
      if (SDU1.config->usbp->state == USB_ACTIVE)
      {
         #ifdef VNA_SHELL_THREAD
            #if CH_CFG_USE_WAITEXIT == FALSE
               #error "VNA_SHELL_THREAD use chThdWait, need enable CH_CFG_USE_WAITEXIT in chconf.h"
            #endif

            thread_t *shelltp = chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO + 1, myshellThread, NULL);
            chThdWait(shelltp);
         #else
            shell_printf(VNA_SHELL_NEWLINE_STR"NanoVNA Shell"VNA_SHELL_NEWLINE_STR);
            do {
               shell_printf(VNA_SHELL_PROMPT_STR);
               if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
                  VNAShell_executeLine(shell_line);
               else
                  chThdSleepMilliseconds(200);
            } while (SDU1.config->usbp->state == USB_ACTIVE);
         #endif
      }

      chThdSleepMilliseconds(1000);
   }
}

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler(void);

void hard_fault_handler_c(uint32_t *sp) __attribute__((naked));

void HardFault_Handler(void)
{
  uint32_t *sp;
  //__asm volatile ("mrs %0, msp \n\t": "=r" (sp) );
  __asm volatile("mrs %0, psp \n\t" : "=r"(sp));
  hard_fault_handler_c(sp);
}

void hard_fault_handler_c(uint32_t *sp)
{
#if 0
  uint32_t r0  = sp[0];
  uint32_t r1  = sp[1];
  uint32_t r2  = sp[2];
  uint32_t r3  = sp[3];
  register uint32_t  r4 __asm("r4");
  register uint32_t  r5 __asm("r5");
  register uint32_t  r6 __asm("r6");
  register uint32_t  r7 __asm("r7");
  register uint32_t  r8 __asm("r8");
  register uint32_t  r9 __asm("r9");
  register uint32_t r10 __asm("r10");
  register uint32_t r11 __asm("r11");
  uint32_t r12 = sp[4];
  uint32_t lr  = sp[5];
  uint32_t pc  = sp[6];
  uint32_t psr = sp[7];
  int y = 0;
  int x = 20;
  char buf[16];
  ili9341_set_background(0x0000);
  ili9341_set_foreground(0xFFFF);
  plot_printf(buf, sizeof(buf), "SP  0x%08x",  (uint32_t)sp);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R0  0x%08x",  r0);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R1  0x%08x",  r1);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R2  0x%08x",  r2);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R3  0x%08x",  r3);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R4  0x%08x",  r4);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R5  0x%08x",  r5);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R6  0x%08x",  r6);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R7  0x%08x",  r7);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R8  0x%08x",  r8);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R9  0x%08x",  r9);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R10 0x%08x", r10);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R11 0x%08x", r11);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R12 0x%08x", r12);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "LR  0x%08x",  lr);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "PC  0x%08x",  pc);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "PSR 0x%08x", psr);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);

  shell_printf("===================================\r\n");
#else
  (void)sp;
#endif
  while (true) {
  }
}
