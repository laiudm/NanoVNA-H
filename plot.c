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

#include <arm_math.h>
//#include <math.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"
#ifdef USE_LC_MATCHING
	#include "lc_matching.h"
#endif

static void cell_draw_marker_info(int x0, int y0);
static void draw_battery_status(void);

static int16_t grid_offset;
static int16_t grid_width;

int16_t area_width  = AREA_WIDTH_NORMAL;
int16_t area_height = AREA_HEIGHT_NORMAL;

// Cell render use spi buffer
typedef uint16_t pixel_t;
pixel_t *cell_buffer = (pixel_t *)spi_buffer;

// Cell size
// Depends from spi_buffer size, CELLWIDTH * CELLHEIGHT * sizeof(pixel) <= sizeof(spi_buffer)
#define CELLWIDTH			64
#define CELLHEIGHT		32

// Check buffer size
#if ((CELLWIDTH * CELLHEIGHT) > SPI_BUFFER_SIZE)
	#error "Too small spi_buffer size SPI_BUFFER_SIZE < CELLWIDTH*CELLHEIGH"
#endif

// indicate dirty cells (not redraw if cell data not changed)
#define MAX_MARKMAP_X    ((LCD_WIDTH  + CELLWIDTH  - 1) / CELLWIDTH)
#define MAX_MARKMAP_Y    ((LCD_HEIGHT + CELLHEIGHT - 1) / CELLHEIGHT)

// Define markmap mask size
#if (MAX_MARKMAP_X <= 8)
	typedef uint8_t map_t;
#elif (MAX_MARKMAP_X <= 16)
	typedef uint16_t map_t;
#elif (MAX_MARKMAP_X <= 32)
	typedef uint32_t map_t;
#endif

map_t   markmap[2][MAX_MARKMAP_Y];
uint8_t current_mappage = 0;

// Trace data cache, for faster redraw cells
//   CELL_X[16:31] x position
//   CELL_Y[ 0:15] y position
typedef uint32_t index_t;
static index_t trace_index[TRACES_MAX][POINTS_COUNT];

#define INDEX(x, y) ((((index_t)x) << 16) | (((index_t)y)))
#define CELL_X(i)   (int)(((i) >> 16))
#define CELL_Y(i)   (int)(((i) & 0xFFFF))

#if 1
	#define float2int(v) ((int)(v))
#else
	static int float2int(float v)
	{
		if (v < 0) return v - 0.5f;
		if (v > 0) return v + 0.5f;
		return 0.0f;
	}
#endif

void update_grid(void)
{
  uint32_t gdigit = 100000000;
  uint32_t fstart = get_sweep_frequency(ST_START);
  uint32_t fspan  = get_sweep_frequency(ST_SPAN);
  uint32_t grid;

  while (gdigit > 100) {
    grid = 5 * gdigit;
    if (fspan / grid >= 4)
      break;
    grid = 2 * gdigit;
    if (fspan / grid >= 4)
      break;
    grid = gdigit;
    if (fspan / grid >= 4)
      break;
    gdigit /= 10;
  }

  grid_offset = (WIDTH) * ((fstart % grid) / 100) / (fspan / 100);
  grid_width  = (WIDTH) * (grid / 100) / (fspan / 1000);

  force_set_markmap();
  redraw_request |= REDRAW_FREQUENCY;
}

static inline int circle_inout(int x, int y, int r)
{
	int d = x*x + y*y - r*r;
	if (d < -r)
		return 1;
	if (d >  r)
		return -1;
	return 0;
}

static int
polar_grid(int x, int y)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;

  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0) return 0;
  if (d == 0) return 1;

  // vertical and horizontal axis
  if (x == 0 || y == 0) return 1;

  d = circle_inout(x, y, P_RADIUS / 5);
  if (d == 0) return 1;
  if (d > 0) return 0;

  d = circle_inout(x, y, P_RADIUS * 2 / 5);
  if (d == 0) return 1;
  if (d > 0) return 0;

  // cross sloping lines
  if (x == y || x == -y) return 1;

  d = circle_inout(x, y, P_RADIUS * 3 / 5);
  if (d == 0) return 1;
  if (d > 0) return 0;

  d = circle_inout(x, y, P_RADIUS * 4 / 5);
  if (d == 0) return 1;
  return 0;
}

/*
 * Constant Resistance circle: (u - r/(r+1))^2 + v^2 = 1/(r+1)^2
 * Constant Reactance circle:  (u - 1)^2 + (v-1/x)^2 = 1/x^2
 */
static int smith_grid(int x, int y)
{
	int d;

	// offset to center
	x -= P_CENTER_X;
	y -= P_CENTER_Y;

	// outer circle
	d = circle_inout(x, y, P_RADIUS);
	if (d  < 0) return 0;
	if (d == 0) return 1;

	// horizontal axis
	if (y == 0) return 1;

	#ifdef USE_LC_MATCHING
		// Unity conductance circle .. left circle
		if (circle_inout(x + P_RADIUS / 2, y, P_RADIUS / 2) == 0) return 1;
	#endif

	// shift circle center to right origin
	x -= P_RADIUS;

	// Constant Reactance Circle: 2j : R/2 = P_RADIUS/2
	if (circle_inout(x, y + P_RADIUS / 2, P_RADIUS / 2) == 0) return 1;
	if (circle_inout(x, y - P_RADIUS / 2, P_RADIUS / 2) == 0) return 1;

	// Constant Resistance Circle: 3 : R/4 = P_RADIUS/4
	d = circle_inout(x + P_RADIUS / 4, y, P_RADIUS / 4);
	if (d  > 0) return 0;
	if (d == 0) return 1;

	// Constant Reactance Circle: 1j : R = P_RADIUS
	if (circle_inout(x, y + P_RADIUS, P_RADIUS) == 0) return 1;
	if (circle_inout(x, y - P_RADIUS, P_RADIUS) == 0) return 1;

	// Constant Resistance Circle: 1 : R/2
	d = circle_inout(x + P_RADIUS / 2, y, P_RADIUS / 2);
	if (d  > 0) return 0;
	if (d == 0) return 1;

	// Constant Reactance Circle: 1/2j : R*2
	if (circle_inout(x, y + P_RADIUS * 2, P_RADIUS * 2) == 0) return 1;
	if (circle_inout(x, y - P_RADIUS * 2, P_RADIUS * 2) == 0) return 1;

	// Constant Resistance Circle: 1/3 : R*3/4
	if (circle_inout(x + P_RADIUS * 3 / 4, y, P_RADIUS * 3 / 4) == 0) return 1;

	return 0;
}

#if 0
static int smith_grid2(int x, int y, float scale)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;

  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0)
    return 0;
  if (d == 0)
    return 1;

  // shift circle center to right origin
  x -= P_RADIUS * scale;

  // Constant Reactance Circle: 2j : R/2 = 58
  if (circle_inout(x, y+58*scale, 58*scale) == 0)
    return 1;
  if (circle_inout(x, y-58*scale, 58*scale) == 0)
    return 1;
#if 0
  // Constant Resistance Circle: 3 : R/4 = 29
  d = circle_inout(x+29*scale, y, 29*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-29*scale, y, 29*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
#endif

  // Constant Reactance Circle: 1j : R = 116
  if (circle_inout(x, y+116*scale, 116*scale) == 0)
    return 1;
  if (circle_inout(x, y-116*scale, 116*scale) == 0)
    return 1;

  // Constant Resistance Circle: 1 : R/2 = 58
  d = circle_inout(x+58*scale, y, 58*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-58*scale, y, 58*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Reactance Circle: 1/2j : R*2 = 232
  if (circle_inout(x, y+232*scale, 232*scale) == 0)
    return 1;
  if (circle_inout(x, y-232*scale, 232*scale) == 0)
    return 1;

#if 0
  // Constant Resistance Circle: 1/3 : R*3/4 = 87
  d = circle_inout(x+87*scale, y, 87*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x+87*scale, y, 87*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
#endif

  // Constant Resistance Circle: 0 : R
  d = circle_inout(x+P_RADIUS*scale, y, P_RADIUS*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-P_RADIUS*scale, y, P_RADIUS*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Resistance Circle: -1/3 : R*3/2 = 174
  d = circle_inout(x+174*scale, y, 174*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-174*scale, y, 174*scale);
  //if (d > 0) return 0;
  if (d == 0) return 1;
  return 0;
}
#endif

#if 0
const int cirs[][4] = {
  { 0, 58/2, 58/2, 0 },    // Constant Reactance Circle: 2j : R/2 = 58
  { 29/2, 0, 29/2, 1 },    // Constant Resistance Circle: 3 : R/4 = 29
  { 0, 115/2, 115/2, 0 },  // Constant Reactance Circle: 1j : R = 115
  { 58/2, 0, 58/2, 1 },    // Constant Resistance Circle: 1 : R/2 = 58
  { 0, 230/2, 230/2, 0 },  // Constant Reactance Circle: 1/2j : R*2 = 230
  { 86/2, 0, 86/2, 1 },    // Constant Resistance Circle: 1/3 : R*3/4 = 86
  { 0, 460/2, 460/2, 0 },  // Constant Reactance Circle: 1/4j : R*4 = 460
  { 115/2, 0, 115/2, 1 },  // Constant Resistance Circle: 0 : R
  { 173/2, 0, 173/2, 1 },  // Constant Resistance Circle: -1/3 : R*3/2 = 173
  { 0, 0, 0, 0 } // sentinel
};

static int smith_grid3(int x, int y)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;

  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0)
    return 0;
  if (d == 0)
    return 1;

  // shift circle center to right origin
  x -= P_RADIUS /2;

  int i;
  for (i = 0; cirs[i][2]; i++) {
    d = circle_inout(x+cirs[i][0], y+cirs[i][1], cirs[i][2]);
    if (d == 0)
      return 1;
    if (d > 0 && cirs[i][3])
      return 0;
    d = circle_inout(x-cirs[i][0], y-cirs[i][1], cirs[i][2]);
    if (d == 0)
      return 1;
    if (d > 0 && cirs[i][3])
      return 0;
  }
  return 0;
}
#endif

#if 0
static int rectangular_grid(int x, int y)
{
  //#define FREQ(x) (((x) * (fspan / 1000) / (WIDTH-1)) * 1000 + fstart)
  //int32_t n = FREQ(x-1) / fgrid;
  //int32_t m = FREQ(x) / fgrid;
  //if ((m - n) > 0)
  //if (((x * 6) % (WIDTH-1)) < 6)
  //if (((x - grid_offset) % grid_width) == 0)
  if (x == 0 || x == WIDTH-1)
    return 1;
  if ((y % GRIDY) == 0)
    return 1;
  if ((((x + grid_offset) * 10) % grid_width) < 10)
    return 1;
  return 0;
}
#endif

static int rectangular_grid_x(int x)
{
	x -= CELLOFFSETX;

	if (x < 0)
		return 0;

	if (x == 0 || x == WIDTH)
		return 1;

	if ((((x + grid_offset) * 10) % grid_width) < 10)
		return 1;

	return 0;
}

static int rectangular_grid_y(int y)
{
	if (y < 0)
		return 0;
	if ((y % GRIDY) == 0)
		return 1;
	return 0;
}

#if 0
int set_strut_grid(int x)
{
  uint16_t *buf = spi_buffer;
  int y;

  for (y = 0; y < HEIGHT; y++) {
    int c = rectangular_grid(x, y);
    c |= smith_grid(x, y);
    *buf++ = c;
  }
  return y;
}

void draw_on_strut(int v0, int d, int color)
{
  int v;
  int v1 = v0 + d;
  if (v0 < 0) v0 = 0;
  if (v1 < 0) v1 = 0;
  if (v0 >= HEIGHT) v0 = HEIGHT-1;
  if (v1 >= HEIGHT) v1 = HEIGHT-1;
  if (v0 == v1) {
    v = v0; d = 2;
  } else if (v0 < v1) {
    v = v0; d = v1 - v0 + 1;
  } else {
    v = v1; d = v0 - v1 + 1;
  }
  while (d-- > 0)
    spi_buffer[v++] |= color;
}
#endif

float pwr(const float *v)
{
	return (v[0] * v[0]) + (v[1] * v[1]);
}

/*
 * calculate log10(abs(gamma))
 */
float logmag(const float *v)
{
	const float p = pwr(v);
	return (p == 0.0f) ? 0.0f : 10.0f * log10f(p);
}

/*
 * calculate abs(gamma)
 */
float linear(const float *v)
{
	return -sqrtf(pwr(v));
}

/*
 * calculate phase[-2:2] of coefficient
 */
float phase(const float *v)
{
	return (v[0] == 0.0f) ? 0.0f : 2.0f * atan2f(v[1], v[0]) / VNA_PI * 90.0f;
}

/*
 * calculate groupdelay
 */
float groupdelay(const float *v, const float *w, float deltaf)
{
	#if 1
		// atan(w)-atan(v) = atan((w-v)/(1+wv))
		const float r = (w[0] * v[1]) - (w[1] * v[0]);
		const float i = (w[0] * v[0]) + (w[1] * v[1]);
		return (deltaf == 0.0f || i == 0.0f) ? 0.0f : atan2f(r, i) / ((float)(2 * VNA_PI) * deltaf);
	#else
		return (deltaf == 0.0f || w[1] == 0.0f || v[1]) ? 0.0f : (atan2f(w[0], w[1]) - atan2f(v[0], v[1])) / ((float)(2 * VNA_PI) * deltaf);
	#endif
}

/*
 * calculate vswr; (1+gamma)/(1-gamma)
 */
float swr(const float *v)
{
	const float x = -linear(v);
	return (x >= 1) ? INFINITY : (1.0f + x) / (1.0f - x);
}

void impedance(const float *v, float *zr, float *zi, float z0)
{
	const float d = z0 / ((1.0f - v[0]) * (1.0f - v[0]) + (v[1] * v[1]));
	if (zr)
		*zr = ((1.0f + v[0]) * (1.0f - v[0]) - (v[1] * v[1])) * d;
	if (zi)
		*zi = 2.0f * v[1] * d;
}
/*
float resistance(const float *v, float z0)
{
	float zr;
	impedance(v, &zr, NULL, z0);
	return zr;
}

float reactance(const float *v, float z0)
{
	float zi;
	impedance(v, NULL, &zi, z0);
	return zi;
}
*/
float qualityfactor(const float *v)
{
	const float i = 2.0f * v[1];
	const float r = (1.0f + v[0]) * (1.0f - v[0]) - (v[1] * v[1]);
	return (r != 0.0f) ? fabs(i / r) : INFINITY;
}

static void cartesian_scale(float re, float im, int *xp, int *yp, float scale)
{
// const float scale = 4e-3;

//	const int max = P_RADIUS;
	const int max = P_RADIUS * 1.5;

	int x = float2int(re * P_RADIUS * scale);
	int y = float2int(im * P_RADIUS * scale);

	if (x < -max) x = -max;
	else
	if (x >  max) x =  max;

	if (y < -max) y = -max;
	else
	if (y >  max) y =  max;

	*xp = P_CENTER_X + x;
	*yp = P_CENTER_Y - y;
}

float groupdelay_from_array(int i, float array[POINTS_COUNT][2])
{
  const int bottom = (i ==   0) ?   0 : i - 1;
  const int top    = (i == sweep_points - 1) ? sweep_points-1 : i + 1;
  const float deltaf = frequencies[top] - frequencies[bottom];
  return groupdelay(array[bottom], array[top], deltaf);
}

static index_t trace_into_index(int t, int i, float array[POINTS_COUNT][2])
{
	int   x = 0;
	int   y = 0;
	float v            = 0.0f;
	const float z0     = 50.0f;
	const float *coeff = array[i];
	const float scale  = 1.0f / get_trace_scale(t);
	const float refpos = NGRIDY - get_trace_refpos(t);

	switch (trace[t].type)
	{
		case TRC_LOGMAG:
			v = logmag(coeff);
			break;
		case TRC_PHASE:
			v = phase(coeff);
			break;
		case TRC_DELAY:
			v = groupdelay_from_array(i, array);
			break;
		case TRC_LINEAR:
			v = -linear(coeff);
			break;
		case TRC_SWR:
			v = -(1.0f - swr(coeff));
			break;
		case TRC_REAL:
			v = coeff[0];
			break;
		case TRC_IMAG:
			v = coeff[1];
			break;
		case TRC_R:
			impedance(coeff, &v, NULL, z0);
			break;
		case TRC_X:
			impedance(coeff, NULL, &v, z0);
			break;
		case TRC_Q:
			v = qualityfactor(coeff);
			break;
		case TRC_SMITH:
		//case TRC_ADMIT:
		case TRC_POLAR:
			cartesian_scale(coeff[0], coeff[1], &x, &y, scale);
			break;
	}

	if (trace[t].type != TRC_SMITH && trace[t].type != TRC_POLAR)	// && trace[t].type != TRC_ADMI)
	{	// non-smith chart
		v *= scale;
		v = refpos - v;

		x = ((i * (WIDTH)) + (sweep_points - 1) / 2) / (sweep_points - 1) + CELLOFFSETX;
		y = float2int(v * GRIDY);
	}

	if (x < CELLOFFSETX) x = CELLOFFSETX;
	else
	if (x > (CELLOFFSETX + WIDTH - 1)) x = CELLOFFSETX + WIDTH - 1;

	if (y < 0) y = 0;
	else
	if (y > (GRIDY * NGRIDY)) y = GRIDY * NGRIDY;

	return INDEX(x, y);
}

static void format_smith_value(char *buf, int len, const float coeff[2], uint32_t frequency)
{
	// z = (gamma + 1) / (gamma - 1) * z0
	const float z0 = 50.0f;		// 50R impedance
	float zr;
	float zi;
	float v;
	char prefix;
	float value;

	switch (marker_smith_format)
	{
		case MS_LIN:
			plot_printf(buf, len, "%.2f %.1f"S_DEGREE, linear(coeff), phase(coeff));
			break;

		case MS_LOG:
			v = logmag(coeff);
			if (v == -INFINITY)
				plot_printf(buf, len, "-"S_INFINITY" dB");
			else
				plot_printf(buf, len, "%.1fdB %.1f"S_DEGREE, v, phase(coeff));
			break;

		case MS_REIM:
			//plot_printf(buf, len, "%F %c j%F", coeff[0], (coeff[1] >= 0) ? '+' : '-', fabsf(coeff[1]));
			plot_printf(buf, len, "%.3f %c j%.3f", coeff[0], (coeff[1] >= 0) ? '+' : '-', fabsf(coeff[1]));
			break;

		case MS_RX:
			impedance(coeff, &zr, &zi, z0);
			//plot_printf(buf, len, "%F %cj%F"S_OHM, zr, (zi >= 0) ? '+' : '-', fabsf(zi));
			plot_printf(buf, len, "%.2f %c j%.2f"S_OHM, zr, (zi >= 0) ? '+' : '-', fabsf(zi));
			break;

		case MS_RLC:
			impedance(coeff, &zr, &zi, z0);
			if (zi < 0)
			{	// capacitance
				prefix = 'F';
				value = -1.0f / ((2 * VNA_PI) * frequency * zi);
			}
			else
			{	// inductance
				prefix = 'H';
				value = zi / ((2 * VNA_PI) * frequency);
			}
			plot_printf(buf, len, "%F"S_OHM" %F%c", zr, value, prefix);
			break;
	}
}

static void trace_get_value_string(int t, char *buf, int len, float array[POINTS_COUNT][2], int i)
{
	const float z0 = 50.0f;
	float *coeff = array[i];
	float v;
	char *format;

	switch (trace[t].type)
	{
		case TRC_LOGMAG:
			format = "%.2fdB";
			v = logmag(coeff);
			break;
		case TRC_PHASE:
			format = "%.1f"S_DEGREE;
			v = phase(coeff);
			break;
		case TRC_DELAY:
			format = "%.2Fs";
			v = groupdelay_from_array(i, array);
			break;
		case TRC_LINEAR:
			format = "%.4f";
			v = linear(coeff);
			break;
		case TRC_SWR:
			format = "%.4f";
			v = swr(coeff);
			break;
		case TRC_REAL:
			format = "%.4f";
			v = coeff[0];
			break;
		case TRC_IMAG:
			format = "%.4fj";
			v = coeff[1];
			break;
		case TRC_R:
			format = "%.2F"S_OHM;
			impedance(coeff, &v, NULL, z0);
			break;
		case TRC_X:
			format = "%.2F"S_OHM;
			impedance(coeff, NULL, &v, z0);
			break;
		case TRC_Q:
			format = "%.1f";
			v = qualityfactor(coeff);
			break;
		case TRC_SMITH:
			format_smith_value(buf, len, coeff, frequencies[i]);
			return;
		//case TRC_ADMIT:
		case TRC_POLAR:
			plot_printf(buf, len, "%.2f%+.2fj", coeff[0], coeff[1]);
		default:
			return;
	}

	plot_printf(buf, len, format, v);
}

static void trace_get_value_string_delta(int t, char *buf, int len, float array[POINTS_COUNT][2], int index, int index_ref)
{
	const float z0 = 50.0f;
	const float *coeff = array[index];
	const float *coeff_ref = array[index_ref];
	float v;
	char *format;
	float zr;
	float zi;

	switch (trace[t].type)
	{
		case TRC_LOGMAG:
			format = S_DELTA"%.2fdB";
			v = logmag(coeff) - logmag(coeff_ref);
			break;
		case TRC_PHASE:
			format = S_DELTA"%.2f"S_DEGREE;
			v = phase(coeff) - phase(coeff_ref);
			break;
		case TRC_DELAY:
			format = "%.2Fs";
			v = groupdelay_from_array(index, array) - groupdelay_from_array(index_ref, array);
			break;
		case TRC_LINEAR:
			format = S_DELTA"%.3f";
			v = linear(coeff) - linear(coeff_ref);
			break;
		case TRC_SWR:
			format = S_DELTA"%.3f";
			v = swr(coeff);
			if (v != INFINITY)
				v -= swr(coeff_ref);
			break;
		case TRC_SMITH:
			format_smith_value(buf, len, coeff, frequencies[index]);
			return;
		case TRC_REAL:
			format = S_DELTA"%.3f";
			v = coeff[0] - coeff_ref[0];
			break;
		case TRC_IMAG:
			format = S_DELTA"%.3fj";
			v = coeff[1] - coeff_ref[1];
			break;
		case TRC_R:
			format = "%.2F"S_OHM;
			impedance(coeff, &zr, NULL, z0);
			v = zr;
			break;
		case TRC_X:
			format = "%.2F"S_OHM;
			impedance(coeff, NULL, &zi, z0);
			v = zi;
			break;
		case TRC_Q:
			format = "%.1f";
			v = qualityfactor(coeff);
			break;
		//case TRC_ADMIT:
		case TRC_POLAR:
			plot_printf(buf, len, "%.2f%+.2fj", coeff[0], coeff[1]);
		default:
			return;
	}

	plot_printf(buf, len, format, v);
}

static int trace_get_info(int t, char *buf, int len)
{
	const char *name  = get_trace_typename(t);
	const float scale = get_trace_scale(t);

	switch (trace[t].type)
	{
		case TRC_LOGMAG:
			return plot_printf(buf, len, "%s %ddB/", name, (int)scale);
		case TRC_PHASE:
			return plot_printf(buf, len, "%s %d" S_DEGREE "/", name, (int)scale);
		case TRC_SMITH:
		//case TRC_ADMIT:
		case TRC_POLAR:
			return (scale != 1.0f) ? plot_printf(buf, len, "%s %.1fFS", name, scale) : plot_printf(buf, len, "%s ", name);
		default:
			return plot_printf(buf, len, "%s %F/", name, scale);
	}

	return 0;
}

static float time_of_index(int idx)
{
   return 1.0f / (float)(frequencies[1] - frequencies[0]) / (float)FFT_SIZE * idx;
}

static float distance_of_index(int idx)
{
//	return ((float)idx * velocity_factor * SPEED_OF_LIGHT) / ((frequencies[1] - frequencies[0]) * (FFT_SIZE * 2));
	return ((float)idx * velocity_factor * ((float)SPEED_OF_LIGHT / (FFT_SIZE * 2))) / (frequencies[1] - frequencies[0]);
}

static inline void mark_map(int x, int y)
{
   if (y >= 0 && y < MAX_MARKMAP_Y && x >= 0 && x < MAX_MARKMAP_X)
      markmap[current_mappage][y] |= 1 << x;
}

static inline void swap_markmap(void)
{
   current_mappage ^= 1;
}

static void clear_markmap(void)
{
   memset(markmap[current_mappage], 0, sizeof markmap[current_mappage]);
}

void force_set_markmap(void)
{
   memset(markmap[current_mappage], 0xff, sizeof markmap[current_mappage]);
}

void invalidate_rect(int x0, int y0, int x1, int y1)
{
	int x;
	int y;
	x0 /= CELLWIDTH;
	x1 /= CELLWIDTH;
	y0 /= CELLHEIGHT;
	y1 /= CELLHEIGHT;
	for (y = y0; y <= y1; y++)
		for (x = x0; x <= x1; x++)
			mark_map(x, y);
}

#define SWAP(x, y) {const int t = x; x = y; y = t;}

static void mark_cells_from_index(void)
{
	int t;

	// mark cells between each neighbour points

	map_t *map = &markmap[current_mappage][0];

	for (t = 0; t < TRACES_MAX; t++)
	{
		int i;

		if (!trace[t].enabled)
			continue;

		index_t *index = &trace_index[t][0];

		int m0 = CELL_X(index[0]) / CELLWIDTH;
		int n0 = CELL_Y(index[0]) / CELLHEIGHT;

		map[n0] |= 1 << m0;

		for (i = 1; i < sweep_points; i++)
		{
			int j;

			int m1 = CELL_X(index[i]) / CELLWIDTH;
			int n1 = CELL_Y(index[i]) / CELLHEIGHT;

			if (m0 == m1 && n0 == n1)
				continue;

			int x0 = m0;
			int x1 = m1;
			if (x0 > x1)
				SWAP(x0, x1);
			m0 = m1;

			int y0 = n0;
			int y1 = n1;
			if (y0 > y1)
				SWAP(y0, y1);
			n0 = n1;

			for (; y0 <= y1; y0++)
				for (j = x0; j <= x1; j++)
					map[y0] |= 1 << j;
		}
	}
}

static inline void
markmap_upperarea(void)
{
  // Hardcoded, Text info from upper area
  invalidate_rect(0, 0, AREA_WIDTH_NORMAL, 3*FONT_STR_HEIGHT);
}

//
// in most cases _compute_outcode clip calculation not give render line speedup
//
static inline void
cell_drawline(int x0, int y0, int x1, int y1, int c)
{
  if (x0 < 0 && x1 < 0) return;
  if (y0 < 0 && y1 < 0) return;
  if (x0 >= CELLWIDTH && x1 >= CELLWIDTH) return;
  if (y0 >= CELLHEIGHT && y1 >= CELLHEIGHT) return;

  // modifed Bresenham's line algorithm, see https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  if (x1 < x0) { SWAP(x0, x1); SWAP(y0, y1); }
  int dx = x1 - x0;
  int dy = y1 - y0, sy = 1; if (dy < 0) { dy = -dy; sy = -1; }
  int err = (dx > dy ? dx : -dy) / 2;

  while (1) {
    if (y0 >= 0 && y0 < CELLHEIGHT && x0 >= 0 && x0 < CELLWIDTH)
      cell_buffer[y0 * CELLWIDTH + x0] |= c;
    if (x0 == x1 && y0 == y1)
      return;
    int e2 = err;
    if (e2 > -dx) { err -= dy; x0++;  }
    if (e2 <  dy) { err += dx; y0+=sy;}
  }
}

// Give a little speedup then draw rectangular plot (50 systick on all calls, all render req 700 systick)
// Write more difficult algoritm for seach indexes not give speedup
static int
search_index_range_x(int x1, int x2, index_t index[POINTS_COUNT], int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = sweep_points;
  int idx_x;

  // Search index point in cell
  while (1) {
    i = (head + tail) / 2;
    idx_x = CELL_X(index[i]);
    if (idx_x >= x2) { // index after cell
      if (tail == i)
        return false;
      tail = i;
    }
    else if (idx_x < x1) {    // index before cell
      if (head == i)
        return false;
      head = i;
    }
    else  // index in cell (x =< idx_x < cell_end)
      break;
  }
  j = i;
  // Search index left from point
  do {
    j--;
  } while (j > 0 && x1 <= CELL_X(index[j]));
  *i0 = j;
  // Search index right from point
  do {
    i++;
  } while (i < sweep_points-1 && CELL_X(index[i]) < x2);
  *i1 = i;

  return TRUE;
}

#define REFERENCE_WIDTH    6
#define REFERENCE_HEIGHT   5
#define REFERENCE_X_OFFSET 5
#define REFERENCE_Y_OFFSET 2

// Reference bitmap
static const uint8_t reference_bitmap[]={
  0b11000000,
  0b11110000,
  0b11111100,
  0b11110000,
  0b11000000,
};

static void
draw_refpos(int x, int y, int c)
{
  int y0 = y, j;
  for (j = 0; j < REFERENCE_HEIGHT; j++, y0++) {
    if (y0 < 0 || y0 >= CELLHEIGHT) continue;
    int x0 = x;
    uint8_t bits = reference_bitmap[j];
    while (bits) {
      if (x0 >= 0 && x0 < CELLWIDTH)
        cell_buffer[y0 * CELLWIDTH + x0] = (bits & 0x80) ? c : DEFAULT_BG_COLOR;
      x0++;
      bits <<= 1;
    }
  }
}

#define MARKER_WIDTH  7
#define MARKER_HEIGHT 10
#define X_MARKER_OFFSET 3
#define Y_MARKER_OFFSET 10
static const uint8_t marker_bitmap[] =
{
  // Marker 1
  0b11111110,
  0b11101110,
  0b11001110,
  0b11101110,
  0b11101110,
  0b11101110,
  0b11000110,
  0b01111100,
  0b00111000,
  0b00010000,
  // Marker 2
  0b11111110,
  0b11000110,
  0b10111010,
  0b11111010,
  0b11000110,
  0b10111110,
  0b10000010,
  0b01111100,
  0b00111000,
  0b00010000,
  // Marker 3
  0b11111110,
  0b11000110,
  0b10111010,
  0b11100110,
  0b11111010,
  0b10111010,
  0b11000110,
  0b01111100,
  0b00111000,
  0b00010000,
  // Marker 4
  0b11111110,
  0b11110110,
  0b11100110,
  0b11010110,
  0b10110110,
  0b10110110,
  0b10000010,
  0b01110100,
  0b00111000,
  0b00010000,
};

static void draw_marker(int x, int y, int c, int ch)
{
	int j;
  int y0 = y;
  for (j = 0; j < MARKER_HEIGHT; j++, y0++)
  {
    int x0 = x;
    uint8_t bits = marker_bitmap[ch * MARKER_HEIGHT + j];
    bool force_color = false;
    while (bits)
    {
      if (bits & 0x80) force_color = true;
      if (x0 >= 0 && x0 < CELLWIDTH && y0 >= 0 && y0 < CELLHEIGHT)
      {
        if (bits & 0x80)
          cell_buffer[y0 * CELLWIDTH + x0] = c;
        else
        if (force_color)
          cell_buffer[y0 * CELLWIDTH + x0] = DEFAULT_BG_COLOR;
      }
      x0++;
      bits <<= 1;
    }
  }
}

static void markmap_marker(int marker)
{
	int t;

	if (marker < 0 || marker >= MARKERS_MAX)
		return;

	const int m_index = marker_index(marker);

	if (!marker_enabled(marker))
		return;

	for (t = 0; t < TRACES_MAX; t++)
	{
		if (!trace[t].enabled)
			continue;

		const index_t index = trace_index[t][m_index];
		const int x = CELL_X(index) - X_MARKER_OFFSET;
		const int y = CELL_Y(index) - Y_MARKER_OFFSET;

		invalidate_rect(x, y, x + MARKER_WIDTH - 1, y + MARKER_HEIGHT - 1);
	}
}

static void markmap_all_markers(void)
{
	int i;

  for (i = 0; i < MARKERS_MAX; i++)
  {
    if (!marker_enabled(i))
      continue;

    markmap_marker(i);
  }
  markmap_upperarea();
}

void marker_position(int m, int t, int *x, int *y)
{
	if (m >= 0 && m < MARKERS_MAX)
	{
		const index_t index = trace_index[t][marker_index(m)];
		*x = CELL_X(index);
		*y = CELL_Y(index);
	}
}

static int greater(int x, int y) { return x > y; }
static int lesser(int x, int y)  { return x < y; }

static int (*compare)(int x, int y) = lesser;

int marker_search(void)
{
	int i;

	int found = 0;
	const int current_trace = config.current_trace;

	if (current_trace < 0 || current_trace >= TRACES_MAX)
		return -1;

	int value = CELL_Y(trace_index[current_trace][0]);

	for (i = 0; i < sweep_points; i++)
	{
		const index_t index = trace_index[current_trace][i];
		if ((*compare)(value, CELL_Y(index)))
		{
			value = CELL_Y(index);
			found = i;
		}
	}

	return found;
}

int get_marker_search(void)
{
	return (compare == greater) ? 0 : 1;
}

void set_marker_search(int mode)
{
	compare = (mode == 0) ? greater : lesser;
}

int marker_search_left(int from)
{
  int found = -1;
  const int current_trace = config.current_trace;

  if (current_trace < 0 || current_trace >= TRACES_MAX)
    return -1;

  int value = CELL_Y(trace_index[current_trace][from]);

  int i;

  for (i = from - 1; i >= 0; i--)
  {
    const index_t index = trace_index[current_trace][i];
    if ((*compare)(value, CELL_Y(index)))
      break;
    value = CELL_Y(index);
  }

  for (; i >= 0; i--)
  {
    const index_t index = trace_index[current_trace][i];
    if ((*compare)(CELL_Y(index), value))
      break;
    found = i;
    value = CELL_Y(index);
  }

  return found;
}

int marker_search_right(int from)
{
  int found = -1;
  const int current_trace = config.current_trace;

  if (current_trace < 0 || current_trace >= TRACES_MAX)
    return -1;

  int value = CELL_Y(trace_index[current_trace][from]);

  int i;

  for (i = from + 1; i < sweep_points; i++)
  {
    const index_t index = trace_index[current_trace][i];
    if ((*compare)(value, CELL_Y(index)))
      break;
    value = CELL_Y(index);
  }

  for (; i < sweep_points; i++)
  {
    const index_t index = trace_index[current_trace][i];
    if ((*compare)(CELL_Y(index), value))
      break;
    found = i;
    value = CELL_Y(index);
  }

  return found;
}

int search_nearest_index(int x, int y, int t)
{
	int i;

	index_t *index = trace_index[t];

	int min_i = -1;
	int min_d = 1000;

	for (i = 0; i < sweep_points; i++)
	{
		int16_t dx = x - CELL_X(index[i]);
		int16_t dy = y - CELL_Y(index[i]);
		if (dx < 0) dx = -dx;
		if (dy < 0) dy = -dy;
		if (dx > 20 || dy > 20)
			continue;
		int d = dx*dx + dy*dy;
		if (min_d > d || i == 0)
		{
			min_d = d;
			min_i = i;
		}
	}

	return min_i;
}

void plot_into_index(float measured[2][POINTS_COUNT][2])
{
	int t;

	for (t = 0; t < TRACES_MAX; t++)
	{
		int i;

		if (!trace[t].enabled)
			continue;

		const int ch   = trace[t].channel;
		index_t *index = trace_index[t];

		for (i = 0; i < sweep_points; i++)
			index[i] = trace_into_index(t, i, measured[ch]);
	}

	#if 0
		for (t = 0; t < TRACES_MAX; t++)
			if (trace[t].enabled && trace[t].polar)
				quicksort(trace_index[t], 0, sweep_points);
	#endif

	mark_cells_from_index();
	markmap_all_markers();
}

static void draw_cell(int m, int n)
{
	int x0 = m * CELLWIDTH;
	int y0 = n * CELLHEIGHT;
	int w = CELLWIDTH;
	int h = CELLHEIGHT;
	int x;
	int y;
	int i0;
	int i1;
	int i;
	int t;
	uint16_t c;

	// Clip cell by area
	if ((x0 + w) > area_width)
		w = area_width - x0;
	if ((y0 + h) > area_height)
		h = area_height - y0;
	if (w <= 0 || h <= 0)
		return;

//	PULSE;

	// Clear buffer ("0 : height" lines)
	#if 0
		// use memset 350 system ticks for all screen calls
		// as understand it use 8 bit set, slow down on 32 bit systems
		memset(spi_buffer, DEFAULT_BG_COLOR, (h*CELLWIDTH)*sizeof(uint16_t));
	#else
		// use direct set  35 system ticks for all screen calls
		#if (CELLWIDTH % 8) != 0
			#error "CELLWIDTH % 8 should be == 0 for speed, or need rewrite cell cleanup"
		#endif

		// Set DEFAULT_BG_COLOR for 8 pixels in one cycle
		int count = h * CELLWIDTH / (16 / sizeof(pixel_t));
		uint32_t *p = (uint32_t *)cell_buffer;
		while (count--)
		{
			p[0] = DEFAULT_BG_COLOR | (DEFAULT_BG_COLOR << 16);
			p[1] = DEFAULT_BG_COLOR | (DEFAULT_BG_COLOR << 16);
			p[2] = DEFAULT_BG_COLOR | (DEFAULT_BG_COLOR << 16);
			p[3] = DEFAULT_BG_COLOR | (DEFAULT_BG_COLOR << 16);
			p += 4;
		}
	#endif

	// Draw grid
	#if 1
		c = config.grid_color;

		// Generate grid type list
		uint32_t trace_type = 0;
		for (t = 0; t < TRACES_MAX; t++)
		{
			if (trace[t].enabled)
				trace_type |= (1 << trace[t].type);
		}

		// Draw rectangular plot (40 system ticks for all screen calls)
		if (trace_type & RECTANGULAR_GRID_MASK)
		{
			for (x = 0; x < w; x++)
			{
				if (rectangular_grid_x(x + x0))
				{
					for (y = 0; y < h; y++)
						if (!(y & 3))
							cell_buffer[(y * CELLWIDTH) + x] = c;
				}
			}

			for (y = 0; y < h; y++)
			{
				if (rectangular_grid_y(y + y0))
				{
					for (x = 0; x < w; x++)
					{
						if (!(x & 3) && (x + x0) >= CELLOFFSETX && (x + x0) <= (WIDTH + CELLOFFSETX))
							cell_buffer[(y * CELLWIDTH) + x] = c;
					}
				}
			}
		}

		// Smith greed line (1000 system ticks for all screen calls)
		if (trace_type & (1 << TRC_SMITH))
		{
			for (y = 0; y < h; y++)
				for (x = 0; x < w; x++)
					if (smith_grid(x + x0, y + y0))
						cell_buffer[y * CELLWIDTH + x] = c;
		}
		// Polar greed line (800 system ticks for all screen calls)
		else
		if (trace_type & (1 << TRC_POLAR))
		{
			for (y = 0; y < h; y++)
				for (x = 0; x < w; x++)
					if (polar_grid(x + x0, y + y0))
						cell_buffer[y * CELLWIDTH + x] = c;
		}

		#if 0
			else
			if (trace_type & (1 << TRC_ADMIT))
			{
				for (y = 0; y < h; y++)
					for (x = 0; x < w; x++)
						if (smith_grid3(x+x0, y+y0)
							// smith_grid2(x+x0, y+y0, 0.5))
							cell_buffer[y * CELLWIDTH + x] = c;
			}
		#endif
	#endif

   //  PULSE;
   // Draw traces (50-600 system ticks for all screen calls, depend from lines count and size)
	#if 1
		for (t = 0; t < TRACES_MAX; t++)
		{
			if (!trace[t].enabled)
				continue;

			c = config.trace_color[t];
			// draw polar plot (check all points)
			i0 = 0;
			i1 = 0;
			uint32_t trace_type = (1 << trace[t].type);
			if (trace_type & ((1 << TRC_SMITH) | (1 << TRC_POLAR)))
				i1 = sweep_points - 1;
			else  // draw rectangular plot (search index range in cell, save 50-70 system ticks for all screen calls)
				search_index_range_x(x0, x0 + w, trace_index[t], &i0, &i1);

			index_t *index = trace_index[t];

			for (i = i0; i < i1; i++)
			{
				int x1 = CELL_X(index[i]) - x0;
				int y1 = CELL_Y(index[i]) - y0;
				int x2 = CELL_X(index[i + 1]) - x0;
				int y2 = CELL_Y(index[i + 1]) - y0;
				cell_drawline(x1, y1, x2, y2, c);
			}
		}
	#else
		for (x = 0; x < area_width; x += 6)
			cell_drawline(x - x0, 0 - y0, area_width - x - x0, area_height - y0, config.trace_color[0]);
	#endif

	//  PULSE;
   // draw marker symbols on each trace (<10 system ticks for all screen calls)
	#if 1
    	 for (i = 0; i < MARKERS_MAX; i++)
    	 {
    		 if (!marker_enabled(i))
    			 continue;

    		 for (t = 0; t < TRACES_MAX; t++)
    		 {
    			 if (!trace[t].enabled)
    				 continue;

    			 const index_t index = trace_index[t][marker_index(i)];
    			 const int x = CELL_X(index) - x0 - X_MARKER_OFFSET;
    			 const int y = CELL_Y(index) - y0 - Y_MARKER_OFFSET;

    			 // Check marker icon on cell
    			 if (x + MARKER_WIDTH >= 0 && x - MARKER_WIDTH < CELLWIDTH && y + MARKER_HEIGHT >= 0 && y - MARKER_HEIGHT < CELLHEIGHT)
    				 draw_marker(x, y, config.trace_color[t], i);
    		 }
    	 }
	#endif

    // Draw trace and marker info on the top (50 system ticks for all screen calls)
	#if 1
    	 if (n <= (3*FONT_STR_HEIGHT)/CELLHEIGHT)
    		 cell_draw_marker_info(x0, y0);
	#endif

   //  PULSE;
   // Draw reference position (<10 system ticks for all screen calls)
   for (t = 0; t < TRACES_MAX; t++)
   {
   	if (!trace[t].enabled)
   		continue;

   	uint32_t trace_type = (1 << trace[t].type);
   	if (trace_type & ((1 << TRC_SMITH) | (1 << TRC_POLAR)))
   		continue;

   	int x = 0 - x0 + CELLOFFSETX - REFERENCE_X_OFFSET;

   	if (x + REFERENCE_WIDTH >= 0 && x - REFERENCE_WIDTH < CELLWIDTH)
   	{
   		int y = HEIGHT - float2int((get_trace_refpos(t) * GRIDY)) - y0 - REFERENCE_Y_OFFSET;
   		if (y + REFERENCE_HEIGHT >= 0 && y - REFERENCE_HEIGHT < CELLHEIGHT)
   			draw_refpos(x, y, config.trace_color[t]);
   	}
   }

   #ifdef USE_LC_MATCHING
   	if (config.flags & CONFIG_FLAGS_LC_MATCH)
   		cell_draw_lc_match(&lc_match_array, x0, y0, CELLHEIGHT);
   #endif

   // Need right clip cell render (25 system ticks for all screen calls)
	#if 1
   	if (w < CELLWIDTH)
   	{
   		pixel_t *src = cell_buffer + CELLWIDTH;
   		pixel_t *dst = cell_buffer + w;
   		for (y = h; --y; src += CELLWIDTH - w)
   			for (x = w; x--;)
   				*dst++ = *src++;
   	}
	#endif

   // Draw cell (500 system ticks for all screen calls)
   ili9341_bulk(OFFSETX + x0, OFFSETY + y0, w, h);
//   ili9341_bulk(OFFSETX + x0, y0, w, h);
}

static void draw_all_cells(bool flush_markmap)
{
	//  START_PROFILE
	int m;
	for (m = 0; m < (area_width + CELLWIDTH - 1) / CELLWIDTH; m++)
	{
		int n;
		for (n = 0; n < (area_height + CELLHEIGHT-1) / CELLHEIGHT; n++)
		{
			if ((markmap[0][n] | markmap[1][n]) & (1u << m))
			{
				draw_cell(m, n);
				//ili9341_fill(m * CELLWIDTH + OFFSETX, n * CELLHEIGHT, 2, 2, RGB565(255, 0, 0));
			}
			//else
			//	ili9341_fill(m * CELLWIDTH + OFFSETX, n * CELLHEIGHT, 2, 2, RGB565(0, 255, 0));
		}
	}

	//  STOP_PROFILE
	if (flush_markmap)
	{
		// keep current map for update
		swap_markmap();
		// clear map for next plotting
		clear_markmap();
	}
}

void draw_all(bool flush)
{
	if (redraw_request & REDRAW_AREA)
		force_set_markmap();

	if (redraw_request & REDRAW_MARKER)
		markmap_upperarea();

	if (redraw_request & (REDRAW_CELLS | REDRAW_MARKER | REDRAW_AREA))
		draw_all_cells(flush);

	if (redraw_request & REDRAW_FREQUENCY)
		draw_frequencies();

	if (redraw_request & REDRAW_CAL_STATUS)
		draw_cal_status();

	if (redraw_request & REDRAW_BATTERY)
		draw_battery_status();

	redraw_request = 0;
}

// Call this function then need fast draw marker and marker info Used in ui.c for leveler move marker, drag marker and etc.
void redraw_marker(int marker)
{
	if (marker < 0 || marker >= MARKERS_MAX)
		return;

	// mark map on new position of marker
	markmap_marker(marker);

	// mark cells on marker info
	markmap_upperarea();

	draw_all_cells(TRUE);

	// Force redraw all area after (disable artifacts after fast marker update area)
	redraw_request |= REDRAW_AREA;
}

void request_to_draw_cells_behind_menu(void)
{  // Values Hardcoded from ui.c
   invalidate_rect(LCD_WIDTH - MENU_BUTTON_WIDTH - OFFSETX, 0, LCD_WIDTH - OFFSETX, LCD_HEIGHT - 1);
   redraw_request |= REDRAW_CELLS;
}

void request_to_draw_cells_behind_numeric_input(void)
{  // Values Hardcoded from ui.c
   invalidate_rect(0, LCD_HEIGHT - NUM_INPUT_HEIGHT, LCD_WIDTH - 1, LCD_HEIGHT - 1);
   redraw_request |= REDRAW_CELLS;
}

static int cell_drawchar(uint8_t ch, int x, int y, bool fixed_pitch)
{
	int c;

	const uint8_t *char_buf = FONT_GET_DATA(ch);
	const int ch_size       = fixed_pitch ? FONT_WIDTH : FONT_GET_WIDTH(ch);

	//if (y <= -FONT_GET_HEIGHT || y >= CELLHEIGHT || x <= -ch_size || x >= CELLWIDTH)
	//	return ch_size;

	if (x <= -ch_size)
		return ch_size;

	for (c = 0; c < FONT_GET_HEIGHT; c++)
	{
		int r;
		uint8_t bits = *char_buf++;
		if ((y + c) < 0 || (y + c) >= CELLHEIGHT)
			continue;
		for (r = 0; r < ch_size; r++)
		{
			if ((x + r) >= 0 && (x + r) < CELLWIDTH && (0x80 & bits))
				cell_buffer[(y + c) * CELLWIDTH + (x + r)] = foreground_color;
			bits <<= 1;
		}
	}

	return ch_size;
}

void cell_drawstring(char *str, int x, int y, bool fixed_pitch)
{
	if (y <= -FONT_GET_HEIGHT || y >= CELLHEIGHT)
		return;

	while (*str && x < CELLWIDTH)
		x += cell_drawchar(*str++, x, y, fixed_pitch);
}

static void cell_draw_marker_info(int x0, int y0)
{
	int i;
	char buf[24];

	const int am = active_marker;
//	if (am < 0 || am >= MARKERS_MAX)
//		return;

	const int mi = (am < 0 || am >= MARKERS_MAX) ? -1 : marker_index(am);

	const int current_trace = config.current_trace;

	// count how many markers are enabled
	int first_marker   = -1;
	int num_of_markers = 0;
	for (i = 0; i < MARKERS_MAX; i++)
	{
		if (marker_enabled(i))
		{
			num_of_markers++;
			if (first_marker < 0)
				first_marker = i;
		}
	}

	if (num_of_markers > 1 && previous_marker < 0)
		previous_marker = first_marker;	// stay in marker delta mode

	int j = 0;

	#if 1
		if (num_of_markers > 1 && previous_marker >= 0 && current_trace >= 0 && current_trace < TRACES_MAX)
		{	// this section shows the delta between markers

			int mk;

			for (mk = 0; mk < MARKERS_MAX; mk++)
			{
				if (!marker_enabled(mk))
					continue;

				int xpos = 1 + (j % 2) * (WIDTH / 2) + CELLOFFSETX - x0;
				int ypos = 1 + (j / 2) * (FONT_STR_HEIGHT) - y0;

				ili9341_set_foreground(config.trace_color[current_trace]);

				if (mk == am)
					cell_drawstring(S_SARROW, xpos, ypos, false);

				xpos += 5;

				plot_printf(buf, sizeof buf, "M%d", mk + 1);
				cell_drawstring(buf, xpos, ypos, false);

				xpos += 13;

				//trace_get_info(t, buf, sizeof buf);

				const uint32_t freq = frequencies[marker_index(mk)];

				if (uistat.marker_delta && mk != am)
				{
					const uint32_t freq1 = frequencies[mi];
					const uint32_t delta = freq > freq1 ? freq - freq1 : freq1 - freq;
					plot_printf(buf, sizeof buf, S_DELTA"%.9qHz", delta);
				}
				else
				{
					plot_printf(buf, sizeof buf, "%.10qHz", freq);
				}

				cell_drawstring(buf, xpos, ypos, false);

				xpos += 67;

				const int mki = marker_index(mk);

				if (uistat.marker_delta && mk != am)
					trace_get_value_string_delta(current_trace, buf, sizeof buf, measured[trace[current_trace].channel], mki, mi);
				else
					trace_get_value_string(current_trace, buf, sizeof buf, measured[trace[current_trace].channel], mki);

				ili9341_set_foreground(DEFAULT_FG_COLOR);
				cell_drawstring(buf, xpos, ypos, false);

				j++;
			}

			// draw marker delta
			if (!uistat.marker_delta && previous_marker >= 0 && am != previous_marker && marker_enabled(previous_marker))
			{
				const int mip = marker_index(previous_marker);

				const int ypos = 1 + (j / 2) * (FONT_STR_HEIGHT) - y0;
				int xpos = (WIDTH / 2 + 30) + CELLOFFSETX - x0;

				plot_printf(buf, sizeof buf, S_DELTA"%d-%d:", am + 1, previous_marker + 1);

				ili9341_set_foreground(DEFAULT_FG_COLOR);
				cell_drawstring(buf, xpos, ypos, false);

				xpos += 27;

				if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ)
				{
					const uint32_t freq  = frequencies[mi];
					const uint32_t freq1 = frequencies[mip];
					const uint32_t delta = freq > freq1 ? freq - freq1 : freq1 - freq;
					plot_printf(buf, sizeof buf, "%c%.13qHz", freq >= freq1 ? '+' : '-', delta);
				}
				else
				{
					plot_printf(buf, sizeof buf, "%Fs (%Fm)", time_of_index(mi) - time_of_index(mip), distance_of_index(mi) - distance_of_index(mip));
				}

				cell_drawstring(buf, xpos, ypos, false);
			}
		}
		else
	#endif
		{
			int t;

			for (t = 0; t < TRACES_MAX; t++)
			{
				if (!trace[t].enabled)
					continue;

				const int ch = trace[t].channel;

				const int ypos = 1 + (j / 2) * (FONT_STR_HEIGHT)         - y0;
				int xpos       = 1 + (j % 2) * (WIDTH / 2) + CELLOFFSETX - x0;

				ili9341_set_foreground(config.trace_color[t]);

				if (t == current_trace)
					cell_drawstring(S_SARROW, xpos, ypos, false);

				xpos += 5;

				const char *ch_name_str[] = {"S11", "S21"};
				//plot_printf(buf, sizeof buf, "CH%d", ch_name_str[ch], ch);
				plot_printf(buf, sizeof(buf), "%s", ch_name_str[ch]);
				cell_drawstring(buf, xpos, ypos, false);

				xpos += 19;

				const int n = trace_get_info(t, buf, sizeof buf);
				cell_drawstring(buf, xpos, ypos, false);

				xpos += n * 5 + 2;
				//xpos += 60;

				if (mi >= 0)	// cathy
				{
					trace_get_value_string(t, buf, sizeof buf, measured[ch], mi);
					ili9341_set_foreground(DEFAULT_FG_COLOR);
					cell_drawstring(buf, xpos, ypos, false);
				}

				j++;
			}

			if (mi >= 0)
			{
				// draw marker frequency
				const int ypos = 1 + (j / 2) * (FONT_STR_HEIGHT) - y0;
				int xpos       = (WIDTH / 2 + 40) + CELLOFFSETX  - x0;

				ili9341_set_foreground(DEFAULT_FG_COLOR);

				if (uistat.lever_mode == LM_MARKER)
					cell_drawstring(S_SARROW, xpos, ypos, false);

				xpos += 5;

				plot_printf(buf, sizeof buf, "M%d:", am + 1);
				cell_drawstring(buf, xpos, ypos, false);

				xpos += 19;

				if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ)
					plot_printf(buf, sizeof buf, "%qHz", frequencies[mi]);
				else
					plot_printf(buf, sizeof buf, "%Fs (%Fm)", time_of_index(mi), distance_of_index(mi));
				cell_drawstring(buf, xpos, ypos, false);
			}
		}

		ili9341_set_foreground(DEFAULT_FG_COLOR);

		if (electrical_delay != 0)
		{	// draw electrical delay
			const int ypos = 1  + ((j + 1) / 2) * (FONT_STR_HEIGHT) - y0;
			int xpos       = 21 + CELLOFFSETX - x0;

			if (uistat.lever_mode == LM_EDELAY)
				cell_drawstring(S_SARROW, xpos, ypos, false);

			xpos += 5;

			const float light_speed_ps = SPEED_OF_LIGHT * 1e-12f; // (m / ps)

			plot_printf(buf, sizeof(buf), "Edelay %Fs %Fm", electrical_delay * 1e-12f, electrical_delay * light_speed_ps * velocity_factor);
			cell_drawstring(buf, xpos, ypos, false);
		}
}

void draw_frequencies(void)
{
	char buf1[32];
	char buf2[32];
	buf2[0] = 0;

	if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ)
	{
		if (FREQ_IS_CW())
		{
			plot_printf(buf1, sizeof(buf1), " CW %qHz", get_sweep_frequency(ST_CW));
		}
		else
		if (FREQ_IS_STARTSTOP())
		{
			plot_printf(buf1, sizeof(buf1), " START %qHz", get_sweep_frequency(ST_START));
			plot_printf(buf2, sizeof(buf2), " STOP %qHz",  get_sweep_frequency(ST_STOP));
		}
		else
		if (FREQ_IS_CENTERSPAN())
		{
			plot_printf(buf1, sizeof(buf1), " CENTER %qHz", get_sweep_frequency(ST_CENTER));
			plot_printf(buf2, sizeof(buf2), " SPAN %qHz",   get_sweep_frequency(ST_SPAN));
		}
	}
	else
	{
		plot_printf(buf1, sizeof(buf1), " START 0s");
		plot_printf(buf2, sizeof(buf2), "STOP %Fs (%Fm)", time_of_index(sweep_points - 1), distance_of_index(sweep_points - 1));
	}

	if (uistat.lever_mode == LM_CENTER)
		buf1[0] = S_SARROW[0];
	if (uistat.lever_mode == LM_SPAN)
		buf2[0] = S_SARROW[0];

	ili9341_fill(0, FREQUENCIES_YPOS, LCD_WIDTH, FONT_GET_HEIGHT, DEFAULT_BG_COLOR);

	ili9341_set_background(DEFAULT_BG_COLOR);

	ili9341_set_foreground(DEFAULT_FG_COLOR);
	ili9341_drawstring(buf1, FREQUENCIES_XPOS1, FREQUENCIES_YPOS, false);
	ili9341_drawstring(buf2, FREQUENCIES_XPOS2, FREQUENCIES_YPOS, false);

	{	// draw the bandwidth
		const uint32_t bw = get_bandwidth_frequency(config.bandwidth);
		if (bw < 1000)
			plot_printf(buf1, sizeof(buf1), "%uHz", bw);
		else
			plot_printf(buf1, sizeof(buf1), "%ukHz", bw / 1000);
		plot_printf(buf2, sizeof(buf2), "BW %s %up", buf1, sweep_points);

		//ili9341_set_foreground(DEFAULT_GRID_COLOR);
		ili9341_set_foreground(DEFAULT_BANDWIDTH_COLOR);	// test only
		ili9341_drawstring(buf2, FREQUENCIES_XPOS3, FREQUENCIES_YPOS, false);
	}
}

void draw_cal_status(void)
{
	char c[3];
	int i;
	int x = 0;
	int y = 100;

	ili9341_set_foreground(DEFAULT_FG_COLOR);
	ili9341_set_background(DEFAULT_BG_COLOR);

	ili9341_fill(0, y, OFFSETX, 6*(FONT_STR_HEIGHT), DEFAULT_BG_COLOR);

	if (cal_status & CALSTAT_APPLY)
	{
		c[0] = (cal_status & CALSTAT_INTERPOLATED) ? 'c' : 'C';
		c[1] = (active_props == &current_props) ? '*' : '0' + config.current_id;
		c[2] = 0;
		ili9341_drawstring(c, x, y, false);
		y += FONT_STR_HEIGHT;
	}

	static const struct {char text, zero, mask;} calibration_text[] = {
			{'D', 0, CALSTAT_ED},
			{'R', 0, CALSTAT_ER},
			{'S', 0, CALSTAT_ES},
			{'T', 0, CALSTAT_ET},
			{'X', 0, CALSTAT_EX}};

	for (i = 0; i < SAVEAREA_MAX; i++, y += FONT_STR_HEIGHT)
		if (cal_status & calibration_text[i].mask)
			ili9341_drawstring(&calibration_text[i].text, x, y, false);
}

static uint8_t vbat_reading_count = 0;

static void draw_battery_status(void)
{
	int16_t bat_mv = (int16_t)adc_vbat_read();
   if (bat_mv <= 0)
		return;

	ili9341_set_background(DEFAULT_BG_COLOR);
	ili9341_set_foreground(bat_mv < DEFAULT_BATTERY_WARNING_MV ? DEFAULT_LOW_BAT_COLOR : DEFAULT_NORMAL_BAT_COLOR);

	if (++vbat_reading_count >= 254)
		vbat_reading_count -= 2;	// just keep the LSB toggling

	if (bat_mv <= DEFAULT_BATTERY_WARNING_MV && (vbat_reading_count & 1))
	{	// blank .. flash indicator when we're nearly empty
		ili9341_fill(1, 1, 1 + FONT_GET_HEIGHT, 1 + (FONT_WIDTH * 11), DEFAULT_BG_COLOR);
	}
	else
	{
		#if 0
			// only battery level text, no battery level meter
			char buf[16];
			plot_printf(buf, sizeof(buf), "%d.%02dV %3d%%", bat_mv / 1000, (bat_mv / 10) % 100, ((bat_mv - config.vbat_bottom_mv) * 100) / (BATTERY_TOP_MV - config.vbat_bottom_mv));
			ili9341_drawstringV90(buf, 1, 1);
		#else
			uint8_t buf[18];

			{	// battery level meter
				uint16_t power;

				unsigned int x = 0;

				// battery top
				buf[x++] = 0b00111100;
				buf[x++] = 0b00111100;
				buf[x++] = 0b01111110;
				buf[x++] = 0b10000001;

				// battery level
				for (power = DEFAULT_BATTERY_TOP_MV; power > DEFAULT_BATTERY_BOTTOM_MV; power -= (DEFAULT_BATTERY_TOP_MV - DEFAULT_BATTERY_BOTTOM_MV) / (sizeof(buf) - 1 - 6))
				{
					if (x < sizeof(buf))
						buf[x++] = (power > bat_mv) ? 0b10000001 : 0b10111101;
				}

				// battery bottom
				if (x < sizeof(buf))
					buf[x++] = 0b10000001;
				if (x < sizeof(buf))
					buf[x++] = 0b01111110;
				while (x < sizeof(buf))
					buf[x++] = 0b00000000;

				// draw the battery
				#ifdef SHOW_BATTERY_LEVEL_PERCENT
					blit8BitWidthBitmap(1, 1 + (FONT_WIDTH * 5), 8, x, buf);
				#else
					blit8BitWidthBitmap(1, 1, 8, x, buf);
				#endif
			}

			#ifdef SHOW_BATTERY_LEVEL_PERCENT
				// percentage
				plot_printf((char *)buf, sizeof(buf), "%3d%%", ((bat_mv - DEFAULT_BATTERY_BOTTOM_MV) * 100) / (DEFAULT_BATTERY_TOP_MV - DEFAULT_BATTERY_BOTTOM_MV));
				ili9341_drawstringV90((char *)buf, 1, 0, true);
			#endif

		#endif
	}
}

void request_to_redraw_grid(void)
{
	force_set_markmap();
	redraw_request |= REDRAW_CELLS;
}

void redraw_frame(void)
{
  ili9341_set_background(DEFAULT_BG_COLOR);
  ili9341_clear_screen();
  draw_frequencies();
  draw_cal_status();
}

void plot_init(void)
{
  force_set_markmap();
}
