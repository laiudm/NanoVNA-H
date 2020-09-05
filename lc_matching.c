/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 *
 *   Heavily messed about with by OneOfEleven July 2020
 */

// calculate physical component values to match an impendace to 'ref_impedance' (ie 50R)

#include <arm_math.h>

#include "nanovna.h"
#include "lc_matching.h"

#ifdef USE_LC_MATCHING

   t_lc_match_array lc_match_array;

   void lc_match_quadratic_equation(float a, float b, float c, float *x)
   {
      const float d = (b * b) - (4.0f * a * c);
      if (d < 0)
      {
         x[0] = 0.0f;
         x[1] = 0.0f;
      }
      else
      {
         const float sd = sqrtf(d);
         const float a2 = 2.0f * a;
         x[0] = (-b + sd) / a2;
         x[1] = (-b - sd) / a2;
      }
   }

   // Calculate two solutions for ZL where (R + X * X / R) > R0
   void lc_match_calc_hi(float R0, float RL, float XL, t_lc_match *matches)
   {
      float xs[2];
      float xp[2];

      const float a = R0 - RL;
      const float b =  2.0f * XL * R0;
      const float c = R0 * ((XL * XL) + (RL * RL));
      lc_match_quadratic_equation(a, b, c, xp);

      // found two impedances parallel to load
      //
      // now calculate serial impedances

      const float RL1 = -XL * xp[0];
      const float XL1 =  RL * xp[0];
      const float RL2 =  RL + 0.0f;
      const float XL2 =  XL + xp[0];
      xs[0] = -((RL2 * XL1) - (RL1 * XL2)) / ((RL2 * RL2) + (XL2 * XL2));

      const float RL3 = -XL * xp[1];
      const float XL3 =  RL * xp[1];
      const float RL4 =  RL + 0.0f;
      const float XL4 =  XL + xp[1];
      xs[1] = -((RL4 * XL3) - (RL3 * XL4)) / ((RL4 * RL4) + (XL4 * XL4));

      matches[0].xs  = xs[0];
//      matches[0].xps = NAN;
      matches[0].xps = 0.0f;
      matches[0].xpl = xp[0];

      matches[1].xs  = xs[1];
//      matches[1].xps = NAN;
      matches[1].xps = 0.0f;
      matches[1].xpl = xp[1];
   }

   // Calculate two solutions for ZL where R < R0
   void lc_match_calc_lo(float R0, float RL, float XL, t_lc_match *matches)
   {
      float xs[2];
      float xp[2];

      // Calculate Xs

      const float a = 1.0f;
      const float b = 2.0f * XL;
      const float c = (RL * RL) + (XL * XL) - (R0 * RL);
      lc_match_quadratic_equation(a, b, c, xs);

      // got two serial impedances that change ZL to the Y.real = 1/R0
      //
      // now calculate impedances parallel to source

      const float RL1 = RL  + 0.0f;
      const float XL1 = XL  + xs[0];
      const float RL3 = RL1 * R0;
      const float XL3 = XL1 * R0;
      const float RL5 = RL1 - R0;
      const float XL5 = XL1 - 0.0f;
      xp[0] = ((RL5 * XL3) - (RL3 * XL5)) / ((RL5 * RL5) + (XL5 * XL5));

      const float RL2 = RL  + 0.0f;
      const float XL2 = XL  + xs[1];
      const float RL4 = RL2 * R0;
      const float XL4 = XL2 * R0;
      const float RL6 = RL2 - R0;
      const float XL6 = XL2 - 0.0f;
      xp[1] = ((RL6 * XL4) - (RL4 * XL6)) / ((RL6 * RL6) + (XL6 * XL6));

      matches[0].xs  = xs[0];
      matches[0].xps = xp[0];
//      matches[0].xpl = NAN;
      matches[0].xpl = 0.0f;

      matches[1].xs  = xs[1];
      matches[1].xps = xp[1];
//      matches[1].xpl = NAN;
      matches[1].xpl = 0.0f;
   }

   float _nonz(float f)
   {
      return (0.0f == f || -0.0f == f) ? 1e-30f : f;
   }

   float lc_match_calc_VSWR(float RL, float XL, float ref_impedance)
   {
      const float XL2 = XL * XL;
      const float R   = (RL > 0.0f) ? RL : 0.0f;
      const float n   = R - ref_impedance;
      const float p   = R + ref_impedance;

      float ro = sqrtf(((n * n) + XL2) / _nonz((p * p) + XL2));
      if (ro > 0.9999f)
         ro = 0.9999f;

      const float vswr = (1.0f + ro) / (1.0f - ro);

      return vswr;
   }

   int lc_match_calc(float RL, float XL, t_lc_match *matches, float ref_impedance)
   {
      const float R0 = ref_impedance;

      if (RL <= 0.5f)
         return -1;

      const float vswr = lc_match_calc_VSWR(RL, XL, R0);
      const float q_factor = XL / RL;

      if (vswr <= 1.1f || q_factor >= 100.0f)
         return 0;      // no need for any matching

      if (RL > (R0 / 1.1f) && RL < (R0 * 1.1f))
      {   // only one solution is enough: just a serial reactance
         // this gives SWR < 1.1 if R is within the range 0.91 .. 1.1 of R0
//         matches[0].xpl = NAN;
         matches[0].xpl = 0.0f;
//         matches[0].xps = NAN;
         matches[0].xps = 0.0f;
         matches[0].xs  = -XL;
         return 1;
      }

      if (RL >= R0)
      {   // two Hi-Z solutions
         lc_match_calc_hi(R0, RL, XL, &matches[0]);
         return 2;
      }

      // compute Lo-Z solutions
      lc_match_calc_lo(R0, RL, XL, &matches[0]);
      if ((RL + (XL * q_factor)) <= R0)
         return 2;

      // two more Hi-Z solutions exist
      lc_match_calc_hi(R0, RL, XL, &matches[2]);
      return 4;
   }

   void lc_match_x_str(uint32_t FHz, float X, char *str, int str_size)
   {
      if (isnan(X))
      {
         strcpy(str, "         ");
         return;
      }

      if (0.0f == X || -0.0f == X)
      {   // catch divide-by-zero
         strcpy(str, "         ");
         return;
      }

      if (X < 0.0f)
      {
         const float c = 1.0f / (2.0f * VNA_PI * FHz * -X);
         if (c >= 1e0f)   plot_printf(str, str_size, "%6.2f F ", c * 1e0f);
         else
         if (c >= 1e-3f)  plot_printf(str, str_size, "%6.2f mF", c * 1e3f);
         else
         if (c >= 1e-6f)  plot_printf(str, str_size, "%6.2f uF", c * 1e6f);
         else
         if (c >= 1e-9f)  plot_printf(str, str_size, "%6.2f nF", c * 1e9f);
         else
            plot_printf(str, str_size, "%6.2f pF", c * 1e12f);
      }
      else
      {
         const float l = X / (2.0f * VNA_PI * FHz);
         if (l >= 1e0f)    plot_printf(str, str_size, "%6.2f H ", l * 1e0f);
         else
         if (l >= 1e-3f)   plot_printf(str, str_size, "%6.2f mH", l * 1e3f);
         else
         if (l >= 1e-6f)   plot_printf(str, str_size, "%6.2f uH", l * 1e6f);
         else
            plot_printf(str, str_size, "%6.2f nH", l * 1e9f);
      }
   }

   extern void cell_drawstring(char *str, int x, int y, bool fixed_pitch);

   void cell_draw_lc_match(t_lc_match_array *match_array, int x0, int y0, int cell_height)
   {
      char s[38];

      int xp = OFFSETX +  0 - x0;
      int yp = OFFSETY + 30 - y0;

      ili9341_set_background(DEFAULT_BG_COLOR);
      ili9341_set_foreground(DEFAULT_FG_COLOR);

      //ili9341_fill(xp, yp, 2 + (FONT_WIDTH * 30), 2 + ((FONT_STR_HEIGHT + 3 + 4) * 3) + 8 + 2, DEFAULT_BG_COLOR);

      xp += 2;
      yp += 2;

      if (yp > -FONT_GET_HEIGHT && yp < cell_height)
      {
         plot_printf(s, sizeof(s), "LC match for source Z0 = %0.1f"S_OHM" ", match_array->R0);
         cell_drawstring(s, xp, yp, false);
      }

      yp += FONT_STR_HEIGHT + 3 + 4;

      if (yp > -FONT_GET_HEIGHT && yp < cell_height)
      {
         plot_printf(s, sizeof(s), "%qHz %0.1f %c j%0.1f"S_OHM" ", match_array->Hz, match_array->RL, (match_array->XL >= 0) ? '+' : '-', fabsf(match_array->XL));
         cell_drawstring(s, xp, yp, false);
      }

      yp += FONT_STR_HEIGHT + 3 + 4;

      if (match_array->num_matches < 0)
      {
         if (yp > -FONT_GET_HEIGHT && yp < cell_height)
            cell_drawstring(     "No LC match for this load ", xp, yp, false);
      }
      else
      if (match_array->num_matches == 0)
      {
         if (yp > -FONT_GET_HEIGHT && yp < cell_height)
            cell_drawstring(     "No need for LC matching ", xp, yp, false);
      }
      else
      {
         int i;

         if (yp > -FONT_GET_HEIGHT && yp < cell_height)
            cell_drawstring(     "src shunt    series     load shunt", xp, yp, false);

         for (i = 0; i < match_array->num_matches; i++)
         {
            char str[3][12];

            yp += FONT_STR_HEIGHT + 3;

            if (yp > -FONT_GET_HEIGHT && yp < cell_height)
            {
               lc_match_x_str(match_array->Hz, match_array->matches[i].xpl, str[0], sizeof(str[0]));
               lc_match_x_str(match_array->Hz, match_array->matches[i].xps, str[1], sizeof(str[1]));
               lc_match_x_str(match_array->Hz, match_array->matches[i].xs,  str[2], sizeof(str[2]));

               plot_printf(s, sizeof(s), "%s  %s  %s ", str[1], str[2], str[0]);

               //ili9341_fill(xp, yp, 2 + (FONT_WIDTH * 30), FONT_STR_HEIGHT + 3, DEFAULT_BG_COLOR);
               cell_drawstring(s, xp, yp, true);
            }
         }
      }
   }

   int lc_match_process(void)
   {
        memset(&lc_match_array, 0, sizeof(lc_match_array));

      if (!(config.flags & CONFIG_FLAGS_LC_MATCH))
         return -1;

      const int am = active_marker;
      if (am < 0 || am >= MARKERS_MAX)
         return -1;

      if (!marker_enabled(am))
         return -1;

      const int index = marker_index(am);
      if (index < 0 || index >= sweep_points)
         return -1;

      lc_match_array.R0 = 50.0f;
      lc_match_array.Hz = frequencies[index];

      if (lc_match_array.Hz == 0)
         return -1;

      const float *coeff = measured[0][index];

      // compute the impedance at the chosen frequency
      impedance(coeff, &lc_match_array.RL, &lc_match_array.XL, lc_match_array.R0);

      // compute the possible LC matches
        lc_match_array.num_matches = (lc_match_array.Hz > 0) ? lc_match_calc(lc_match_array.RL, lc_match_array.XL, lc_match_array.matches, lc_match_array.R0) : 0;

        return lc_match_array.num_matches;
   }
#endif
