/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 *
 *   Heavily messed about with by OneOfEleven July 2020
 */

#ifndef LC_MATCHINGH
#define LC_MATCHINGH

typedef struct
{
   float xps;   // Reactance parallel to source (can be NAN if not applicable)
   float xs;    // Serial reactance (can be 0.0 if not applicable)
   float xpl;   // Reactance parallel to load (can be NAN if not applicable)
} t_lc_match;

typedef struct
{
   uint32_t Hz;
   float R0;
   float RL;
   float XL;
   // L-Network solution structure
   t_lc_match matches[4];
   int num_matches;
} t_lc_match_array;

extern t_lc_match_array lc_match_array;

void cell_draw_lc_match(t_lc_match_array *match_array, int x0, int y0, int cell_height);
int lc_match_process(void);

#endif
