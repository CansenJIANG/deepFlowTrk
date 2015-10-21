/*
Copyright (C) 2013 Philippe Weinzaepfel

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef __OPTICALFLOW_H__
#define __OPTICALFLOW_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "image.h"

typedef struct optical_flow_params_s {
  float alpha;             // smoothness weight
  float beta;              // matching weight
  float gamma;             // gradient constancy assumption weight
  float delta;             // color constancy assumption weight
  float sigma;             // presmoothing of the images
  float bk;                // downweight parameter of the matching weight
  float eta;               // downscale factor
  int min_size;            // minimum size of the first level
  int n_inner_iteration;   // number of inner fixed point iterations
  int n_solver_iteration;  // number of solver iterations 
  float sor_omega;         // omega parameter of sor method
} optical_flow_params_t;

/* set flow parameters to default */
void optical_flow_params_default(optical_flow_params_t *params);

/* set flow parameters to sintel one */
void optical_flow_params_sintel(optical_flow_params_t *params);

/* set flow parameters to middlebury one */
void optical_flow_params_middlebury(optical_flow_params_t *params);

/* set flow parameters to kitti one */
void optical_flow_params_kitti(optical_flow_params_t *params);

/* Compute the optical flow between im1 and im2 and store it as two 1-channel images in wx for flow along x-axis and wy for flow along y-axis. match_x, match_y and match_z contains eventually the input matches (NULL for no match) at any scale. */
void optical_flow(image_t *wx, image_t *wy, const color_image_t *im1, const color_image_t *im2, optical_flow_params_t *params, const image_t *match_x, const image_t *match_y, image_t *match_z);

#endif
