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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "image.h"
#include "opticalflow.h"
#include "io.h"

int main(int argc, char ** argv){
  
    // load images
    if(argc < 4){
        fprintf(stderr,"Wrong command, require at least 3 arguments.\n\n");
        usage(EXE_OPTIONS);
        exit(1);
    }
    color_image_t *im1 = color_image_load(argv[1]), *im2 = color_image_load(argv[2]);
    if(im1->width != im2->width || im1->height != im2->height){
        fprintf(stderr,"Image dimensions does not match\n");
        exit(1);
    }
  
    // set params to default
    optical_flow_params_t* params = (optical_flow_params_t*) malloc(sizeof(optical_flow_params_t));
    if(!params){
        fprintf(stderr,"error deepflow2(): not enough memory\n");
        exit(1);
    }
    optical_flow_params_default(params);

    // parse options   
    image_t **matches = parse_options(params, argc-4, &argv[4], EXE_OPTIONS, im1->width, im1->height);
	image_t *match_x = matches[0], *match_y = matches[1], *match_z = matches[2];
    free(matches);
    
    image_t *wx = image_new(im1->width,im1->height), *wy = image_new(im1->width,im1->height);
    optical_flow(wx, wy, im1, im2, params, match_x, match_y, match_z);
    writeFlowFile(argv[3], wx, wy);
    image_delete(wx);
    image_delete(wy);
    image_delete(match_x); image_delete(match_y); image_delete(match_z);
    color_image_delete(im1); color_image_delete(im2);
    free(params);

    return 0;
}
