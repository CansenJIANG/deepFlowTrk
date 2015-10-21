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

/*
MATLAB Wrapper
*/
#include <mex.h>
#include <assert.h>
#include <math.h>
#include <string.h>

#include <stdint.h>

#define printf mexPrintf

extern "C" {
    #include "image.h"
    #include "opticalflow.h"
    #include "io.h"
}

color_image_t *input3darray_to_color_image(const mxArray *p){
    const int *dims = mxGetDimensions(p);
    const int h = dims[0], w = dims[1];
    assert( dims[2]==3 );
    float *in = (float*) mxGetData(p);
    color_image_t *out = color_image_new(w, h);
    const int s = out->stride;
    for(int c=0 ; c<3 ; c++){
        float *inptr = in + c*w*h;
        float *outptr = out->c1 + c*s*h;
        for( int j=0 ; j<h ; j++){
            for( int i=0 ; i<w ; i++){
                outptr[j*s+i] = inptr[i*h+j];
            }
        }
    }
    return out;
}

void input2darray_to_matches( image_t *match_x, image_t *match_y, image_t *match_z, const mxArray *p){
    const int nmatch = mxGetM(p);
    const int w = match_x->width, h = match_x->height, s = match_x->stride;
    float *data = (float*) mxGetData(p);
    image_erase(match_x); image_erase(match_y); image_erase(match_z);
    for( int i=0 ; i<nmatch ; i++){
        float x1 = data[0*nmatch+i], y1 = data[1*nmatch+i], x2 = data[2*nmatch+i], y2 = data[3*nmatch+i];
        if( x1<0 || y1<0 || x2<0 || y2<0 || x1>=w || y1>=h || x2>=w || y2>=h){
            fprintf(stderr, "Warning: match out of bound: %f %f -> %f %f\n", x1, y1, x2, y2);
            x1 = MINMAX(x1,w);
            x2 = MINMAX(x2,w);
            y1 = MINMAX(y1,h);
            y2 = MINMAX(y2,h);
        }
        int pos = (int) (y1*s+x1);
        match_x->data[ pos ] = x2-x1;
        match_y->data[ pos ] = y2-y1;
        match_z->data[ pos ] = 1.0f;       
    }
}



void flow_to_output3darray(image_t *wx, image_t *wy, mxArray *p){
    const int h = wx->height, w = wx->width, s = wx->stride;
    float *data = (float*) mxGetData(p);
    for( int j=0 ; j<h ; j++) {
        for( int i=0 ; i<w ; i++) {
            data[i*h+j] = wx->data[j*s+i];
            data[(i+w)*h+j] = wy->data[j*s+i];
        }
    }    
}

void mexFunction( int nl, mxArray *pl[], int nr, const mxArray *pr[] ) {
    
    if( nr==0 ){
        usage(MATLAB_OPTIONS);
        return;
    }
    if ( nl != 1){
        usage(MATLAB_OPTIONS);
        mexErrMsgTxt("error: returns one output");
        return;
    }
    if( nr < 2 || nr > 4){
        usage(MATLAB_OPTIONS);
        mexErrMsgTxt("error: takes two to four inputs");
        return;
    }
    
    // The code is originally written for C-order arrays.
    // We thus transpose all arrays in this mex-function which is not efficient...
    
    const int *pDims;
    if( mxGetNumberOfDimensions(pr[0]) != 3 ) mexErrMsgTxt("input images must have 3 dimensions");
    if( !mxIsClass(pr[0], "single") ) mexErrMsgTxt("input images must be single");
    pDims = mxGetDimensions(pr[0]);
    if( pDims[2]!=3 ) mexErrMsgTxt("input images must have 3 channels");
    const int h = pDims[0], w = pDims[1];
    color_image_t *im1 = input3darray_to_color_image( pr[0] );
   
    if( mxGetNumberOfDimensions(pr[1]) != 3 ) mexErrMsgTxt("input images must have 3 dimensions");
    if( !mxIsClass(pr[1], "single") ) mexErrMsgTxt("input images must be single");
    pDims = mxGetDimensions(pr[1]);
    if( pDims[0]!=h || pDims[1]!=w || pDims[2]!=3) mexErrMsgTxt( "input images must have the same size" );
    color_image_t *im2 = input3darray_to_color_image( pr[1] );

    image_t *match_x = NULL, *match_y = NULL, *match_z = NULL;
    if( nr>2 && !mxIsEmpty(pr[2]) ){
        if( mxGetNumberOfDimensions(pr[2]) != 2 ) mexErrMsgTxt("input matches must be a 2d-matrix");
        if( !mxIsClass(pr[2], "single")) mexErrMsgTxt("input matches must be single");  
        pDims = mxGetDimensions(pr[1]); 
        if( pDims[1]<4) mexErrMsgTxt( "input matches must have at least 4 columns: x1 y1 x2 y2" );
        match_x = image_new(w, h); match_y = image_new(w, h); match_z = image_new(w, h); 
        input2darray_to_matches( match_x, match_y, match_z, pr[2]);
    }
        
    // set params to default
    optical_flow_params_t* params = (optical_flow_params_t*) malloc(sizeof(optical_flow_params_t));
    if(!params){
        fprintf(stderr,"error deepflow2(): not enough memory\n");
        exit(1);
    }
    optical_flow_params_default(params);

    // read options
    if( nr > 3 ){
        char *options = mxArrayToString(pr[3]);
        if( !options )  mexErrMsgTxt("Fourth parameter must be a string");
	    int argc=0;
	    char* argv[256];
        argv[argc]=strtok(options," ");
	    while(argv[argc]!=NULL)
	    {
		    argv[++argc]=strtok(NULL," ");
	    }
	    parse_options(params, argc, argv, MATLAB_OPTIONS, w, h);
    }
    
    
    image_t *wx = image_new(im1->width,im1->height), *wy = image_new(im1->width,im1->height);
    optical_flow(wx, wy, im1, im2, params, match_x, match_y, match_z);
    
    int dims[3] = {h,w,2};
    pl[0] = mxCreateNumericArray(3, dims, mxSINGLE_CLASS, mxREAL);
    flow_to_output3darray(wx, wy, pl[0]);
    
    image_delete(wx);
    image_delete(wy);
    image_delete(match_x); image_delete(match_y); image_delete(match_z);
    color_image_delete(im1); color_image_delete(im2);
    free(params);

}
