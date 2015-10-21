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
Python wrapper
*/

%module(docstring="Module to compute DeepFlow") deepflow2

%{
    #define SWIG_FILE_WITH_INIT

    #include <numpy/arrayobject.h>

    
    #define CHECK_NUMPY_ARRAY(a, expected_npy)                                 \
      if(!a) {                                                              \
        fprintf(stderr,"error in %s(): NULL input\n",__PRETTY_FUNCTION__);             \
        return NULL;                                                        \
      }                                                                     \
      if(!PyArray_Check(a)) {                                               \
        fprintf(stderr,"error in %s(): input not numpy array\n",__PRETTY_FUNCTION__);  \
        return NULL;                                                        \
      }                                                                     \
      if(!PyArray_ISCONTIGUOUS(a)) {                                        \
        fprintf(stderr,"error in %s(): array is not C-contiguous\n",__PRETTY_FUNCTION__);  \
        return NULL;                                                        \
      }                                                                     \
      if(PyArray_TYPE(a)!=expected_npy) {                                   \
        fprintf(stderr,"error in %s(): input has bad type (type id %d != " #expected_npy " %d)\n",__PRETTY_FUNCTION__, \
                PyArray_TYPE(a),expected_npy);                              \
        return NULL;                                                        \
      }

%}

%init %{
    import_array();
%}


%{
    #include "image.h"
%}

%typemap(in) 
  (image_t* img) 
  (image_t image) {
  
  PyObject* a = $input;
  if(a==Py_None) {
    $1 = NULL;
  } else {
    CHECK_NUMPY_ARRAY(a, NPY_FLOAT)
    image.data = (float*) PyArray_DATA(a);
    a = PyObject_GetAttrString($input,"shape");
    assert(PyTuple_Size(a)==2);
    image.height = PyInt_AsLong(PyTuple_GetItem(a,0));
    image.width = PyInt_AsLong(PyTuple_GetItem(a,1));
    assert( image.width%4==0);
    image.stride = image.width;
    $1=&image;
  }
}
%apply (image_t* img) {(image_t* )};

%typemap(in) 
  (color_image_t* cimg) 
  (color_image_t cimage) {
  
  PyObject* a = $input;
  if(a==Py_None) {
    $1 = NULL;
  } else {
    CHECK_NUMPY_ARRAY(a, NPY_FLOAT)
    cimage.c1 = (float*) PyArray_DATA(a);
    a = PyObject_GetAttrString($input,"shape");
    assert(PyTuple_Size(a)==3);
    assert( PyInt_AsLong(PyTuple_GetItem(a,0)) == 3);
    cimage.height =  PyInt_AsLong(PyTuple_GetItem(a,1));
    cimage.width =  PyInt_AsLong(PyTuple_GetItem(a,2));
    assert( cimage.width%4==0);
    cimage.stride = cimage.width;
    cimage.c2 = cimage.c1 + cimage.stride*cimage.height; 
    cimage.c3 = cimage.c2 + cimage.stride*cimage.height; 
    $1=&cimage;
  }
}
%apply (color_image_t* cimg) {(color_image_t* )};

void deepflow2_numpy( int w, image_t* wx, image_t* wy, color_image_t* im1, color_image_t* im2, image_t* match, char *options);

void usage_python();

%{
    #include "io.h"
    #include "image.h"
    #include "opticalflow.h"
    #include <string.h>
    
    void deepflow2_numpy( int w, image_t* wx, image_t* wy, color_image_t* im1, color_image_t* im2, image_t* match, char *options){
        // correct the width on the inputs
        wx->width = w;
        wy->width = w;
        im1->width = w;
        im2->width = w;
        int h = im1->height;
        
        // read the matches
        image_t *match_x = NULL, *match_y = NULL, *match_z = NULL;
        if( match != NULL){
            match_x = image_new(w, h);
            match_y = image_new(w, h);
            match_z = image_new(w, h);
            image_erase(match_x); image_erase(match_y); image_erase(match_z);
            int i;
            for( i = 0 ; i<match->height ; i++){
                float x1 = match->data[i*match->stride+0], y1 = match->data[i*match->stride+1], x2 = match->data[i*match->stride+2], y2 = match->data[i*match->stride+3];
                if( x1<0 || y1<0 || x2<0 || y2<0 || x1>=w || y1>=h || x2>=w || y2>=h){
                    fprintf(stderr, "Warning: match out of bound: %f %f -> %f %f\n", x1, y1, x2, y2);
                    x1 = MINMAX(x1, w);
                    x2 = MINMAX(x2, w);
                    y1 = MINMAX(y1, h);
                    y2 = MINMAX(y2, h);
                }
                int pos = (int) (y1*match_x->stride+x1);
                match_x->data[ pos ] = x2-x1;
                match_y->data[ pos ] = y2-y1;
                match_z->data[ pos ] = 1.0f;                         
            }
        }
        
        // set params to default
        optical_flow_params_t* params = (optical_flow_params_t*) malloc(sizeof(optical_flow_params_t));
        if(!params){
            fprintf(stderr,"error deepflow2(): not enough memory\n");
            exit(1);
        }
        optical_flow_params_default(params);

        // read options
        if( options!=NULL ){
	        int argc=0;
	        char* argv[256];
            argv[argc]=strtok(options," ");
	        while(argv[argc]!=NULL)
	        {
		        argv[++argc]=strtok(NULL," ");
	        }
	        parse_options(params, argc, argv, PYTHON_OPTIONS, w, h);
        }

        // launch flow
        image_t *flowx = image_new(w, h), *flowy = image_new(w,h); // optical_flow changes the pointer to the data
        optical_flow(flowx, flowy, im1, im2, params, match_x, match_y, match_z);
        memcpy( wx->data, flowx->data, sizeof(float)*flowx->stride*h);
        memcpy( wy->data, flowy->data, sizeof(float)*flowx->stride*h);        
       
        free(params);
        image_delete(flowx);
        image_delete(flowy);
        if(match!=NULL){
            image_delete(match_x);
            image_delete(match_y);
            image_delete(match_z);        
        }
    }
    
    void usage_python(){
        usage(PYTHON_OPTIONS);
    }

%}


%pythoncode %{  
    from numpy import float32, concatenate, empty, rollaxis, ascontiguousarray, pad
    def deepflow2( im1=None, im2=None, match=None, options=""):
        """
        flow = deepflow2.deepflow2(image1, image2, match=None, options='')
        Compute the flow between two images, eventually using given matches.
        Images must be HxWx3 numpy arrays (convert to float32).
        Match is an optional numpy array argument (None by default, ie no input match), where each row starts by x1 y1 x2 y2.
        Options is an optional string argument ('' by default), to set the options. Type deepflow2() to see the list of available options.
        The function returns the optical flow as a HxWx2 numpy array."""
        #convert images
        if None in (im1,im2):
            usage_python()
            return
        assert im1.shape == im2.shape, "images must have the same shape"
        if im1.dtype != float32:
            im1 = im1.astype(float32)
        if im2.dtype != float32:
            im2 = im2.astype(float32)
        h, w, nchannels = im1.shape
        assert nchannels==3, "images must have 3 channels"
        stride = 4*((w+3)//4)
        im1 = pad( rollaxis(im1,2), ((0,0),(0,0),(0, stride-w)), 'constant')
        im2 = pad( rollaxis(im2,2), ((0,0),(0,0),(0, stride-w)), 'constant')
        # allocate flow
        flowx = empty((h,stride), dtype=float32)
        flowy = empty((h,stride), dtype=float32)
        # compute flow
        if match is not None:
            assert match.shape[1]>=4
            match = ascontiguousarray(match[:,:4], dtype=float32)
        deepflow2_numpy( w, flowx, flowy, im1, im2, match, options)
        return concatenate ( (flowx[:,:w,None], flowy[:,:w,None]), axis=2)
%} 







