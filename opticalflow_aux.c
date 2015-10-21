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
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <string.h>
#include "opticalflow_aux.h"

#include <xmmintrin.h>
typedef __v4sf v4sf;

#define datanorm 0.1f*0.1f//0.01f // square of the normalization factor
#define epsilon_color (0.001f*0.001f)//0.000001f
#define epsilon_grad (0.001f*0.001f)//0.000001f
#define epsilon_desc (0.001f*0.001f)//0.000001f
#define epsilon_smooth (0.001f*0.001f)//0.000001f

/* warp a color image according to a flow. src is the input image, wx and wy, the input flow. dst is the warped image and mask contains 0 or 1 if the pixels goes outside/inside image boundaries */
void image_warp(color_image_t *dst, image_t *mask, const color_image_t *src, const image_t *wx, const image_t *wy){
    int i, j, offset, incr_line = mask->stride-mask->width, x, y, x1, x2, y1, y2;
    float xx, yy, dx, dy;
    for(j=0,offset=0 ; j<src->height ; j++){
        for(i=0 ; i<src->width ; i++,offset++){
	        xx = i+wx->data[offset];
	        yy = j+wy->data[offset];
	        x = floor(xx);
	        y = floor(yy);
	        dx = xx-x;
	        dy = yy-y;
	        mask->data[offset] = (xx>=0 && xx<=src->width-1 && yy>=0 && yy<=src->height-1);
	        x1 = MINMAX(x,src->width);
	        x2 = MINMAX(x+1,src->width);
	        y1 = MINMAX(y,src->height);
	        y2 = MINMAX(y+1,src->height);
	        dst->c1[offset] = 
	            src->c1[y1*src->stride+x1]*(1.0f-dx)*(1.0f-dy) +
	            src->c1[y1*src->stride+x2]*dx*(1.0f-dy) +
	            src->c1[y2*src->stride+x1]*(1.0f-dx)*dy +
	            src->c1[y2*src->stride+x2]*dx*dy;
	        dst->c2[offset] = 
	            src->c2[y1*src->stride+x1]*(1.0f-dx)*(1.0f-dy) +
	            src->c2[y1*src->stride+x2]*dx*(1.0f-dy) +
	            src->c2[y2*src->stride+x1]*(1.0f-dx)*dy +
	            src->c2[y2*src->stride+x2]*dx*dy;
	        dst->c3[offset] = 
	            src->c3[y1*src->stride+x1]*(1.0f-dx)*(1.0f-dy) +
	            src->c3[y1*src->stride+x2]*dx*(1.0f-dy) +
	            src->c3[y2*src->stride+x1]*(1.0f-dx)*dy +
	            src->c3[y2*src->stride+x2]*dx*dy;
	    }
        offset += incr_line;
    }
}

/* compute image first and second order spatio-temporal derivatives of a color image */
void get_derivatives(const color_image_t *im1, const color_image_t *im2, const convolution_t *deriv,
		     color_image_t *dx, color_image_t *dy, color_image_t *dt, 
		     color_image_t *dxx, color_image_t *dxy, color_image_t *dyy, color_image_t *dxt, color_image_t *dyt){
    // derivatives are computed on the mean of the first image and the warped second image
    color_image_t *tmp_im2 = color_image_new(im2->width,im2->height);
    v4sf *tmp_im2p = (v4sf*) tmp_im2->c1, *dtp = (v4sf*) dt->c1, *im1p = (v4sf*) im1->c1, *im2p = (v4sf*) im2->c1;
    const v4sf half = {0.5f,0.5f,0.5f,0.5f};
    int i=0;
    for(i=0 ; i<3*im1->height*im1->stride/4 ; i++){
        *tmp_im2p = half * ( (*im2p) + (*im1p) );
        *dtp = (*im2p)-(*im1p);
        dtp+=1; im1p+=1; im2p+=1; tmp_im2p+=1;
    }   
    // compute all other derivatives
    color_image_convolve_hv(dx, tmp_im2, deriv, NULL);
    color_image_convolve_hv(dy, tmp_im2, NULL, deriv);
    color_image_convolve_hv(dxx, dx, deriv, NULL);
    color_image_convolve_hv(dxy, dx, NULL, deriv);
    color_image_convolve_hv(dyy, dy, NULL, deriv);
    color_image_convolve_hv(dxt, dt, deriv, NULL);
    color_image_convolve_hv(dyt, dt, NULL, deriv);
    // free memory
    color_image_delete(tmp_im2);
}

/* compute smoothness weight using image gradient's norm */
image_t* compute_smoothness_weight(color_image_t *im, float coef, const convolution_t *deriv) {
    image_t* lum = image_new(im->width, im->height), *lum_x = image_new(im->width, im->height), *lum_y = image_new(im->width, im->height);
    int i;
    // ocompute luminance
    v4sf *im1p = (v4sf*) im->c1, *im2p = (v4sf*) im->c2, *im3p = (v4sf*) im->c3, *lump = (v4sf*) lum->data;
    for( i=0 ; i<im->height*im->stride/4 ; i++){
        *lump = (0.299f*(*im1p) + 0.587f*(*im2p) + 0.114f*(*im3p))/255.0f;
        lump+=1; im1p+=1; im2p+=1; im3p+=1;
    }
    // compute derivatives with five-point tencil
    convolve_horiz(lum_x, lum, deriv);
    convolve_vert(lum_y, lum, deriv);
    // compute lum norm
    lump = (v4sf*) lum->data;
    v4sf *lumxp = (v4sf*) lum_x->data, *lumyp = (v4sf*) lum_y->data;
    for( i=0 ; i<lum->height*lum->stride/4 ; i++){
        *lump = -coef*__builtin_ia32_sqrtps( (*lumxp)*(*lumxp) + (*lumyp)*(*lumyp));
        lump[0][0] = 0.5f*expf(lump[0][0]);
        lump[0][1] = 0.5f*expf(lump[0][1]);
        lump[0][2] = 0.5f*expf(lump[0][2]);
        lump[0][3] = 0.5f*expf(lump[0][3]);
        lump+=1; lumxp+=1; lumyp+=1;
    }
    image_delete(lum_x);
    image_delete(lum_y);
    return lum;
}

/* compute the smoothness term */
/* It is represented as two images, the first one for horizontal smoothness, the second for vertical
   in dst_horiz, the pixel i,j represents the smoothness weight between pixel i,j and i,j+1
   in dst_vert, the pixel i,j represents the smoothness weight between pixel i,j and i+1,j */
void compute_smoothness(image_t *dst_horiz, image_t *dst_vert, const image_t *uu, const image_t *vv, const image_t *dpsis_weight, const convolution_t *deriv_flow, const float half_alpha) {
    int w = uu->width, h = uu->height, s = uu->stride, i, j, offset;
    image_t *ux1 = image_new(w,h), *uy1 = image_new(w,h), *vx1 = image_new(w,h), *vy1 = image_new(w,h), 
        *ux2 = image_new(w,h), *uy2 = image_new(w,h), *vx2 = image_new(w,h), *vy2 = image_new(w,h);  
    // compute ux1, vx1, filter [-1 1]
    for( j=0 ; j<h ; j++) {
        offset = j*s;
        for( i=0 ; i<w-1 ; i++, offset++) {
            ux1->data[offset] = uu->data[offset+1] - uu->data[offset];
            vx1->data[offset] = vv->data[offset+1] - vv->data[offset];
	    }
    }
    // compute uy1, vy1, filter [-1;1]
    for( j=0 ; j<h-1 ; j++) {
        offset = j*s;
        for( i=0 ; i<w ; i++, offset++) {
            uy1->data[offset] = uu->data[offset+s] - uu->data[offset];
            vy1->data[offset] = vv->data[offset+s] - vv->data[offset];
	    }
    }
    // compute ux2, uy2, vx2, vy2, filter [-0.5 0 0.5]
    convolve_horiz(ux2,uu,deriv_flow);
    convolve_horiz(vx2,vv,deriv_flow);
    convolve_vert(uy2,uu,deriv_flow);
    convolve_vert(vy2,vv,deriv_flow);
    // compute final value, horiz
    for( j=0 ; j<h ; j++) {
        offset = j*s;
        for( i=0 ; i<w-1 ; i++, offset++) {
            float tmp = 0.5f*(uy2->data[offset]+uy2->data[offset+1]);
            float uxsq = ux1->data[offset]*ux1->data[offset] + tmp*tmp;
            tmp = 0.5f*(vy2->data[offset]+vy2->data[offset+1]);
            float vxsq = vx1->data[offset]*vx1->data[offset] + tmp*tmp;
            tmp = uxsq + vxsq;
            dst_horiz->data[offset] = (dpsis_weight->data[offset]+dpsis_weight->data[offset+1])*half_alpha / sqrtf( tmp + epsilon_smooth ) ;
        }
        memset( &dst_horiz->data[j*s+w-1], 0, sizeof(float)*(s-w+1));
    }
  // compute final value, vert
  for( j=0 ; j<h-1 ; j++)
    {
      offset = j*s;
      for( i=0 ; i<w ; i++, offset++) {
	      float tmp = 0.5f*(ux2->data[offset]+ux2->data[offset+s]);
	      float uysq = uy1->data[offset]*uy1->data[offset] + tmp*tmp;
	      tmp = 0.5f*(vx2->data[offset]+vx2->data[offset+s]);
	      float vysq = vy1->data[offset]*vy1->data[offset] + tmp*tmp;
	      tmp = uysq + vysq;
	      dst_vert->data[offset] = (dpsis_weight->data[offset]+dpsis_weight->data[offset+s])*half_alpha / sqrtf( tmp + epsilon_smooth ) ;
	    }
    }
  memset( &dst_vert->data[(h-1)*s], 0, sizeof(float)*s);
  image_delete(ux1); image_delete(uy1); image_delete(vx1); image_delete(vy1);
  image_delete(ux2); image_delete(uy2); image_delete(vx2); image_delete(vy2);
}


/* sub the laplacian (smoothness term) to the right-hand term */
void sub_laplacian(image_t *dst, const image_t *src, const image_t *weight_horiz, const image_t *weight_vert){
    int j;
    const int offsetline = src->stride-src->width;
    float *src_ptr = src->data, *dst_ptr = dst->data, *weight_horiz_ptr = weight_horiz->data;
    // horizontal filtering
    for(j=src->height+1;--j;){ // faster than for(j=0;j<src->height;j++)
        int i;
        for(i=src->width;--i;){
	        const float tmp = (*weight_horiz_ptr)*((*(src_ptr+1))-(*src_ptr));
	        *dst_ptr += tmp;
	        *(dst_ptr+1) -= tmp;
	        dst_ptr++;
	        src_ptr++;
	        weight_horiz_ptr++;
	    }
        dst_ptr += offsetline+1;
        src_ptr += offsetline+1;
        weight_horiz_ptr += offsetline+1;
    }
  
    v4sf *wvp = (v4sf*) weight_vert->data, *srcp = (v4sf*) src->data, *srcp_s = (v4sf*) (src->data+src->stride), *dstp = (v4sf*) dst->data, *dstp_s = (v4sf*) (dst->data+src->stride);
    for(j=1+(src->height-1)*src->stride/4 ; --j ;){
        const v4sf tmp = (*wvp) * ((*srcp_s)-(*srcp));
        *dstp += tmp;
        *dstp_s -= tmp;
        wvp+=1; srcp+=1; srcp_s+=1; dstp+=1; dstp_s+=1;
    }
}

/* compute the dataterm and the matching term
   a11 a12 a22 represents the 2x2 diagonal matrix, b1 and b2 the right hand side
   other (color) images are input */
void compute_data_and_match(image_t *a11, image_t *a12, image_t *a22, image_t *b1, image_t *b2, image_t *mask, image_t *wx, image_t *wy, image_t *du, image_t *dv, image_t *uu, image_t *vv, color_image_t *Ix, color_image_t *Iy, color_image_t *Iz, color_image_t *Ixx, color_image_t *Ixy, color_image_t *Iyy, color_image_t *Ixz, color_image_t *Iyz, image_t *desc_weight, image_t *desc_flow_x, image_t *desc_flow_y, const float half_delta_over3, const float half_beta, const float half_gamma_over3){
 
    const v4sf dnorm = {datanorm, datanorm, datanorm, datanorm};
    const v4sf hdover3 = {half_delta_over3, half_delta_over3, half_delta_over3, half_delta_over3};
    const v4sf epscolor = {epsilon_color, epsilon_color, epsilon_color, epsilon_color};
    const v4sf hgover3 = {half_gamma_over3, half_gamma_over3, half_gamma_over3, half_gamma_over3};
    const v4sf epsgrad = {epsilon_grad, epsilon_grad, epsilon_grad, epsilon_grad};
    const v4sf hbeta = {half_beta,half_beta,half_beta,half_beta};
    const v4sf epsdesc = {epsilon_desc,epsilon_desc,epsilon_desc,epsilon_desc};
    
    v4sf *dup = (v4sf*) du->data, *dvp = (v4sf*) dv->data,
        *maskp = (v4sf*) mask->data,
        *a11p = (v4sf*) a11->data, *a12p = (v4sf*) a12->data, *a22p = (v4sf*) a22->data, 
        *b1p = (v4sf*) b1->data, *b2p = (v4sf*) b2->data, 
        *ix1p=(v4sf*)Ix->c1, *iy1p=(v4sf*)Iy->c1, *iz1p=(v4sf*)Iz->c1, *ixx1p=(v4sf*)Ixx->c1, *ixy1p=(v4sf*)Ixy->c1, *iyy1p=(v4sf*)Iyy->c1, *ixz1p=(v4sf*)Ixz->c1, *iyz1p=(v4sf*) Iyz->c1, 
        *ix2p=(v4sf*)Ix->c2, *iy2p=(v4sf*)Iy->c2, *iz2p=(v4sf*)Iz->c2, *ixx2p=(v4sf*)Ixx->c2, *ixy2p=(v4sf*)Ixy->c2, *iyy2p=(v4sf*)Iyy->c2, *ixz2p=(v4sf*)Ixz->c2, *iyz2p=(v4sf*) Iyz->c2, 
        *ix3p=(v4sf*)Ix->c3, *iy3p=(v4sf*)Iy->c3, *iz3p=(v4sf*)Iz->c3, *ixx3p=(v4sf*)Ixx->c3, *ixy3p=(v4sf*)Ixy->c3, *iyy3p=(v4sf*)Iyy->c3, *ixz3p=(v4sf*)Ixz->c3, *iyz3p=(v4sf*) Iyz->c3, 
        *uup = (v4sf*) uu->data, *vvp = (v4sf*)vv->data, *wxp = (v4sf*)wx->data, *wyp = (v4sf*)wy->data,
        *descflowxp = (v4sf*)desc_flow_x->data, *descflowyp = (v4sf*)desc_flow_y->data, *descweightp = (v4sf*)desc_weight->data;
            
    memset(a11->data, 0, sizeof(float)*uu->height*uu->stride);
    memset(a12->data, 0, sizeof(float)*uu->height*uu->stride);
    memset(a22->data, 0, sizeof(float)*uu->height*uu->stride);
    memset(b1->data , 0, sizeof(float)*uu->height*uu->stride);
    memset(b2->data , 0, sizeof(float)*uu->height*uu->stride);
              
    int i;
    for(i = 0 ; i<uu->height*uu->stride/4 ; i++){
        v4sf tmp, tmp2, tmp3, tmp4, tmp5, tmp6, n1, n2, n3, n4, n5, n6;
        // dpsi color
        if(half_delta_over3){
            tmp  = *iz1p + (*ix1p)*(*dup) + (*iy1p)*(*dvp);
            n1 = (*ix1p) * (*ix1p) + (*iy1p) * (*iy1p) + dnorm;
            tmp2 = *iz2p + (*ix2p)*(*dup) + (*iy2p)*(*dvp);
            n2 = (*ix2p) * (*ix2p) + (*iy2p) * (*iy2p) + dnorm;
            tmp3 = *iz3p + (*ix3p)*(*dup) + (*iy3p)*(*dvp);
            n3 = (*ix3p) * (*ix3p) + (*iy3p) * (*iy3p) + dnorm;
            tmp = (*maskp) * hdover3 / __builtin_ia32_sqrtps(tmp*tmp/n1 + tmp2*tmp2/n2 + tmp3*tmp3/n3 + epscolor);
            tmp3 = tmp/n3; tmp2 = tmp/n2; tmp /= n1;
            *a11p += tmp  * (*ix1p) * (*ix1p);
            *a12p += tmp  * (*ix1p) * (*iy1p);
            *a22p += tmp  * (*iy1p) * (*iy1p);
            *b1p -=  tmp  * (*iz1p) * (*ix1p);
            *b2p -=  tmp  * (*iz1p) * (*iy1p);
            *a11p += tmp2 * (*ix2p) * (*ix2p);
            *a12p += tmp2 * (*ix2p) * (*iy2p);
            *a22p += tmp2 * (*iy2p) * (*iy2p);
            *b1p -=  tmp2 * (*iz2p) * (*ix2p);
            *b2p -=  tmp2 * (*iz2p) * (*iy2p);
            *a11p += tmp3 * (*ix3p) * (*ix3p);
            *a12p += tmp3 * (*ix3p) * (*iy3p);
            *a22p += tmp3 * (*iy3p) * (*iy3p);
            *b1p -=  tmp3 * (*iz3p) * (*ix3p);
            *b2p -=  tmp3 * (*iz3p) * (*iy3p);
        }
        // dpsi gradient
        n1 = (*ixx1p) * (*ixx1p) + (*ixy1p) * (*ixy1p) + dnorm;
        n2 = (*iyy1p) * (*iyy1p) + (*ixy1p) * (*ixy1p) + dnorm;
        tmp  = *ixz1p + (*ixx1p) * (*dup) + (*ixy1p) * (*dvp);
        tmp2 = *iyz1p + (*ixy1p) * (*dup) + (*iyy1p) * (*dvp);
        n3 = (*ixx2p) * (*ixx2p) + (*ixy2p) * (*ixy2p) + dnorm;
        n4 = (*iyy2p) * (*iyy2p) + (*ixy2p) * (*ixy2p) + dnorm;
        tmp3 = *ixz2p + (*ixx2p) * (*dup) + (*ixy2p) * (*dvp);
        tmp4 = *iyz2p + (*ixy2p) * (*dup) + (*iyy2p) * (*dvp);
        n5 = (*ixx3p) * (*ixx3p) + (*ixy3p) * (*ixy3p) + dnorm;
        n6 = (*iyy3p) * (*iyy3p) + (*ixy3p) * (*ixy3p) + dnorm;
        tmp5 = *ixz3p + (*ixx3p) * (*dup) + (*ixy3p) * (*dvp);
        tmp6 = *iyz3p + (*ixy3p) * (*dup) + (*iyy3p) * (*dvp);
        tmp = (*maskp) * hgover3 / __builtin_ia32_sqrtps(tmp*tmp/n1 + tmp2*tmp2/n2 + tmp3*tmp3/n3 + tmp4*tmp4/n4 + tmp5*tmp5/n5 + tmp6*tmp6/n6 + epsgrad);
        tmp6 = tmp/n6; tmp5 = tmp/n5; tmp4 = tmp/n4; tmp3 = tmp/n3; tmp2 = tmp/n2; tmp /= n1;      
        *a11p += tmp *(*ixx1p)*(*ixx1p) + tmp2*(*ixy1p)*(*ixy1p);
        *a12p += tmp *(*ixx1p)*(*ixy1p) + tmp2*(*ixy1p)*(*iyy1p);
        *a22p += tmp2*(*iyy1p)*(*iyy1p) + tmp *(*ixy1p)*(*ixy1p);
        *b1p -=  tmp *(*ixx1p)*(*ixz1p) + tmp2*(*ixy1p)*(*iyz1p);
        *b2p -=  tmp2*(*iyy1p)*(*iyz1p) + tmp *(*ixy1p)*(*ixz1p);
        *a11p += tmp3*(*ixx2p)*(*ixx2p) + tmp4*(*ixy2p)*(*ixy2p);
        *a12p += tmp3*(*ixx2p)*(*ixy2p) + tmp4*(*ixy2p)*(*iyy2p);
        *a22p += tmp4*(*iyy2p)*(*iyy2p) + tmp3*(*ixy2p)*(*ixy2p);
        *b1p -=  tmp3*(*ixx2p)*(*ixz2p) + tmp4*(*ixy2p)*(*iyz2p);
        *b2p -=  tmp4*(*iyy2p)*(*iyz2p) + tmp3*(*ixy2p)*(*ixz2p);
        *a11p += tmp5*(*ixx3p)*(*ixx3p) + tmp6*(*ixy3p)*(*ixy3p);
        *a12p += tmp5*(*ixx3p)*(*ixy3p) + tmp6*(*ixy3p)*(*iyy3p);
        *a22p += tmp6*(*iyy3p)*(*iyy3p) + tmp5*(*ixy3p)*(*ixy3p);
        *b1p -=  tmp5*(*ixx3p)*(*ixz3p) + tmp6*(*ixy3p)*(*iyz3p);
        *b2p -=  tmp6*(*iyy3p)*(*iyz3p) + tmp5*(*ixy3p)*(*ixz3p);  
        if(half_beta){ // dpsi_match
            tmp  = *uup - (*descflowxp);
            tmp2 = *vvp - (*descflowyp);
            tmp = hbeta*(*descweightp)/__builtin_ia32_sqrtps(tmp*tmp+tmp2*tmp2+epsdesc);
            *a11p += tmp;
            *a22p += tmp;
            *b1p -= tmp*((*wxp)-(*descflowxp));
            *b2p -= tmp*((*wyp)-(*descflowyp));
        }
        dup+=1; dvp+=1; maskp+=1; a11p+=1; a12p+=1; a22p+=1; b1p+=1; b2p+=1; 
        ix1p+=1; iy1p+=1; iz1p+=1; ixx1p+=1; ixy1p+=1; iyy1p+=1; ixz1p+=1; iyz1p+=1;
        ix2p+=1; iy2p+=1; iz2p+=1; ixx2p+=1; ixy2p+=1; iyy2p+=1; ixz2p+=1; iyz2p+=1;
        ix3p+=1; iy3p+=1; iz3p+=1; ixx3p+=1; ixy3p+=1; iyy3p+=1; ixz3p+=1; iyz3p+=1;
        uup+=1;vvp+=1;wxp+=1; wyp+=1;descflowxp+=1;descflowyp+=1;descweightp+=1;
    }
}

/* compute score for matches based on autocorrelation matrix and similarity in color/gradient */
void compute_desc_weight(const color_image_t* im1, const color_image_t* im2, const image_t* desc_x, const image_t *desc_y, image_t *desc_w, const convolution_t* deriv_autocor, const convolution_t *deriv_flow) {
    int w = im1->width, h = im1->height;
    if( w != desc_x->width || h != desc_x->height ) {
        fprintf(stderr, "only implemented if descriptors are given for the original resolution\n");
        exit(1);
    }
    color_image_t *sim1=color_image_new(w, h), *sim2=color_image_new(w,h), 
        *im1dx=color_image_new(w, h), *im1dy=color_image_new(w, h), *im2dx=color_image_new(w, h), *im2dy=color_image_new(w, h);
    image_t *dx2=image_new(w, h), *dxy=image_new(w, h), *dy2=image_new(w, h), *tmp=image_new(w, h);
    int i, j, o, fsize, s=desc_x->stride;
    float sigma_image = 0.8f;
    float autocor_sigma_matrix = 3.0f;
    float grad_weight = 1.0f;
    float flow_sigma_score = 50.0f;
    float mul_coef = 10.0f;
    // smooth images
    float *presmooth_filter = gaussian_filter(sigma_image, &fsize);
    convolution_t *presmooth_conv = convolution_new(fsize, presmooth_filter, 1);
    color_image_convolve_hv(sim1, im1, presmooth_conv, presmooth_conv);
    color_image_convolve_hv(sim2, im2, presmooth_conv, presmooth_conv);
    convolution_delete(presmooth_conv);
    free(presmooth_filter);
    // compute derivatives of first image for autocorrelation
    color_image_convolve_hv(im1dx, sim1, deriv_autocor, NULL);
    color_image_convolve_hv(im1dy, sim1, NULL, deriv_autocor);
    // compute autocorrelation matrix
    for( j=0 ; j<h ; j++) {
        for( i=0,o=j*s ; i<w ; i++,o++) {
	        dx2->data[o] = im1dx->c1[o]*im1dx->c1[o] + im1dx->c2[o]*im1dx->c2[o] + im1dx->c3[o]*im1dx->c3[o];
	        dxy->data[o] = im1dx->c1[o]*im1dy->c1[o] + im1dx->c2[o]*im1dy->c2[o] + im1dx->c3[o]*im1dy->c3[o];
	        dy2->data[o] = im1dy->c1[o]*im1dy->c1[o] + im1dy->c2[o]*im1dy->c2[o] + im1dy->c3[o]*im1dy->c3[o];
	    }
    }
    // integrate autocor matrix
    float *autocor_filter = gaussian_filter(autocor_sigma_matrix, &fsize);
    convolution_t *autocor_conv = convolution_new(fsize, autocor_filter, 1.0f);
    convolve_horiz(tmp, dx2, autocor_conv);
    convolve_vert(dx2, tmp, autocor_conv);
    convolve_horiz(tmp, dxy, autocor_conv);
    convolve_vert(dxy, tmp, autocor_conv);
    convolve_horiz(tmp, dy2, autocor_conv);
    convolve_vert(dy2, tmp, autocor_conv);
    convolution_delete(autocor_conv);
    free(autocor_filter);
    // compute minimal eigenvalues: it is done by computing (dx2+dy2)/2 - sqrt( ((dx2+dy2)/2)^2 + (dxy)^2 - dx^2*dy^2)
    for( j=0 ; j<h ; j++) {
        for( i=0,o=j*s ; i<w ; i++,o++) {
            float t = 0.5f * (dx2->data[o]+dy2->data[o]);
            float t2 = t*t + dxy->data[o]*dxy->data[o] - dx2->data[o]*dy2->data[o];
            tmp->data[o] = t - (t2<=0.0f?0.0f:sqrt(t2)); // may be negative due to floating points approximation
	    }
    }
    // compute derivatives of the images for flow weight
    color_image_convolve_hv(im1dx, sim1, deriv_flow, NULL);
    color_image_convolve_hv(im1dy, sim1, NULL, deriv_flow);
    color_image_convolve_hv(im2dx, sim2, deriv_flow, NULL);
    color_image_convolve_hv(im2dy, sim2, NULL, deriv_flow);
    // compute final weight
    float t = 1.0f/(flow_sigma_score*sqrt(2.0f*M_PI));
    float sigmascore2 = -0.5f/(flow_sigma_score*flow_sigma_score);
    for( j=0 ; j<h ; j++) {
        for( i=0,o=j*s ; i<w ; i++,o++) {
            if( desc_w->data[o] > 0.0f ) {
                int o2 = o+desc_y->data[o]*s+desc_x->data[o];
                float flowscore = 
                    abs(sim1->c1[o]-sim2->c1[o2]) + abs(sim1->c2[o]-sim2->c2[o2]) + abs(sim1->c3[o]-sim2->c3[o2])
                    + grad_weight * (
                        abs(im1dx->c1[o]-im2dx->c1[o2]) + abs(im1dx->c2[o]-im2dx->c2[o2]) + abs(im1dx->c3[o]-im2dx->c3[o2]) + 
                        abs(im1dy->c1[o]-im2dy->c1[o2]) + abs(im1dy->c2[o]-im2dy->c2[o2]) + abs(im1dy->c3[o]-im2dy->c3[o2]) );
            float t2 = tmp->data[o];
            t2 = t2<=0.0f?0.0f:sqrt(t2);
            desc_w->data[o] = mul_coef * t2 * t * exp( flowscore*flowscore*sigmascore2 );
            if(desc_w->data[o]<0.0f)  // may be negative due to floating points approximation
                desc_w->data[o] = 0.0f;
            }
        }
    }
    // free memory
    color_image_delete(sim1); color_image_delete(sim2);
    color_image_delete(im1dx); color_image_delete(im1dy); color_image_delete(im2dx); color_image_delete(im2dy);
    image_delete(dx2); image_delete(dxy); image_delete(dy2); image_delete(tmp); 
}

/* resize the descriptors to the new size using a weighted mean */
void descflow_resize(image_t *dst_flow_x, image_t *dst_flow_y, image_t *dst_weight, const image_t *src_flow_x, const image_t *src_flow_y, const image_t *src_weight){
    const int src_width = src_flow_x->width, src_height = src_flow_x->height, src_stride = src_flow_x->stride,
                dst_width = dst_flow_x->width, dst_height = dst_flow_x->height, dst_stride = dst_flow_x->stride;
    const float scale_x = ((float)dst_width-1)/((float)src_width-1), scale_y = ((float)dst_height-1)/((float)src_height-1);
    image_erase(dst_flow_x); image_erase(dst_flow_y); image_erase(dst_weight);
    int j;
    for( j=0 ; j<src_height ; j++){
        const float yy = ((float)j)*scale_y;
        const float yyf = floor(yy);
        const float dy = yy-yyf;
        const int y1 = MINMAX( (int) yyf   , dst_height);
        const int y2 = MINMAX( (int) yyf+1 , dst_height);
        int i;
        for( i=0 ; i<src_width ; i++ ){
            const float weight = src_weight->data[j*src_stride+i];
	        if( weight<0.0000000001f ) continue;
            const float xx = ((float)i)*scale_x;
	        const float xxf = floor(xx);
	        const float dx = xx-xxf;
	        const int x1 = MINMAX( (int) xxf   , dst_width);
	        const int x2 = MINMAX( (int) xxf+1 , dst_width);
	        float weightxy, newweight;
	        if( dx ){
	            if( dy ){
		            weightxy = weight*dx*dy;
		            newweight = dst_weight->data[y2*dst_stride+x2] + weightxy;
		            dst_flow_x->data[y2*dst_stride+x2] = (dst_flow_x->data[y2*dst_stride+x2]*dst_weight->data[y2*dst_stride+x2] + src_flow_x->data[j*src_stride+i]*weightxy*scale_x)/newweight;
		            dst_flow_y->data[y2*dst_stride+x2] = (dst_flow_y->data[y2*dst_stride+x2]*dst_weight->data[y2*dst_stride+x2] + src_flow_y->data[j*src_stride+i]*weightxy*scale_y)/newweight;
		            dst_weight->data[y2*dst_stride+x2] = newweight;
		        }
	            weightxy = weight*dx*(1.0f-dy);
	            newweight = dst_weight->data[y1*dst_stride+x2] + weightxy;
	            dst_flow_x->data[y1*dst_stride+x2] = (dst_flow_x->data[y1*dst_stride+x2]*dst_weight->data[y1*dst_stride+x2] + src_flow_x->data[j*src_stride+i]*weightxy*scale_x)/newweight;
	            dst_flow_y->data[y1*dst_stride+x2] = (dst_flow_y->data[y1*dst_stride+x2]*dst_weight->data[y1*dst_stride+x2] + src_flow_y->data[j*src_stride+i]*weightxy*scale_y)/newweight;
	            dst_weight->data[y1*dst_stride+x2] = newweight;
            }
	        if( dy ) {
                weightxy = weight*(1.0f-dx)*dy;
                newweight = dst_weight->data[y2*dst_stride+x1] + weightxy;
                dst_flow_x->data[y2*dst_stride+x1] = (dst_flow_x->data[y2*dst_stride+x1]*dst_weight->data[y2*dst_stride+x1] + src_flow_x->data[j*src_stride+i]*weightxy*scale_x)/newweight;
                dst_flow_y->data[y2*dst_stride+x1] = (dst_flow_y->data[y2*dst_stride+x1]*dst_weight->data[y2*dst_stride+x1] + src_flow_y->data[j*src_stride+i]*weightxy*scale_y)/newweight;
                dst_weight->data[y2*dst_stride+x1] = newweight;
	        }
	        weightxy = weight*(1.0f-dx)*(1.0f-dy);
	        newweight = dst_weight->data[y1*dst_stride+x1] + weightxy;
	        dst_flow_x->data[y1*dst_stride+x1] = (dst_flow_x->data[y1*dst_stride+x1]*dst_weight->data[y1*dst_stride+x1] + src_flow_x->data[j*src_stride+i]*weightxy*scale_x)/newweight;
	        dst_flow_y->data[y1*dst_stride+x1] = (dst_flow_y->data[y1*dst_stride+x1]*dst_weight->data[y1*dst_stride+x1] + src_flow_y->data[j*src_stride+i]*weightxy*scale_y)/newweight;
	        dst_weight->data[y1*dst_stride+x1] = newweight;
	    }
    }
}

/* resize the descriptors to the new size using a nearest neighbor method while keeping the descriptor with the higher weight at the end */
void descflow_resize_nn(image_t *dst_flow_x, image_t *dst_flow_y, image_t *dst_weight, const image_t *src_flow_x, const image_t *src_flow_y, const image_t *src_weight){
    const int src_width = src_flow_x->width, src_height = src_flow_x->height, src_stride = src_flow_x->stride,
                dst_width = dst_flow_x->width, dst_height = dst_flow_x->height, dst_stride = dst_flow_x->stride;
    const float scale_x = ((float)dst_width-1)/((float)src_width-1), scale_y = ((float)dst_height-1)/((float)src_height-1);
    image_erase(dst_flow_x); image_erase(dst_flow_y); image_erase(dst_weight);
    int j;
    for( j=0 ; j<src_height ; j++){
        const float yy = ((float)j)*scale_y;
        const int y = (int) 0.5f+yy; // equivalent to round(yy)
        int i;
        for( i=0 ; i<src_width ; i++ ){
	        const float weight = src_weight->data[j*src_stride+i];
	        if( !weight )
	            continue;
	        const float xx = ((float)i)*scale_x;
	        const int x = (int) 0.5f+xx; // equivalent to round(xx)
	        if( dst_weight->data[y*dst_stride+x] < weight ){
	            dst_weight->data[y*dst_stride+x] = weight;
	            dst_flow_x->data[y*dst_stride+x] = src_flow_x->data[j*src_stride+i]*scale_x;
	            dst_flow_y->data[y*dst_stride+x] = src_flow_y->data[j*src_stride+i]*scale_y;
	        }
	    }
    }
}
