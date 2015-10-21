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
#include <png.h>
#include <stdlib.h>
#include <stdio.h>
#include <jpeglib.h>
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "io.h"

/* FLOW */

/* read a flow file and returns a pointer with two images containing the flow along x and y axis */
image_t** readFlowFile(const char* filename){
    FILE *fid = fopen(filename, "rb");
    if (fid == 0){
        fprintf(stderr,"readFlow() error: could not open file  %s\n",filename);
        exit(1);
    }
    float help;
    fread(&help,sizeof(float),1,fid);
    int aXSize,aYSize;
    fread(&aXSize,sizeof(int),1,fid);
    fread(&aYSize,sizeof(int),1,fid);
    image_t** flow = (image_t**) malloc(sizeof(image_t*)*2);
    flow[0] = image_new(aXSize, aYSize);
    flow[1] = image_new(aXSize, aYSize);
    int x,y;
    for (y = 0; y < aYSize; y++)
        for (x = 0; x < aXSize ; x++) {
            fread(&(flow[0]->data[y*flow[0]->stride+x]),sizeof(float),1,fid);
            fread(&(flow[1]->data[y*flow[0]->stride+x]),sizeof(float),1,fid);
    }
    fclose(fid);
    return flow;
}

/* write a flow to a file */
void writeFlowFile(const char *filename, const image_t *flowx, const image_t *flowy){
    FILE *stream = fopen(filename, "wb");
    if (stream == 0){
        fprintf(stderr, "Error while opening %s\n",filename);
        exit(1);
    }
    const float help=202021.25;
    fwrite(&help,sizeof(float),1,stream);
    const int aXSize = flowx->width, aYSize = flowx->height;
    fwrite(&aXSize,sizeof(int),1,stream);
    fwrite(&aYSize,sizeof(int),1,stream);
    int y,x;
    for (y = 0; y < aYSize ; y++)
        for (x = 0; x < aXSize ; x++) {
	        fwrite(&flowx->data[y*flowx->stride+x],sizeof(float),1,stream);
	        fwrite(&flowy->data[y*flowy->stride+x],sizeof(float),1,stream);
        }
    fclose(stream);
}

/* IMAGE */

// PPM

typedef struct{
    int magic;
    int width;
    int height;
    int pixmax;
} ppm_hdr_t;

static void get_magic(FILE *fp, ppm_hdr_t *ppm_hdr){
    char str[1024];
    fgets(str, 1024, fp);
    if(str[0] == 'P' && (str[1] <= '6' || str[1] >= '1')){
        ppm_hdr->magic = str[1] - '0';
    }
}

static int skip_comment(FILE *fp){
    char c;
    do{
        c = (char) fgetc(fp);
    } 
	while (c == ' ' || c == '\t' || c == '\n');
    if(c == '#'){
        do {
            c = (char) fgetc(fp);

        } while(c != 0x0A);
        return 1;
    }else{
        ungetc(c, fp);
    }
    return 0;
}

/*----------------------------------------------------------------------------*/

static void skip_comments(FILE *fp){
    while(skip_comment(fp));
}

/*----------------------------------------------------------------------------*/

static int get_image_size(FILE *fp, ppm_hdr_t *ppm_hdr){
    skip_comments(fp);
    if(fscanf(fp, "%d %d", &ppm_hdr->width, &ppm_hdr->height) != 2){
        fprintf(stderr, "Warning: PGM --> File currupted\n");
        return 0;
    }
    return 1;
}

/*----------------------------------------------------------------------------*/

static int get_pixmax(FILE *fp, ppm_hdr_t *ppm_hdr){
    skip_comments(fp);
    ppm_hdr->pixmax = 1;
    if(ppm_hdr->magic == 2 || ppm_hdr->magic == 3 || ppm_hdr->magic == 5 || ppm_hdr->magic == 6){
        if(fscanf(fp, "%d", &ppm_hdr->pixmax) != 1){
            fprintf(stderr, "Warning: PGM --> pixmax not valid\n");
            return 0;
        }
    }
    fgetc(fp);
    return 1;
}

/*----------------------------------------------------------------------------*/

static int get_ppm_hdr(FILE *fp, ppm_hdr_t *ppm_hdr){
    get_magic(fp, ppm_hdr);
    if(!get_image_size(fp, ppm_hdr)){
        return 0;
    }
    if(!get_pixmax(fp, ppm_hdr)){
        return 0;
    }
    return 1;
}

static void raw_read_color(FILE *fp, color_image_t *image){
    int j;
    for( j=0 ; j<image->height ; j++){
        int o = j*image->stride, i;
        for( i=0 ; i<image->width ; i++,o++ ){
            image->c1[o] = (float) fgetc(fp);
            image->c2[o] = (float) fgetc(fp);
            image->c3[o] = (float) fgetc(fp);
        }
    }
}

color_image_t *color_image_pnm_load(FILE *fp){
    color_image_t *image = NULL;
    ppm_hdr_t ppm_hdr;
    if(!get_ppm_hdr(fp, &ppm_hdr)) 	{
        return NULL;
    }
    switch(ppm_hdr.magic)    {
        case 1: /* PBM ASCII */
        case 2: /* PGM ASCII */
        case 3: /* PPM ASCII */
        case 4: /* PBM RAW */
        case 5: /* PGM RAW */
            fprintf(stderr, "color_image_pnm_load: only PPM raw with maxval 255 supported\n");            
            break;
        case 6: /* PPM RAW */
            image = color_image_new(ppm_hdr.width, ppm_hdr.height);
            raw_read_color(fp, image);
            break;
    }
    return image;
}

// JPG

color_image_t *color_image_jpeg_load(FILE *fp){
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPARRAY buffer;
    int row_stride;
    int index = 0;
    color_image_t *image = NULL;
    float *r_p, *g_p, *b_p;
    JSAMPROW buffer_p;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, fp);
    jpeg_read_header(&cinfo, TRUE);
    cinfo.out_color_space = JCS_RGB;
    cinfo.quantize_colors = FALSE;
    image = color_image_new(cinfo.image_width, cinfo.image_height);
    if(image == NULL){
        return NULL;
    }
    jpeg_start_decompress(&cinfo);
    row_stride = cinfo.output_width * cinfo.output_components;
    buffer = (*cinfo.mem->alloc_sarray)
        ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);

    r_p = image->c1;
    g_p = image->c2;
    b_p = image->c3;

    const int incr_line = image->stride-image->width;

    while (cinfo.output_scanline < cinfo.output_height){
        jpeg_read_scanlines(&cinfo, buffer, 1);
        buffer_p = buffer[0];
        index = cinfo.output_width;
        while(index--){
            *r_p++ = (float) *buffer_p++;
            *g_p++ = (float) *buffer_p++;
            *b_p++ = (float) *buffer_p++;
        }
        r_p += incr_line; g_p += incr_line; b_p += incr_line;
    }
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return image;
}

// PNG

color_image_t *color_image_png_load( FILE* fp, const char* file_name ){
    // read the header
    png_byte header[8];
    fread(header, 1, 8, fp);
    
    if (png_sig_cmp(header, 0, 8)){
        fprintf(stderr, "error: %s is not a PNG.\n", file_name);
        fclose(fp);
        return 0;
    }
    
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr){
        fprintf(stderr, "error: png_create_read_struct returned 0.\n");
        fclose(fp);
        return 0;
    }
    
    // create png info struct
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr){
        fprintf(stderr, "error: png_create_info_struct returned 0.\n");
        png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
        fclose(fp);
        return 0;
    }
    
    // create png info struct
    png_infop end_info = png_create_info_struct(png_ptr);
    if (!end_info){
        fprintf(stderr, "error: png_create_info_struct returned 0.\n");
        png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp) NULL);
        fclose(fp);
        return 0;
    }

    // the code in this if statement gets called if libpng encounters an error
    if (setjmp(png_jmpbuf(png_ptr))) {
        fprintf(stderr, "error from libpng\n");
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        fclose(fp);
        return 0;
    }

    // init png reading
    png_init_io(png_ptr, fp);

    // let libpng know you already read the first 8 bytes
    png_set_sig_bytes(png_ptr, 8);

    // read all the info up to the image data
    png_read_info(png_ptr, info_ptr);

    // variables to pass to get info
    int bit_depth, color_type;
    png_uint_32 temp_width, temp_height;

    // get info about png
    png_get_IHDR(png_ptr, info_ptr, &temp_width, &temp_height, &bit_depth, &color_type,
        NULL, NULL, NULL);

    // Update the png info struct.
    png_read_update_info(png_ptr, info_ptr);

    // Row size in bytes.
    int rowbytes = png_get_rowbytes(png_ptr, info_ptr);

    // Allocate the image_data as a big block, to be given to opengl
    png_byte * image_data;
    image_data = (png_byte*) malloc(sizeof(png_byte)*rowbytes*temp_height);
    assert(image_data!=NULL);

    // row_pointers is for pointing to image_data for reading the png with libpng
    png_bytep * row_pointers = (png_bytep*) malloc(sizeof(png_bytep)*temp_height);
    assert(row_pointers!=NULL);

    // set the individual row_pointers to point at the correct offsets of image_data
    unsigned int i;
    for (i = 0; i <temp_height; i++)
        row_pointers[i] = image_data + i * rowbytes;

    // read the png into image_data through row_pointers
    png_read_image(png_ptr, row_pointers);
    
    // copy into color image
    color_image_t* image = color_image_new(temp_width,temp_height);
    if( color_type==0 ) {
      assert((unsigned)rowbytes == temp_width || !"error: not a proper gray png image");
      for(i=0; i<temp_height; i++){
        int j;
        for(j=0; j<temp_width; j++)
            image->c1[i*image->stride+j] = image->c2[i*image->stride+j] = image->c3[i*image->stride+j] = image_data[i*image->width+j];
      }
    } else if( color_type == 2 ) {
      assert((unsigned)rowbytes == 3*temp_width || !"error: not a proper color png image");
      for(i=0; i<temp_height; i++) {
        int j;
        for(j=0; j<temp_width; j++){
          image->c1[i*image->stride+j] = image_data[3*(i*image->width+j)+0];
          image->c2[i*image->stride+j] = image_data[3*(i*image->width+j)+1];
          image->c3[i*image->stride+j] = image_data[3*(i*image->width+j)+2];
        }
      }
    } else
      assert(!"error: unknown PNG color type" );
    
    // clean up
    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
    free(row_pointers);
    
    return image;
}

// GENERAL LOAD

/* load a color image from a file */
color_image_t *color_image_load(const char *fname){
    FILE *fp;
    char magic[2];
    unsigned short *magic_short = (unsigned short *) magic;
    color_image_t *image = NULL;
    if((fp = fopen(fname, "rb")) == NULL){
        fprintf(stderr, "Error in color_image_load() - can not open file `%s' !\n", fname);
        exit(1);
    }
    fread(magic, sizeof(char), 2, fp);
    rewind(fp);
    if(magic_short[0] == 0xd8ff){
        image = color_image_jpeg_load(fp);
    } else if(magic[0]=='P' && (magic[1]=='6' || magic[1]=='5')){ /* PPM raw */
        image = color_image_pnm_load(fp);
    } else if( magic[0]==-119 && magic[1]=='P' ) {
      image = color_image_png_load( fp, fname );
    } else{
        fprintf(stderr, "Error in color_image_load(%s) - image format not supported, can only read jpg or ppm\n",fname);
        exit(1);
    }
    fclose(fp);
    return image;
}

// PARSE OPTIONS

/* parse options */
void usage(const int language){
    printf("usage:\n");
    switch(language){
        case EXE_OPTIONS:
            printf("./deepflow2 image1 image2 outputfile [options] \n");
            printf("Compute the flow between two images and store it into a flo file.\n");
            printf("Images must be in JPG, PNG or PPM format.\n");
            break;
        case MATLAB_OPTIONS:
            printf("flow = deepflow2(image1, image2, match, options) \n");
            printf("Compute the flow between two images, eventually using given matches.\n");
            printf("Images are HxWx3 single matrices.\n");
            printf("Match is an optional argument ([] by default), where each row starts by x1 y1 x2 y2.\n");
            printf("Options is an optional string argument ('' by default), to set the options.\n");
            printf("The function returns the optical flow as a HxWx2 single matrix.\n");
            break;
        case PYTHON_OPTIONS:
            printf("flow = deepflow2.deepflow2(image1, image2, match=None, options='')\n");
            printf("Compute the flow between two images, eventually using given matches.\n");
            printf("Images must be HxWx3 numpy arrays (convert to float32).\n");
            printf("Match is an optional numpy array argument (None by default, ie no input match), where each row starts by x1 y1 x2 y2.\n");
            printf("Options is an optional string argument ('' by default), to set the options.\n");
            printf("The function returns the optical flow as a HxWx2 numpy array.\n");
            break;
    }
    printf("\n");
    printf("options:\n"); 
    printf("    -h, --help                                               print this message\n");
    printf("    -a, -alpha              <float>(12.0)                    weight of smoothness terms\n");
    printf("    -b, -beta               <float>(300.0)                   weight of descriptor matching\n");
    printf("    -g, -gamma              <float>(3.0)                     weight of gradient constancy assumption\n");
    printf("    -d, -delta              <float>(2.0)                     weight of color constancy assumption\n");
    printf("    -s, -sigma              <float>(0.8)                     standard deviation of Gaussian presmoothing kernel\n");
    printf("    -e, -eta                <float>(0.95)                    ratio factor for coarse-to-fine scheme\n");
    printf("    -minsize                <interger>(25)                   size of the coarsest level\n");
    printf("    -inner                  <integer>(5)                     number of inner fixed point iterations\n");
    printf("    -iter                   <integer>(25)                    number of iterations for the solver\n");
    printf("    -soromega               <float>(1.6)                     omega parameter of the sor method\n");
    printf("    -bk                     <float>(0.0)                     use decreasing beta i.e. beta(k) = beta*( k / kmax )^betak, if 0, a last iteration is done with beta=0\n");
    printf("\n");
    if( language==EXE_OPTIONS){
        printf("    -match                                                   '-match filename' reads matches from a file and '-match' from stdin. Each line is the match with the first four numbers being x1 y1 x2 y2.\n");
        printf("\n");
    }
    printf("    -sintel                                                  set the parameters to the one used in the arXiv paper for MPI-Sintel dataset\n");
    printf("    -middlebury                                              set the parameters to the one used in the arXiv paper for middlebury dataset\n");
    printf("    -kitti                                                   set the parameters to the one used in the arXiv paper for KITTI dataset\n");
    printf("\n");
}

void require_argument(const char *arg){
    fprintf(stderr, "Require an argument after option %s\n", arg);
    exit(1);
}

/* parse options and eventually return the matches given as argument */
image_t** parse_options(optical_flow_params_t* params, int argc, char **argv, const int language, const int width, const int height){
    image_t **matches = NULL;
    if(language==EXE_OPTIONS){
        matches = (image_t**) malloc(sizeof(image_t*)*3);
        matches[0] = NULL; matches[1] = NULL; matches[2] = NULL;
    }
    int current_arg = 0;
    while(1){	
        if( current_arg >= argc) break;
        if(!strcmp(argv[current_arg],"-h") || !strcmp(argv[current_arg],"--help") ){
            usage(language);
            if( language == EXE_OPTIONS) exit(1);
	    }else if(!strcmp(argv[current_arg],"-a") || !strcmp(argv[current_arg],"-alpha") ){
            current_arg++;
            if(current_arg >= argc) require_argument("alpha");
            float alpha = atof(argv[current_arg++]);
            if(alpha<0){
                fprintf(stderr,"Alpha argument cannot be negative\n");
                exit(1);
            }
            params->alpha = alpha;
	    }else if(!strcmp(argv[current_arg],"-b") || !strcmp(argv[current_arg],"-beta") ){
            current_arg++;
            if(current_arg >= argc) require_argument("beta");
            float beta = atof(argv[current_arg++]);
            if(beta<0){
                fprintf(stderr,"Beta argument cannot be negative\n");
                exit(1);
            }
	        params->beta = beta;
	    }else if(!strcmp(argv[current_arg],"-g") || !strcmp(argv[current_arg],"-gamma") ){
            current_arg++;
            if(current_arg >= argc) require_argument("gamma");
            float gamma = atof(argv[current_arg++]);
            if(gamma<0){
                fprintf(stderr,"Gamma argument cannot be negative\n");
                exit(1);
            }
            params->gamma = gamma;
	    }else if(!strcmp(argv[current_arg],"-d") || !strcmp(argv[current_arg],"-delta") ){
            current_arg++;
            if(current_arg >= argc) require_argument("delta");
            float delta = atof(argv[current_arg++]);
            if(delta<0) {
                fprintf(stderr,"Delta argument cannot be negative\n");
                exit(1);
            }
            params->delta = delta;
	    }else if(!strcmp(argv[current_arg],"-s") || !strcmp(argv[current_arg],"-sigma") ){
            current_arg++;
            if(current_arg >= argc) require_argument("sigma");
            float sigma = atof(argv[current_arg++]);
            if(sigma<0){
                fprintf(stderr,"Sigma argument is negative\n");
                exit(1);
            }
            params->sigma = sigma;
	    }else if(!strcmp(argv[current_arg],"-bk"))	{
            current_arg++;
            if(current_arg >= argc) require_argument("bk");
            float betak = atof(argv[current_arg++]);
            if(betak<0.0f){
                fprintf(stderr,"Bk argument must be positive\n");
                exit(1);
            }
            params->bk = betak;
	    }else if(!strcmp(argv[current_arg],"-e") || !strcmp(argv[current_arg],"-eta") ){
            current_arg++;
            if(current_arg >= argc) require_argument("eta");
            float eta = atof(argv[current_arg++]);
            if(eta<0.25 || eta>0.98){
                fprintf(stderr,"Eta argument has to be between 0.25 and 0.98\n");
                exit(1);
            }
            params->eta = eta;
	    }else if( !strcmp(argv[current_arg],"-minsize") ){
            current_arg++;
            if(current_arg >= argc) require_argument("minsize");
            int minsize = atoi(argv[current_arg++]);
            if(minsize < 10){
                fprintf(stderr,"Minsize argument has to be higher than 10\n");
                exit(1);
            }
            params->min_size = minsize;
	    }else if(!strcmp(argv[current_arg],"-inner") ){
            current_arg++;
            if(current_arg >= argc) require_argument("inner");
            int inner = atoi(argv[current_arg++]);
            if(inner<=0){
                fprintf(stderr,"Inner argument must be strictly positive\n");
                exit(1);
            }
            params->n_inner_iteration = inner;
	    }else if(!strcmp(argv[current_arg],"-iter") ){
            current_arg++;
            if(current_arg >= argc) require_argument("iter");
            int iter = atoi(argv[current_arg++]);
            if(iter<=0){
                fprintf(stderr,"Iter argument must be strictly positive\n");
                exit(1);
            }
            params->n_solver_iteration = iter;
	    }else if( language==EXE_OPTIONS && !strcmp(argv[current_arg],"-match")){
	        current_arg++;
	        image_delete(matches[0]); image_delete(matches[1]); image_delete(matches[2]);
            image_t *match_x = image_new(width, height), *match_y = image_new(width, height), *match_z = image_new(width, height); 
            image_erase(match_x); image_erase(match_y); image_erase(match_z);
            FILE *fid = stdin;
	        if( current_arg<argc && argv[current_arg][0] != '-'){ // if the next argument exists and is not an option, ie must be a filename
	            fid = fopen(argv[current_arg++], "r");
	            if(fid==NULL){
		            fprintf(stderr, "Cannot read matches from file %s", argv[current_arg-1]);
		            exit(1);
		        }
	        }
	        float x1, x2, y1, y2;
	        while(!feof(fid) && fscanf(fid, "%f %f %f %f%*[^\n]", &x1, &y1, &x2, &y2)==4){
	            if( x1<0 || y1<0 || x2<0 || y2<0 || x1>=width || y1>=height || x2>=width || y2>=height){
		            fprintf(stderr, "Warning: match out of bound: %f %f -> %f %f\n", x1, y1, x2, y2);
		            x1 = MINMAX(x1,width);
		            x2 = MINMAX(x2,width);
		            y1 = MINMAX(y1,height);
		            y2 = MINMAX(y2,height);
		        }
		        int pos = (int) (y1*match_x->stride+x1);
	            match_x->data[ pos ] = x2-x1;
	            match_y->data[ pos ] = y2-y1;
	            match_z->data[ pos ] = 1.0f; // will be computed later
	        }
	        matches[0] = match_x;
	        matches[1] = match_y;
	        matches[2] = match_z;
	    }else if ( !strcmp(argv[current_arg],"-sintel") ){
		    current_arg++;
	        optical_flow_params_sintel(params);
	    }else if ( !strcmp(argv[current_arg],"-middlebury") ){
		    current_arg++;
	    optical_flow_params_middlebury(params);
	    }else if ( !strcmp(argv[current_arg],"-kitti") ){
		    current_arg++;
	        optical_flow_params_kitti(params);
	    }else{
            if(argv[current_arg][0] == '-'){
	            fprintf(stderr,"Unknow options %s\n",argv[current_arg]);
            }else{
                fprintf(stderr,"Error while reading options, %s\n",argv[current_arg]);
    	    }
    	    exit(1);
        }
    }
    return matches;
}
