#ifndef COL2IM_HPP
#define COL2IM_HPP
#include "common_header.hpp"

void col2im_cpu(float* data_col,
        int channels, int height, int width,
        int ksize, int stride, int pad, float* data_im);

#ifdef GPU
void col2im_ongpu(float *data_col,
        int channels, int height, int width,
        int ksize, int stride, int pad, float *data_im);
#endif
#endif
