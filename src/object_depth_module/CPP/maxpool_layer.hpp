#ifndef MAXPOOL_LAYER_HPP
#define MAXPOOL_LAYER_HPP

#include "common_header.hpp"
#include "image.hpp"
#include "cuda.hpp"
#include "layer.hpp"
#include "network.hpp"

typedef layer maxpool_layer;

image get_maxpool_image(maxpool_layer l);
maxpool_layer make_maxpool_layer(int batch, int h, int w, int c, int size, int stride, int padding);
void resize_maxpool_layer(maxpool_layer *l, int w, int h);
void forward_maxpool_layer(const maxpool_layer l, network_state state);
void backward_maxpool_layer(const maxpool_layer l, network_state state);

#ifdef GPU
void forward_maxpool_layer_gpu(maxpool_layer l, network_state state);
void backward_maxpool_layer_gpu(maxpool_layer l, network_state state);
#endif

#endif

