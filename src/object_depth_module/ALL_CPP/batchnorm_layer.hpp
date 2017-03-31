#ifndef BATCHNORM_LAYER_HPP
#define BATCHNORM_LAYER_HPP

#include "image.hpp"
#include "layer.hpp"
#include "network.hpp"
#include "common_header.hpp"

layer make_batchnorm_layer(int batch, int w, int h, int c);
void forward_batchnorm_layer(layer l, network_state state);
void backward_batchnorm_layer(layer l, network_state state);

#ifdef GPU
void forward_batchnorm_layer_gpu(layer l, network_state state);
void backward_batchnorm_layer_gpu(layer l, network_state state);
void pull_batchnorm_layer(layer l);
void push_batchnorm_layer(layer l);
#endif

#endif
