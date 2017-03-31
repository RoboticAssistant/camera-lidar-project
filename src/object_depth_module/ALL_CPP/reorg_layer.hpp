#ifndef REORG_LAYER_HPP
#define REORG_LAYER_HPP

#include "image.hpp"
#include "cuda.hpp"
#include "layer.hpp"
#include "network.hpp"
#include "common_header.hpp"

layer make_reorg_layer(int batch, int h, int w, int c, int stride, int reverse);
void resize_reorg_layer(layer *l, int w, int h);
void forward_reorg_layer(const layer l, network_state state);
void backward_reorg_layer(const layer l, network_state state);

#ifdef GPU
void forward_reorg_layer_gpu(layer l, network_state state);
void backward_reorg_layer_gpu(layer l, network_state state);
#endif

#endif

