#ifndef SHORTCUT_LAYER_HPP
#define SHORTCUT_LAYER_HPP

#include "layer.hpp"
#include "network.hpp"
#include "common_header.hpp"

layer make_shortcut_layer(int batch, int index, int w, int h, int c, int w2, int h2, int c2);
void forward_shortcut_layer(const layer l, network_state state);
void backward_shortcut_layer(const layer l, network_state state);

#ifdef GPU
void forward_shortcut_layer_gpu(const layer l, network_state state);
void backward_shortcut_layer_gpu(const layer l, network_state state);
#endif

#endif
