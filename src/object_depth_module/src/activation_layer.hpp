#ifndef ACTIVATION_LAYER_HPP
#define ACTIVATION_LAYER_HPP

#include "activations.hpp"
#include "layer.hpp"
#include "network.hpp"
#include "common_header.hpp"

layer make_activation_layer(int batch, int inputs, ACTIVATION activation);

void forward_activation_layer(layer l, network_state state);
void backward_activation_layer(layer l, network_state state);

#ifdef GPU
void forward_activation_layer_gpu(layer l, network_state state);
void backward_activation_layer_gpu(layer l, network_state state);
#endif

#endif

