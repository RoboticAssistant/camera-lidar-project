
#ifndef GRU_LAYER_HPP
#define GRU_LAYER_HPP

#include "activations.hpp"
#include "layer.hpp"
#include "network.hpp"
#include "common_header.hpp"

layer make_gru_layer(int batch, int inputs, int outputs, int steps, int batch_normalize);

void forward_gru_layer(layer l, network_state state);
void backward_gru_layer(layer l, network_state state);
void update_gru_layer(layer l, int batch, float learning_rate, float momentum, float decay);

#ifdef GPU
void forward_gru_layer_gpu(layer l, network_state state);
void backward_gru_layer_gpu(layer l, network_state state);
void update_gru_layer_gpu(layer l, int batch, float learning_rate, float momentum, float decay);
void push_gru_layer(layer l);
void pull_gru_layer(layer l);
#endif

#endif

