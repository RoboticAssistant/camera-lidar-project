
#ifndef CRNN_LAYER_HPP
#define CRNN_LAYER_HPP

#include "activations.hpp"
#include "layer.hpp"
#include "network.hpp"
#include "common_header.hpp"

layer make_crnn_layer(int batch, int h, int w, int c, int hidden_filters, int output_filters, int steps, ACTIVATION activation, int batch_normalize);

void forward_crnn_layer(layer l, network_state state);
void backward_crnn_layer(layer l, network_state state);
void update_crnn_layer(layer l, int batch, float learning_rate, float momentum, float decay);

#ifdef GPU
void forward_crnn_layer_gpu(layer l, network_state state);
void backward_crnn_layer_gpu(layer l, network_state state);
void update_crnn_layer_gpu(layer l, int batch, float learning_rate, float momentum, float decay);
void push_crnn_layer(layer l);
void pull_crnn_layer(layer l);
#endif

#endif

