
#ifndef RNN_LAYER_HPP
#define RNN_LAYER_HPP

#include "activations.hpp"
#include "layer.hpp"
#include "network.hpp"
#include "common_header.hpp"
#define USET

layer make_rnn_layer(int batch, int inputs, int hidden, int outputs, int steps, ACTIVATION activation, int batch_normalize, int log);

void forward_rnn_layer(layer l, network_state state);
void backward_rnn_layer(layer l, network_state state);
void update_rnn_layer(layer l, int batch, float learning_rate, float momentum, float decay);

#ifdef GPU
void forward_rnn_layer_gpu(layer l, network_state state);
void backward_rnn_layer_gpu(layer l, network_state state);
void update_rnn_layer_gpu(layer l, int batch, float learning_rate, float momentum, float decay);
void push_rnn_layer(layer l);
void pull_rnn_layer(layer l);
#endif

#endif

