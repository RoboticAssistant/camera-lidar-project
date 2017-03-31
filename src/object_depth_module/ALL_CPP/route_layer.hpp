#ifndef ROUTE_LAYER_HPP
#define ROUTE_LAYER_HPP
#include "network.hpp"
#include "layer.hpp"
#include "common_header.hpp"

typedef layer route_layer;

route_layer make_route_layer(int batch, int n, int *input_layers, int *input_size);
void forward_route_layer(const route_layer l, network_state state);
void backward_route_layer(const route_layer l, network_state state);
void resize_route_layer(route_layer *l, network *net);

#ifdef GPU
void forward_route_layer_gpu(const route_layer l, network_state state);
void backward_route_layer_gpu(const route_layer l, network_state state);
#endif

#endif
