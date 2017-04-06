#ifndef DETECTION_LAYER_HPP
#define DETECTION_LAYER_HPP

#include "layer.hpp"
#include "network.hpp"
#include "common_header.hpp"

typedef layer detection_layer;

detection_layer make_detection_layer(int batch, int inputs, int n, int size, int classes, int coords, int rescore);
void forward_detection_layer(const detection_layer l, network_state state);
void backward_detection_layer(const detection_layer l, network_state state);
void get_detection_boxes(layer l, int w, int h, float thresh, float **probs, box *boxes, int only_objectness);

#ifdef GPU
void forward_detection_layer_gpu(const detection_layer l, network_state state);
void backward_detection_layer_gpu(detection_layer l, network_state state);
#endif

#endif
