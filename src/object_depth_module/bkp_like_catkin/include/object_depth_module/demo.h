#ifndef DEMO
#define DEMO

#include "image.h"
#include "common_header.h"

void demo(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int frame_skip, char *prefix, float hier_thresh);

#endif