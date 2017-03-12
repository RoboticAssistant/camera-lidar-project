#ifndef OBJECT_DEPTH_INTERFACE_HPP
#define OBJECT_DEPTH_INTERFACE_HPP

// System libraries
#include <iostream>

// ZED libraries
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

using namespace std;

class object_depth_interface
{
public:
    object_depth_interface(bool live_flag, string svo_file_path, int depth_requested);
    int initialize_ZED_camera(std::unique_ptr<sl::zed::Camera> zed, bool is_live, string svo_path);
    int depth_reception();

private:
    std::unique_ptr<sl::zed::Camera> zed;
    bool is_live;
    string svo_path;
    int depth_clamp;
};

#endif // OBJECT_DEPTH_INTERFACE_HPP
