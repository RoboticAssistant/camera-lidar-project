#include <iostream>
#include <memory>
#include <thread>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// OpenCv Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// ZED Libraries
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <object_depth_module/depth_computations.hpp>

using namespace sl::zed;
using namespace std;
using namespace cv;





