#ifndef SFVML_TYPE
#define SFVML_TYPE

// opencv
#include "opencv2/opencv.hpp"

namespace sfvml {

using Pixel = cv::Point3_<uint8_t>;

struct Position { double x; 
                  double y; };

} // sfvml::

#endif
