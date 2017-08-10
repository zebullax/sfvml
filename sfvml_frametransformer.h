#ifndef SFVML_FRAMETRANSFORMER
#define SFVML_FRAMETRANSFORMER

// opencv2
#include "opencv2/opencv.hpp"

namespace sfvml {

double compareHistogram(const cv::Mat& frameA, const cv::Mat& frameB);


} // sfvml::

#endif
