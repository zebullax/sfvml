#ifndef SFVML_FRAMETRANSFORMER
#define SFVML_FRAMETRANSFORMER

// sfvml
#include "sfvml_type.h"
// opencv2
#include "opencv2/opencv.hpp"

namespace Sfvml {

// FIXME should not be here, this does not transform anything
double compareHistogram(const cv::Mat& frameA, const cv::Mat& frameB);

// Remove (= set to black) the grey pixels, where we expect
// the background to be "The grid", frame should be a row vector
// frame -> Frame read through cv to be cleaned
void removeGreyPixels(cv::Mat *frame);

// Remove the black pixel on a H S V sorted frame
// We expect frame to be a row vector
void removeBlackPixels(cv::Mat *frame);

void sortFramePixels(cv::Mat *frame);

} // Sfvml::

#endif
