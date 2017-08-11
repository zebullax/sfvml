#ifndef SFVML_FRAMETRANSFORMER
#define SFVML_FRAMETRANSFORMER

// opencv2
#include "opencv2/opencv.hpp"

namespace Sfvml            {
namespace FrameTransformer {

    // FIXME should not be here, this does not transform anything
double compareHistogram(const cv::Mat& frameA, const cv::Mat& frameB);

// Remove (= set to black) the grey pixels, where we expect
// the background to be "The grid"
// frame -> Frame read through cv to be cleaned
void removeGreyPixels(cv::Mat *frame);

void sortFramePixels(cv::Mat *frame);

} // FrameTransformer::
} // Sfvml::

#endif
