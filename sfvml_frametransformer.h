#ifndef SFVML_FRAMETRANSFORMER
#define SFVML_FRAMETRANSFORMER

// sfvml
#include "sfvml_type.h"
// opencv2
#include "opencv2/opencv.hpp"

namespace Sfvml {

// Remove (= set to black) the grey pixels, where we expect
// the background to be "The grid", frame should be a row vector
// frame -> Frame read through cv to be cleaned
void removeGreyPixels(cv::Mat *frame);

// Remove the black pixel on a H S V sorted frame
// We expect frame to be a row vector
void removeBlackPixels(cv::Mat *frame);

void sortFramePixels(cv::Mat *frame);

// Return a shifted crop of the image relative to prevPosition
// The return cropped is a row vector
Position getShiftedCrop(const cv::Mat&    frame,
                        const Position&   prevPosition,
                        const Direction&  updateDirection,
                        size_t            characterCropSize,
                        cv::Mat          *croppedROI,
                        double            offsetScaleDownRatio = 1.0);

} // Sfvml::

#endif
