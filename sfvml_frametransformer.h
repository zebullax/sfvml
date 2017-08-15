#ifndef SFVML_FRAMETRANSFORMER
#define SFVML_FRAMETRANSFORMER

// sfvml
#include "sfvml_type.h"
// opencv2
#include "opencv2/opencv.hpp"

namespace Sfvml            {
namespace FrameTransformer {

void addTracker(cv::Mat *frame,
                const std::vector<Position>& trajectory,
                size_t frameIdx);

void addTrajectoryToVideo(const std::string& inVideoFilename,
                          const std::string& outVideoFilename,
                          const std::vector<Position>& trajectory);

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
