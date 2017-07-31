#ifndef SFVML_FRAMEEXTRACTOR
#define SFVML_FRAMEEXTRACTOR

// std
#include <string>
// opencv2
#include "opencv2/opencv.hpp"

namespace sfvml {


// Extact all frames from the video, 
// Frame will be saved to a directory built if not existing already.
// videoFilename -> Path to the video
// <- Returns the number of extracted frames
uint32_t extractFrameFromMp4(const std::string& videoFilename);

// Remove (= set to black) the grey pixels, where we expect
// the background to be "The grid"
// frame -> Frame read through cv to be cleaned
void removeGrey(cv::Mat *frame);

} // sfvml::

#endif
