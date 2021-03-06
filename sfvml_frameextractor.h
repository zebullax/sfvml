#ifndef SFVML_FRAMEEXTRACTOR
#define SFVML_FRAMEEXTRACTOR

// std
#include <string>
// opencv2
#include "opencv2/opencv.hpp"

namespace Sfvml {

struct FrameExtractor
{
    FrameExtractor(const std::string& videoFilename, 
                   bool saveToFolder = false);

    // MODIFICATORS
    void discardNextNFrames(size_t nbToSkip);
 
    FrameExtractor& operator>>(cv::Mat& frame);

    // ACCESSORS

    int    matFormat() const { return d_matFormat; } 
    size_t currentFrame() const { return d_currentFrame; } 
    bool   isLastFrame() const { return d_currentFrame + 1== d_nbOfFrames; } 
    size_t nbOfFrames() const { return d_nbOfFrames; }
    const std::string& filename() const { return d_videoFilename; };
    // CONVERSION
    operator bool() const { return d_vid.isOpened(); }

private:

    size_t                d_nbOfFrames;
    size_t                d_currentFrame;
    size_t                d_fps;
    size_t                d_videoWidth;
    size_t                d_videoHeight;
    int                    d_matFormat;
    std::string            d_outFilePrefix;
    std::string            d_videoFilename;
    cv::VideoCapture    d_vid;
    bool                d_saveToFolder;
};
} // Sfvml::

#endif
