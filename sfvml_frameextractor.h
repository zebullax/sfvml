#ifndef SFVML_FRAMEEXTRACTOR
#define SFVML_FRAMEEXTRACTOR

// std
#include <string>
// opencv2
#include "opencv2/opencv.hpp"

namespace sfvml {

struct FrameExtractor
{
	FrameExtractor(const std::string& videoFilename, 
                   bool saveToFolder = false);

    // MODIFICATORS
    
	FrameExtractor& operator>>(cv::Mat& frame);

    // ACCESSORS

	int    matFormat() const { return d_matFormat; } 
    size_t currentFrame() const { return d_currentFrame; } 
    bool   isLastFrame() const { return d_currentFrame == d_nbOfFrames; } 
    size_t nbOfFrames() const { return d_nbOfFrames; }

    // CONVERSION
    operator bool() const { return d_vid.isOpened(); }

private:

	// Remove (= set to black) the grey pixels, where we expect
	// the background to be "The grid"
	// frame -> Frame read through cv to be cleaned
	void removeGrey(cv::Mat *frame) const;

	size_t				d_nbOfFrames;
	size_t				d_currentFrame;
	size_t				d_fps;
	size_t				d_videoWidth;
	size_t				d_videoHeight;
	int					d_matFormat;
	std::string			d_outFilePrefix;
	std::string			d_videoFilename;
	cv::VideoCapture	d_vid;
	bool				d_saveToFolder;
};
} // sfvml::

#endif
