// sfvml
#include "sfvml_frameextractor.h"
#include "sfvml_property.h"
// std
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <cmath>

namespace sfvml   {

void removeGrey(cv::Mat *frame)
{
	typedef cv::Point3_<uint8_t> Pixel;
	frame->forEach<Pixel>([](Pixel &p, const int * position) -> void 
	{
		// 90 or 140 are the main grey in the background
		if (std::sqrt(std::pow((p.x - 140), 2) + std::pow((p.y - 140), 2) + std::pow((p.y - 140), 2))  < property::k_greyThreshold
			|| std::sqrt(std::pow((p.x - 90), 2) + std::pow((p.y - 90), 2) + std::pow((p.y - 90), 2)) < property::k_greyThreshold)
		{
			p.x = 0;
			p.y = 0;
			p.z = 0;
		}

	});
}

uint32_t extractFrameFromMp4(const std::string& videoFilename)
{
    std::string outFilePrefix(videoFilename);
    std::replace(outFilePrefix.begin(), 
                 outFilePrefix.end(), 
                 '.', 
                 '_');
    
    uint32_t nbExtractedFrames = 0;
    cv::VideoCapture vid(videoFilename);
    if (!vid.isOpened()) {
        return nbExtractedFrames;
    }
    
    // Get some properties
    size_t nbOfFrames  = static_cast<size_t>(vid.get(CV_CAP_PROP_FRAME_COUNT));
    size_t fps         = static_cast<size_t>(vid.get(CV_CAP_PROP_FPS));
    size_t videoWidth  = static_cast<size_t>(vid.get(CV_CAP_PROP_FRAME_WIDTH));
    size_t videoHeight = static_cast<size_t>(vid.get(CV_CAP_PROP_FRAME_HEIGHT));


    std::cout << "Video properties: " << videoWidth << '*' 
              << videoHeight << '*' << fps << std::endl;
    // Boilerplate for the extracted filename
    size_t counterFormat = static_cast<size_t>(std::log10(nbOfFrames)) + 1; 
    std::ostringstream extractedFrameFilename(outFilePrefix);
 
    // Actual frame extraction loop
    cv::Mat extractedFrame; 
    std::cout << "Frame extraction progress: 00%" << std::flush; 
	while (vid.read(extractedFrame))
	{
		// Update progress status
		if (nbExtractedFrames % fps == 0) {
			size_t completionPct = (nbExtractedFrames * 100) / nbOfFrames;
			std::cout << "\b\b\b" << std::setw(2) << std::setfill('0')
				<< completionPct << '%' << std::flush;
		}

		extractedFrameFilename << property::k_frameOutputFolder
			<< outFilePrefix
			<< std::setw(counterFormat) << std::setfill('0')
			<< nbExtractedFrames << ".jpg";

		removeGrey(&extractedFrame);
        if(cv::imwrite(extractedFrameFilename.str(), extractedFrame)) {
            ++nbExtractedFrames;
        }
        extractedFrameFilename.str("");
    }
    return nbExtractedFrames;
}


} // sfvml::
