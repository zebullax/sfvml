// sfvml
#include "sfvml_frameextractor.h"
#include "sfvml_property.h"
#include "sfvml_type.h"
// std
#include <iomanip>
#include <sstream>
#include <cmath>

namespace sfvml   {

	FrameExtractor::FrameExtractor(const std::string& videoFilename,
								   bool saveToFolder) :
									d_nbOfFrames{},
									d_currentFrame{},
									d_fps{},
									d_videoWidth{},
									d_videoHeight{},
									d_matFormat{},
									d_outFilePrefix(videoFilename),
									d_videoFilename(videoFilename),
									d_vid(d_videoFilename),
									d_saveToFolder(saveToFolder)
	{
		d_nbOfFrames	= static_cast<size_t>(d_vid.get(CV_CAP_PROP_FRAME_COUNT));
		d_fps			= static_cast<size_t>(d_vid.get(CV_CAP_PROP_FPS));
		d_videoWidth	= static_cast<size_t>(d_vid.get(CV_CAP_PROP_FRAME_WIDTH));
		d_videoHeight	= static_cast<size_t>(d_vid.get(CV_CAP_PROP_FRAME_HEIGHT));
		d_matFormat		= static_cast<int>(d_vid.get(CV_CAP_PROP_FORMAT));

		std::replace(d_outFilePrefix.begin(), d_outFilePrefix.end(), '.', '_');

		std::cout << "Video properties: " << d_videoWidth << '*' 
                  << d_videoHeight << '*' << d_fps << ":" << d_matFormat << std::endl;
	}

    void FrameExtractor::discardNextNFrames(size_t nbToSkip)
    {
        cv::Mat scratch;
        for (size_t i = 0; i != nbToSkip; ++i) { 
            d_vid.read(scratch);
            d_currentFrame++;
        }
    }

	FrameExtractor& FrameExtractor::operator>>(cv::Mat& extractedFrame)
	{
		d_vid.read(extractedFrame);		
		if (d_saveToFolder)
		{
			std::ostringstream extractedFrameFilename;

			extractedFrameFilename << property::k_frameOutputFolder
				<< d_outFilePrefix
				<< std::setw(10) << std::setfill('0')
				<< d_currentFrame << "_original.jpg";
            cv::imwrite(extractedFrameFilename.str(), extractedFrame);
		}
		d_currentFrame++;
		return *this;		
	}

	void FrameExtractor::removeGrey(cv::Mat* frame) 
	{
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
} // sfvml::
