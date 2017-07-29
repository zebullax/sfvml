// sfvml
#include "sfvml_frameextractor.h"
#include "sfvml_property.h"
// opencv2
#include "opencv2/opencv.hpp"
// std
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <cmath>


namespace sfvml   {
namespace utility {


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
    
        // Boilerplate for the extracted filename
    double nbOfFrames = vid.get(CV_CAP_PROP_FRAME_COUNT);
    size_t counterFormat = static_cast<size_t>(std::log10(nbOfFrames)) + 1; 
    std::ostringstream extractedFrameFilename(outFilePrefix);
    
    // Actual frane extraction loop
    cv::Mat extractedFrame; 
    while(vid.read(extractedFrame)) 
    {
        extractedFrameFilename << property::k_frameOutputFolder
                               << outFilePrefix
                               << std::setw(counterFormat) << std::setfill('0') 
                               << nbExtractedFrames << ".jpg";
        std::cout << extractedFrameFilename.str() << '\n';
        if(cv::imwrite(extractedFrameFilename.str(), extractedFrame)) {
            ++nbExtractedFrames;
        }
        extractedFrameFilename.str("");
    }
    return nbExtractedFrames;
}


} // utility::
} // sfvml::
