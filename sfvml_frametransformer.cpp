//sfvml 
#include "sfvml_frametransformer.h"
#include "sfvml_property.h"
//std::
#include <algorithm>
// opencv
#include "opencv2/opencv.hpp"

namespace Sfvml {


void removeBlackPixels(cv::Mat *frame)
{
    cv::Mat correctedFrame;
    // ...No forEach in opencv 2.3.12
    for (size_t r = 0; r < frame->cols; ++r) 
    {
        cv::Vec3b& p = frame->at<cv::Vec3b>(r);
        if (p[0] != 0 || p[1] != 0 || p[2] != 0) {
            correctedFrame.push_back(p);
        }
    }
    correctedFrame.reshape(0, 1).copyTo(*frame);
}

void removeGreyPixels(cv::Mat *frame) 
{
    for (size_t r = 0; r < frame->cols; ++r) 
    {
        cv::Vec3b & p = frame->at<cv::Vec3b>(r);
        // 90^3 and 140^3 are the main grey in the background
        if (std::sqrt(std::pow((p[0] - 140), 2) + std::pow((p[1] - 140), 2) + std::pow((p[3] - 140), 2))  < property::k_greyThreshold
            || std::sqrt(std::pow((p[0] - 90), 2) + std::pow((p[1] - 90), 2) + std::pow((p[2] - 90), 2)) < property::k_greyThreshold)
        {
            p[0] = 0;
            p[1] = 0;
            p[2] = 0;
        }
    }
}


void sortFramePixels(cv::Mat *frame)
{
    cv::Mat hsvFrame;
    cv::cvtColor(*frame, hsvFrame, CV_BGR2HSV);
    
    std::vector<cv::Vec3b> linearizedFrame; 

    for (size_t i = 0; i < hsvFrame.rows; ++i) {
        linearizedFrame.insert(linearizedFrame.end(), 
                               hsvFrame.ptr<cv::Vec3b>(i), 
                               hsvFrame.ptr<cv::Vec3b>(i) + hsvFrame.cols);
    }
  
    std::sort(linearizedFrame.begin(), 
              linearizedFrame.end(),
              [] (const cv::Vec3b& a, const cv::Vec3b& b) {
                  // Sort on h, then s, then v
                  return (a[0] < b[0] 
                          || (a[0] == b[0] && a[1] < b[1]) 
                          || (a[0] == b[0] && a[1] < b[1] && a[2] < b[2]) ? true :
                                                                            false); 
                  });
    hsvFrame = cv::Mat(frame->rows, 
                       frame->cols,
                       CV_8UC3,
                       &(*linearizedFrame.begin()));
    cv::cvtColor(hsvFrame, *frame, CV_HSV2BGR); 
}

} // Sfvml::
