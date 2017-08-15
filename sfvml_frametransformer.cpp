//sfvml 
#include "sfvml_frametransformer.h"
#include "sfvml_property.h"
//std::
#include <algorithm>
// opencv
#include "opencv2/opencv.hpp"

namespace Sfvml            {
namespace FrameTransformer {

void addTracker(cv::Mat *frame,
                const std::vector<Position>& trajectory,
                size_t frameIdx)
{
    cv::circle(*frame, 
               cv::Point(trajectory[frameIdx].x, trajectory[frameIdx].y), 
               10, 
               cv::Scalar(255, 255, 255));
}


void addTrajectoryToVideo(const std::string& inVideoFilename,
                          const std::string& outVideoFilename,
                          const std::vector<Position>& trajectory)
{
    cv::VideoCapture vidRead(inVideoFilename);
    cv::VideoWriter vidWrite(outVideoFilename, 
                             CV_FOURCC('A','V','C','1'), 
                             29, 
                             cv::Size(1280, 720));
    cv::Mat extractedFrame;
    size_t currIdx = 0;

    while(vidRead.read(extractedFrame) && currIdx < property::k_sampleOutput)
    {
        addTracker(&extractedFrame, trajectory, currIdx);
        vidWrite.write(extractedFrame);
        ++currIdx;
    }
}


double compareHistogram(const cv::Mat& frameA, const cv::Mat& frameB)
{
    int hbins = 30, sbins = 32; 
    int channels[] = {0,  1};
    int histSize[] = {hbins, sbins};
    float hranges[] = { 0, 180 };
    float sranges[] = { 0, 255 };
    const float* ranges[] = { hranges, sranges}; 

    // Convert to HSV forma
    cv::Mat hsvA;
    cv::Mat hsvB;
    // Create the MatND objects to store the histograms
    cv::MatND histA;
    cv::MatND histB;
    // Convert to HSV
    cv::cvtColor(frameA, hsvA, CV_BGR2HSV);
    cv::cvtColor(frameB, hsvB, CV_BGR2HSV);
    // Calculate histograms
    cv::calcHist(&hsvA, 1, channels,  cv::Mat(), // do not use mask
                 histA, 2, histSize, ranges,
                 true, // the histogram is uniform
                 false );
    cv::calcHist(&hsvB, 1, channels,  cv::Mat(), // do not use mask
                 histB, 2, histSize, ranges,
                 true, // the histogram is uniform
                 false );
    
    cv::normalize(histA, histA,  0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(histB, histB,  0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    return compareHist(histA, histB, CV_COMP_CHISQR);
}


void removeGreyPixels(cv::Mat* frame) 
{
    // No forEach in opencv 2.3.12
    for (size_t r = 0; r < frame->rows; ++r) 
    {
        for (size_t c = 0; c < frame->cols; ++c)
        {
            cv::Vec3b & p = frame->at<cv::Vec3b>(r, c);
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
}


void sortFramePixels(cv::Mat *frame)
{
    cv::Mat hsvFrame;
    cv::cvtColor(*frame, hsvFrame, CV_BGR2HSV);
    
    std::vector<cv::Vec3b> linearizedFrame; 
    // This is shit, it ends up spltting the 3 channels into 3 diff values....
    //if (hsvFrame.isContinuous()) {
    //    linearizedFrame.assign(hsvFrame.datastart, hsvFrame.dataend);
    //} 
    //else 
    //{
    for (size_t i = 0; i < hsvFrame.rows; ++i) {
        linearizedFrame.insert(linearizedFrame.end(), 
                               hsvFrame.ptr<cv::Vec3b>(i), 
                               hsvFrame.ptr<cv::Vec3b>(i) + hsvFrame.cols);
    }
    //}
  
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

} // FrameTransformer::
} // Sfvml::
