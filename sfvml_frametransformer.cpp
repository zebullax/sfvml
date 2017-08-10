//sfvml 
#include "sfvml_frametransformer.h"

namespace sfvml {

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


} // sfvml::

#
