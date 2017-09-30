// sfvml
#include "sfvml_charactertracker.h"
#include "sfvml_property.h"
// std
#include <fstream>
#include <sstream>

namespace Sfvml {

void addTracker(cv::Mat          *frame,
                const Trajectory& trajectory,
                size_t            frameIdx)
{
    if (trajectory.isGood(frameIdx))
    {
        cv::circle(*frame, 
                   cv::Point(trajectory[frameIdx].x, trajectory[frameIdx].y), 
                   15,
                   cv::Scalar(255, 0, 0),
                   -1 /* fill */); 
    }
}


void addTrajectoryToVideo(const std::string& inVideoFilename,
                          const std::string& outVideoFilename,
                          const Trajectory&  trajectory)
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

void removeStatisticalUnderliers(const std::vector<double>& distances,
                                 std::vector<size_t> *validFrames,
                                 std::vector<size_t> *unmatchedFrames,
                                 double               stddevOffset)
{
    // Mean / StdDev to remove underliers
    double sum             = .0f;
    size_t nbFramesChecked = 0;

    for(size_t i = 0; i < distances.size(); i += property::k_trackStep + 1) {
        sum += distances[i];
        ++nbFramesChecked;
    }
    
    double mean  = sum / nbFramesChecked; 
    double numer = .0;
    for(size_t i = 0; i < distances.size(); i += property::k_trackStep + 1) {
        numer += std::pow(distances[i] - mean, 2);
    }
    
    double stdev = std::sqrt(numer / nbFramesChecked);
   
    std::cout << "Distance, mean = " << mean << ", stddev = " << stdev << std::endl;
    for(size_t i = 0; i < distances.size(); ++i) 
    {
        if ((i % (property::k_trackStep + 1)) == 0
         && (std::abs(distances[i] - mean) < mean + stddevOffset * stdev))
        {
            validFrames->push_back(i);
        }
        else {
            unmatchedFrames->push_back(i);
        }
    }

}

Position getBarycentre(const std::vector<CropMatch>& cropMatches)
{
    Position wCenter;
    double eps = .00001; // Dont want to divide by 0...
    double sumOfDistance = std::accumulate(cropMatches.begin(), 
                                           cropMatches.end(),
                                           .0,
                                           [eps](double sum, const CropMatch& match) { 
                                               return (sum + (1 / (match.second + eps)));
                                           });

    for(auto&& cropMatch : cropMatches) 
    {
        std::cout << "Adding match " << cropMatch.first << " with weight " 
                  << ((1 / (cropMatch.second + eps)) / sumOfDistance) << std::endl;
        wCenter.x += ((1 / (cropMatch.second + eps)) / sumOfDistance) * cropMatch.first.x;
        wCenter.y += ((1 / (cropMatch.second + eps)) / sumOfDistance) * cropMatch.first.y;
        std::cout << "Updated weighted center to " << wCenter << std::endl;
    }
    return wCenter;
}

} // sfvml::
