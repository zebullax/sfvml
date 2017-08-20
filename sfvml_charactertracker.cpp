#include "sfvml_charactertracker.h"

namespace Sfvml {

void addTracker(cv::Mat *frame,
                const std::vector<Position>& trajectory,
                size_t frameIdx)
{
    bool failedFrame = trajectory[frameIdx] == property::k_characterNotFound;
   
    if (!failedFrame)
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

void interpolatePositionGap(std::vector<Position> *trajectoryP1,
                            size_t prevIdx, 
                            size_t currIdx)
{

    std::cout << "Entering linear interpolation " 
              << (*trajectoryP1)[prevIdx] 
              << " -> "
              << (*trajectoryP1)[currIdx] 
              << std::endl;
    // Linear interpolation
    double xDelta = ((*trajectoryP1)[currIdx].x - (*trajectoryP1)[prevIdx].x) 
                    / (currIdx - prevIdx);
    double yDelta = ((*trajectoryP1)[currIdx].y - (*trajectoryP1)[prevIdx].y) 
                    / (currIdx - prevIdx);
    for (size_t i = prevIdx + 1 ; i != currIdx; ++i) 
    {
        (*trajectoryP1)[i] = {(*trajectoryP1)[i - 1].x + xDelta, 
                              (*trajectoryP1)[i - 1].y + yDelta};
    }
}

void interpolatePositionInTrajectory(std::vector<Position> *trajectoryP1,
                                     const std::vector<size_t>& validFrames)
{
    if (validFrames.size() < 2) {
        return;
    }

    size_t maxIdx   = validFrames.size();
    size_t idx      = 0;
    size_t startIdx = validFrames[idx++];
    size_t stopIdx  = validFrames[idx];

    while(idx < maxIdx)
    {
        interpolatePositionGap(trajectoryP1, 
                               startIdx,
                               stopIdx);
        std::swap(startIdx, stopIdx);
        if (idx != maxIdx) {
            stopIdx = validFrames[++idx];
        }
    } 
}

bool detectPreRound(const cv::Mat& frame)
{
    static cv::Mat preRoundRef(cv::imread(std::string(property::k_referenceCropFolder) + "preRound.png"));
    cv::Mat timerCrop(frame(cv::Rect(605, 40, 70, 55)));
    return (norm(preRoundRef, timerCrop) < 1000); 
}

} // sfvml::
