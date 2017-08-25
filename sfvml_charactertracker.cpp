#include "sfvml_charactertracker.h"
#include <fstream>
#include <sstream>

namespace Sfvml {

void saveTrajectoryToFile(const std::vector<Position>& trajectory,
                          const std::string& filename)
{
    std::ofstream ofs(filename);
    for(auto&& p : trajectory) {
        ofs << p.x << ' ' << p.y << std::endl;
    }
}

std::vector<Position> getTrajectoryFromFile(const std::string& filename)
{
    std::vector<Position> trajectory;
    std::ifstream ifs(filename);
    std::string line;
    double x = .0;
    double y = .0;
    
    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        iss >> x >> y;
        trajectory.push_back(Position{x, y});
    }
    return trajectory; 
}


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

    std::cout << "Linear interpolation between "
              << '#' <<prevIdx << ' ' << (*trajectoryP1)[prevIdx] 
              << "  ->  "
              << '#' << currIdx << ' ' << (*trajectoryP1)[currIdx] 
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

void interpolatePositionInTrajectory(std::vector<Position>     *trajectoryP1,
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

void removeStatisticalUnderliers(const std::vector<double>& distances,
                                 std::vector<size_t> *validFrames,
                                 std::vector<size_t> *unmatchedFrames)
{
    // Mean / StdDev to remove underliers
    double sum             = .0f;
    size_t nbFramesChecked = 0;
    std::cout << distances.size() << std::endl;

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
            && (distances[i] > mean - 1.5 * stdev && distances[i] < mean + 1.5 * stdev))
        {
            validFrames->push_back(i);
        }
        else {
            unmatchedFrames->push_back(i);
        }
    }

}

} // sfvml::
