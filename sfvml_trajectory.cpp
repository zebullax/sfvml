#include "sfvml_trajectory.h"

namespace Sfvml {

void Trajectory::interpolatePositionGap(size_t prevIdx, 
                                        size_t currIdx)
{

    std::cout << "Linear interpolation between "
              << '#' <<prevIdx << ' ' << d_trajectory[prevIdx] 
              << "  ->  "
              << '#' << currIdx << ' ' << d_trajectory[currIdx] 
              << std::endl;
    // Linear interpolation
    double xDelta = d_trajectory[currIdx].x - d_trajectory[prevIdx].x 
                    / (currIdx - prevIdx);
    double yDelta = d_trajectory[currIdx].y - d_trajectory[prevIdx].y 
                    / (currIdx - prevIdx);
    for (size_t i = prevIdx + 1 ; i != currIdx; ++i) 
    {
        d_trajectory[i] = {d_trajectory[i - 1].x + xDelta, 
                              d_trajectory[i - 1].y + yDelta};
    }
}

void Trajectory::readFromFile(const std::string& filename)
{
    std::ifstream ifs(filename);
    std::string line;
    double x = .0;
    double y = .0;
    
    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        iss >> x >> y;
        d_trajectory.emplace_back(x, y);
    }
}

void Trajectory::saveToFile(const std::string& filename) const
{
    std::ofstream ofs(filename);
    for(auto&& p : d_trajectory) {
        ofs << p.x << ' ' << p.y << std::endl;
    } 
}
   

void Trajectory::interpolatePosition(const std::vector<size_t>& keyFrames)
{
    if (keyFrames.size() < 2) {
        return;
    }

    size_t maxIdx   = keyFrames.size();
    size_t idx      = 0;
    size_t startIdx = keyFrames[idx++];
    size_t stopIdx  = keyFrames[idx];

    while(idx < maxIdx)
    {
        interpolatePositionGap(startIdx,
                               stopIdx);
        std::swap(startIdx, stopIdx);
        if (idx != maxIdx) {
            stopIdx = keyFrames[++idx];
        }
    } 
}


} // Sfvml:: 
