#include "sfvml_charactertracker.h"

namespace Sfvml {

void interpolatePositionGap(std::vector<Position> *trajectoryP1,
                        size_t prevIdx, 
                        size_t currIdx)
{
    std::cout << prevIdx << ' ' << currIdx << std::endl;
    std::cout << (*trajectoryP1)[currIdx] << ' ' << (*trajectoryP1)[prevIdx] << std::endl;
    double xDelta = ((*trajectoryP1)[currIdx].x - (*trajectoryP1)[prevIdx].x) 
                    / (currIdx - prevIdx);
    double yDelta = ((*trajectoryP1)[currIdx].y - (*trajectoryP1)[prevIdx].y) 
                    / (currIdx - prevIdx);
    for (size_t i = prevIdx + 1 ; i != currIdx; ++i) 
    {
        std::cout << i << std::endl;
        (*trajectoryP1)[i] = {(*trajectoryP1)[i - 1].x + xDelta, 
                              (*trajectoryP1)[i - 1].y + yDelta};
    }
}

} // sfvml::
