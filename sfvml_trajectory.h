#ifndef SFVML_TRAJECTORY
#define SFVML_TRAJECTORY

// sfvml::
#include "sfvml_type.h"
#include "sfvml_property.h"
// std::
#include <vector>

namespace Sfvml {

class Trajectory{

public:
   
    // CREATORS
    Trajectory() = default;

    Trajectory(const std::string& persistedTrajectory) : d_trajectory() {
        readFromFile(persistedTrajectory);
    }

    // MANIPULATORS
    void saveToFile(const std::string& filename) const;

    void readFromFile(const std::string& filename);
   
    void interpolatePosition(const std::vector<size_t>& keyFrames);

    bool isGood(size_t frameIdx) const { 
        return d_trajectory[frameIdx] == property::k_characterNotFound; } 

    // Forwarded to underlying container
    void push_back(const Position& pos) { d_trajectory.push_back(pos); }
    void resize(size_t newSize) { d_trajectory.resize(newSize); }
    Position& operator[](const size_t idx) { return d_trajectory[idx]; }
    const Position& operator[](const size_t idx) const { return d_trajectory[idx]; }

private:

    // MANIPULATORS

    // Linearly interpolate character positions between 2 keyframe
    // prevIdx -> last known position
    // currIdx -> most recently detected position
    void interpolatePositionGap(size_t prevIdx, 
                                size_t currIdx);

    // DATA
    // Container holding the actual position of the character, depending on the track step
    // some of the values may be irrelevant
    std::vector<Position> d_trajectory;    

};

} // sfvml::

#endif
