#ifndef SFVML_TYPE
#define SFVML_TYPE

// opencv
#include "opencv2/opencv.hpp"
// std
#include <ostream>

namespace sfvml {

using Pixel = cv::Point3_<uint8_t>;

struct Position { double x; 
                  double y; };

inline std::ostream& operator<<(std::ostream& os, const Position& pos) {
    os << "x:" << pos.x << ", y:" << pos.y;
    return os;
}

// When cropping a new ROI, select a direction for the 
// update wrt. previous position
enum Direction
{
    k_IDENT = 0,
    k_RIGHT = 1,
    k_LEFT  = 2,
    k_UP    = 4,
    k_DOWN  = 8,
    // Combinations
    k_UPLEFT = 6,
    k_UPRIGHT = 5,
    k_DOWNRIGHT = 9,
    k_DOWNLEFT = 10
};

inline std::ostream& operator<<(std::ostream& os, const Direction& direction)
{
    os << (direction == Direction::k_IDENT     ? "Identical" :
           direction == Direction::k_RIGHT     ? "Right"     :
           direction == Direction::k_LEFT      ? "Left"      :
           direction == Direction::k_UP        ? "Up"        :
           direction == Direction::k_DOWN      ? "Down"      :
           direction == Direction::k_UPLEFT    ? "UpLeft"    :
           direction == Direction::k_UPRIGHT   ? "UpRight"   :
           direction == Direction::k_DOWNRIGHT ? "DownRight" :
                                                "DownLeft"  );
    return os;
}


} // sfvml::

#endif
