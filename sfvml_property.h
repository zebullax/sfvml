#ifndef SFVML_PROPERTY
#define SFVML_PROPERTY

namespace sfvml    {
namespace property {

// Folder where we will save the extracted frames
static const char* k_frameOutputFolder = "outputFrames/";

// Used as a distance threshold to grey pixels to be removed
static const double k_greyThreshold = 40.0;

// Step size used when tracking characters, we dont track 
// every frame, buy every 'trackStep'-th frame and 
// interpolate in between
static const size_t k_trackStep = 29;


// When cropping a new ROI, select a direction for the 
// update wrt. previous position
enum class MoveDirection 
{
    k_IDENT_UPDATE,
    k_RIGHT_UPDATE,
    k_LEFT_UPDATE,
    k_UP_UPDATE,
    k_DOWN_UPDATE
};

inline std::ostream& operator<<(std::ostream& os, const MoveDirection& direction)
{
    os << (direction == MoveDirection::k_IDENT_UPDATE ? "Identical" :
           direction == MoveDirection::k_RIGHT_UPDATE ? "Right"     :
           direction == MoveDirection::k_LEFT_UPDATE  ? "Left"      :
           direction == MoveDirection::k_UP_UPDATE    ? "Up"        :
                                                        "Down");
    return os;
}

} // property::
} // sfvml::

#endif
