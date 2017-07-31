#ifndef SFVML_PROPERTY
#define SFVML_PROPERTY

namespace sfvml {
namespace size  {

// [0] = width, [1] = height
double k_urien[2] = {285 / 1200.0, 475 / 720.0};
double k_laura[2] = {270 / 1200.0, 420 / 720.0};
} // size:
namespace property {

// Folder where we will save the extracted frames
static const char* k_frameOutputFolder = "outputFrames/";

// [0] = x, [1] = y
double k_startingPos1[2] = {300 / 1280.0, 420 / 720.0};
double k_startingPos2[2] = {700 / 1280.0, 420 / 720.0};

// Used as a distance threshold to grey pixels to be removed
double k_greyThreshold = 40.0;

} // property::
} // sfvml::

#endif
