#ifndef SFVML_PROPERTY
#define SFVML_PROPERTY

#include "sfvml_type.h"

namespace Sfvml    {
namespace property {

// Used to indicate that we could not find the character in the frame
static const Position k_characterNotFound = {-1.0, -1.0};

// Folder where we will save the extracted frames
static const char* k_frameOutputFolder = "outputFrames/";

// Folder that contains the reference crop used for tracking
static const char* k_referenceCropFolder = "referenceCrop/";

// Used as a distance threshold to grey pixels to be removed
static const double k_greyThreshold = 30.0;

// Used when dev... 
static const size_t k_sampleOutput = 29 * 30;

// Step size used when tracking characters, we dont track 
// every frame, but every 'trackStep'-th frame and 
// interpolate in between
static const size_t k_trackStep = 4;

} // property::
} // Sfvml::

#endif
