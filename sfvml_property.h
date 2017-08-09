#ifndef SFVML_PROPERTY
#define SFVML_PROPERTY

namespace sfvml    {
namespace property {

// Folder where we will save the extracted frames
static const char* k_frameOutputFolder = "outputFrames/";

// Used as a distance threshold to grey pixels to be removed
static const double k_greyThreshold = 30.0;

// Step size used when tracking characters, we dont track 
// every frame, buy every 'trackStep'-th frame and 
// interpolate in between
static const size_t k_trackStep = 5;

} // property::
} // sfvml::

#endif
