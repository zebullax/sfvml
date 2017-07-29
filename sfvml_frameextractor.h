#ifndef SFVML_FRAMEEXTRACTOR
#define SFVML_FRAMEEXTRACTOR

// std
#include <string>

namespace sfvml   {
namespace utility {


// Extact all frames from the video, 
// Frame will be saved to a directory built if not existing already.
// videoFilename -> Path to the video
// <- Returns the number of extracted frames
uint32_t extractFrameFromMp4(const std::string& videoFilename);


} // utility::
} // sfvml::

#endif
