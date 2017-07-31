// sfvml
#include "sfvml_frameextractor.h"
//std
#include <string>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <cmath>


int main(int argc, char ** argv)
{
    using namespace sfvml;
    std::string videoFilename(argv[1]);
    
    (void)extractFrameFromMp4(videoFilename);

    return 0;
}
