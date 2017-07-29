// sfvml
#include "sfvml_frameextractor.h"

int main(int argc, char ** argv)
{
    using namespace sfvml;

    utility::extractFrameFromMp4(argv[1]);
    return 0;
}
