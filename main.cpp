// sfvml
#include "sfvml_frameextractor.h"
#include "sfvml_type.h"
#include "sfvml_charactertraits.h"
#include "sfvml_charactertracker.h"
#include "sfvml_frametransformer.h"

int main(int argc, char ** argv)
{
    using namespace Sfvml;
    std::string                                videoFilename(argv[1]);
    std::vector<Position>                    urienTrajectory;
    std::vector<Position>                    lauraTrajectory;
    CharacterTrait<CharacterName::Urien>    urien;
    CharacterTrait<CharacterName::Laura>    laura;

    auto l2norm = [] (const cv::Mat& a, const cv::Mat& b) {
        return cv::norm(a, b);
    };

    auto histCmp = [] (const cv::Mat& a, const cv::Mat& b) {
        return Sfvml::FrameTransformer::compareHistogram(a, b);
    };

    getCharacterTrajectories(videoFilename, 
                             &urienTrajectory, 
                             urien, 
                             &lauraTrajectory, 
                             laura,
                             l2norm);

    return 0;
}
