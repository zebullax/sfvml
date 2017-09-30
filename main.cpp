// sfvml
#include "sfvml_type.h"
#include "sfvml_charactertraits.h"
#include "sfvml_charactertracker.h"
#include "sfvml_trajectory.h"

int main(int argc, char ** argv)
{
    using namespace Sfvml;
    
    std::string                          videoFilename(argv[1]);
    Trajectory                           urienTrajectory;
    Trajectory                           lauraTrajectory;
    CharacterTrait<CharacterName::Urien> urien;
    CharacterTrait<CharacterName::Laura> laura;

    getCharacterTrajectories(videoFilename, 
                             &urienTrajectory, 
                             urien, 
                             &lauraTrajectory, 
                             laura);

    addTrajectoryToVideo(videoFilename,
                         "test.mp4",
                         urienTrajectory);

    return EXIT_SUCCESS;
}
