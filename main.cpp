// sfvml
#include "sfvml_frameextractor.h"
#include "sfvml_type.h"
#include "sfvml_charactertraits.h"
#include "sfvml_charactertracker.h"

int main(int argc, char ** argv)
{
    using namespace sfvml;
    std::string								videoFilename(argv[1]);
	std::vector<Position>					urienTrajectory;
	std::vector<Position>					lauraTrajectory;
	CharacterTrait<CharacterName::Urien>	urien;
	CharacterTrait<CharacterName::Laura>	laura;

	getCharacterTrajectories(videoFilename, 
                             &urienTrajectory, 
                             urien, 
                             &lauraTrajectory, 
                             laura);

    return 0;
}
