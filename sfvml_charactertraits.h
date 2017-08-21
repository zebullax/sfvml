#ifndef SFVML_CHARACTERTRAITS
#define SFVML_CHARACTERTRAITS

#include <cstddef>
#include <string>

namespace Sfvml {

enum class CharacterName {
    Urien,
    Laura
};

// Generic
template<CharacterName characterName, size_t Width = 1280, size_t Height = 720> 
struct CharacterTrait;

// Spec per character
template<size_t Width, size_t Height > 
struct CharacterTrait<CharacterName::Urien, Width, Height>
{
    // TODO change for type Position ?
    // Top left corner
    double      p1StartX = (130 / 1280.0) * Width;
    double      p1StartY = (190 / 720.0) * Height;
    double      width    = (50 / 1280.0) * Width;
    double      height   = (60 / 720.0) * Height;
    std::string name     = "Urien";
    // FIXME the threshold should probably normalized by nb of pixels
    double      matchThreshold = 2200.0;
};

template<size_t Width, size_t Height> 
struct CharacterTrait<CharacterName::Laura, Width, Height>
{
    double      p2StartX = (800 / 1280.0) * Width;
    double      p2StartY = (250 / 720.0) * Height;
    double      width    = (290 / 1280.0) * Width;
    double      height   = (440 / 720.0) * Height;
    std::string name     = "Laura";
    // FIXME the threshold should probably normalized by nb of pixels
    double      matchThreshold = 30000.0;
};

} // sfvml

#endif
