#ifndef SFVML_CHARACTERTRAITS
#define SFVML_CHARACTERTRAITS

// std::
#include <cstddef>
#include <string>
#include <vector>
// opencv
#include "opencv2/opencv.hpp"


namespace Sfvml {

enum class CharacterName {
    Urien,
    Laura
};

// Generic
template<CharacterName characterName>
struct CharacterTrait;

// Character descriptor 
template<>
struct CharacterTrait<CharacterName::Urien>
{
    static const std::string name;
};

template<>
struct CharacterTrait<CharacterName::Laura>
{
    static const std::string name;
};

} // sfvml

#endif
