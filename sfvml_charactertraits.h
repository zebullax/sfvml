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
template<CharacterName characterName>
struct CharacterTrait;

// Spec per character
template<>
struct CharacterTrait<CharacterName::Urien>
{
    // TODO change for type Position ?
    // Top left corner
    size_t      cropSize = 90;
    std::string name     = "Urien";
};

template<>
struct CharacterTrait<CharacterName::Laura>
{
    size_t      cropSize = 90;
    std::string name     = "Laura";
};

} // sfvml

#endif
