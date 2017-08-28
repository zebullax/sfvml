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
    // !! crop is assumed to be square (#cols = #rows)
    std::vector<size_t>  cropSizes; // Will be filled out when loading crops 
    std::vector<cv::Mat> refCrops;
    const std::string name = "Urien";
};

template<>
struct CharacterTrait<CharacterName::Laura>
{
    // !! crop is assumed to be square (#cols = #rows)
    std::vector<size_t>  cropSizes; // Will be filled out when loading crops 
    std::vector<cv::Mat> refCrops;
    const std::string name = "Laura";
};

} // sfvml

#endif
