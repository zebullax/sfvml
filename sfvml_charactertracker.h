#ifndef SFVML_CHARACTERTRACKER
#define SFVML_CHARACTERTRACKER

// sfvml
#include "sfvml_charactertraits.h"
#include "sfvml_property.h"
#include "sfvml_type.h"
#include "sfvml_frameextractor.h"
#include "sfvml_frametransformer.h"
// std
#include <array>
#include <vector>
#include <cmath>
// opencv
#include "opencv2/opencv.hpp"

namespace Sfvml
{

template <typename Character>
Position getCroppedArea(const cv::Mat& frame,
                        const Position& prevPosition,
                        const Direction& updateDirection,
                        const Character& character,
                        cv::Mat* croppedROI)
{
   
    // Translate from center of character to top left corner
    int32_t prevX = prevPosition.x - static_cast<int32_t>(character.width / 2);
    int32_t prevY = prevPosition.y - static_cast<int32_t>(character.height / 2);

    bool goRight = (updateDirection & 1) != 0;
    bool goLeft  = (updateDirection & 2) != 0;
    bool goUp    = (updateDirection & 4) != 0;
    bool goDown  = (updateDirection & 8) != 0;

    size_t offsetH = (goRight || goLeft) ? character.width / 2 : 0;
    size_t offsetV = (goUp || goDown) ? character.height / 2 : 0;

    // TODO change hardcoded 1200 * 720
    int32_t newX = prevX + (goLeft  ? -offsetH :
                            goRight ?  offsetH :
                            0);
    int32_t newY = prevY + (goDown ?  offsetV :
                            goUp   ? -offsetV  :
                            0);

    // clip to min max
    newX = std::max<int32_t>(newX, 0);
    newX = std::min<int32_t>(newX, static_cast<int32_t>(1200 - character.width));
    newY = std::max<int32_t>(newY, 0);
    newY = std::min<int32_t>(newY, static_cast<int32_t>(720 - character.height));
    
    ////std::cout << updateDirection << ": X " << prevX << "->" << newX 
    ////          << ", Y " << prevY << "->" << newY << std::endl;
    
    frame(cv::Rect(newX,
                   newY,
                   character.width,
                   character.height)).copyTo(*croppedROI);        
    croppedROI->reshape(1, character.width * character.height);
    return { static_cast<double>(newX + character.width / 2), 
             static_cast<double>(newY + character.height / 2) };
}

// That's where the magic happens... 
template <typename FirstCharacter,
          typename SecondCharacter,
          typename Measure>
void getCharacterTrajectories(const std::string&     videoFilename,
                              std::vector<Position> *trajectoryP1,
                              FirstCharacter         firstCharacter,
                              std::vector<Position> *trajectoryCharacter2,
                              SecondCharacter        secondCharacter,
                              Measure                norm)
{
	FrameExtractor frameExtractor(videoFilename, true);
    if (!frameExtractor) {
        std::cerr << "Error opening '" << videoFilename << "'\n";
        return;
    }
    // holds the x,y at the center of the character
    trajectoryP1->resize(frameExtractor.nbOfFrames());

    std::string outFilePrefix(videoFilename);
    std::replace(outFilePrefix.begin(),
                 outFilePrefix.end(),
                 '.',
                 '_');
    std::ostringstream extractedFrameFilename(outFilePrefix);

	// Those will hold the square supposedly framing the characters
    cv::Mat firstCharacterCrop;

    // Prime the pump, starting position is fixed 
	cv::Mat extractedFrame;
    size_t currentlyTrackedFrame  = 0;
    size_t prevTrackedFrame = 0;
	frameExtractor >> extractedFrame;
    frameExtractor.discardNextNFrames(property::k_trackStep);
    
    // Translate from top left corner to center of the character
    (*trajectoryP1)[currentlyTrackedFrame] = { firstCharacter.p1StartX
                                                        + firstCharacter.width / 2,
                                                    firstCharacter.p1StartY 
                                                        + firstCharacter.height / 2};
	getStartingCharVectors(extractedFrame,
	                       &firstCharacterCrop,
	                       firstCharacter);
    FrameTransformer::removeGreyPixels(&firstCharacterCrop);
    FrameTransformer::sortFramePixels(&firstCharacterCrop);

    // TODO Add progress status
    std::vector<Direction> updateDirections { 
        k_RIGHT,
        k_LEFT,
        k_UP,
        k_DOWN,
        k_UPLEFT,
        k_UPRIGHT,
        k_DOWNRIGHT,
        k_DOWNLEFT
    };

    std::array<cv::Mat, 11> originalCrops;

    while (!frameExtractor.isLastFrame())
    {
        std::cout <<"Frame " << currentlyTrackedFrame << std::endl;

        // Check surrounding crops and pick the minimal distance
        // Prime the pump from identical position
        cv::Mat minCrop;
        Direction minDirection = Direction::k_IDENT;
        Position minPosition = getCroppedArea(extractedFrame,
                                              (*trajectoryP1)[prevTrackedFrame],
                                              Direction::k_IDENT,
                                              firstCharacter,
                                              &minCrop);

        minCrop.copyTo(originalCrops[Direction::k_IDENT]);
        FrameTransformer::removeGreyPixels(&minCrop);
        FrameTransformer::sortFramePixels(&minCrop);

        double minDistance = norm(minCrop, firstCharacterCrop);
        std::cout << "Picking " << Direction::k_IDENT << " move from " 
                  << (*trajectoryP1)[prevTrackedFrame] << " to " 
                  << minPosition << " with distance " << minDistance << "\n";
        for(auto&& direction : updateDirections)
        {
            cv::Mat candidateCrop;
            Position candidatePosition = getCroppedArea(extractedFrame,
                                                        (*trajectoryP1)[prevTrackedFrame],
                                                        direction,
                                                        firstCharacter,
                                                        &candidateCrop);
            candidateCrop.copyTo(originalCrops[direction]);
            FrameTransformer::removeGreyPixels(&candidateCrop);
            FrameTransformer::sortFramePixels(&candidateCrop); 
            double candidateDistance = norm(candidateCrop, minCrop);
            std::cout << "Distance to " << direction << " " << candidateDistance << '\n'; 
            if (candidateDistance < minDistance)
            {
                minDirection = direction;
                minDistance = candidateDistance;
                candidateCrop.copyTo(minCrop);
                minPosition = candidatePosition;
                std::cout << "Picking " << direction << " move from " 
                          << (*trajectoryP1)[prevTrackedFrame] << " to " 
                          << minPosition << " with distance " << minDistance << "\n";
            }            
        }

        (*trajectoryP1)[currentlyTrackedFrame] = minPosition;
        extractedFrameFilename << property::k_frameOutputFolder
                               << outFilePrefix
                               << std::setw(10) << std::setfill('0')
                               << currentlyTrackedFrame << ".jpg";
        prevTrackedFrame = currentlyTrackedFrame;
        originalCrops[minDirection].reshape(firstCharacter.width, firstCharacter.height);
        cv::imwrite(extractedFrameFilename.str(), originalCrops[minDirection]);
        cv::imwrite(extractedFrameFilename.str() + "_sorted.jpg", minCrop);
        extractedFrameFilename.str("");
        
        // TODO interpolate in between
        
        frameExtractor >> extractedFrame;
        // current points to one past read, hence - 1 
        currentlyTrackedFrame = frameExtractor.currentFrame() - 1;         
        frameExtractor.discardNextNFrames(property::k_trackStep);
    }
}


template <typename FirstCharacter>
void getStartingCharVectors(const cv::Mat&  firstFrame,
                            cv::Mat        *characterVector,
                            FirstCharacter  character)
{

	firstFrame(cv::Rect(character.p1StartX, 
                        character.p1StartY, 
                        character.width, 
                        character.height)).copyTo(*characterVector);
    characterVector->reshape(1, character.width * character.height);

}

template <CharacterName characterName>
void displayCharacter(const cv::Mat&                characterVector, 
                      CharacterTrait<characterName> characterTrait)
{
	cv::Mat reconstructedImg;
    characterVector.copyTo(reconstructedImg);
    reconstructedImg.reshape(characterTrait.width, characterTrait.height);
	cv::imshow(characterTrait.name, reconstructedImg);
	cv::waitKey();
}

} // Sfvml::


#endif
