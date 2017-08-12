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
                        cv::Mat* croppedROI,
                        double coolingRatio = 1.0)
{
   
    // Translate from center of character to top left corner
    int32_t prevX = prevPosition.x - static_cast<int32_t>(character.width / 2);
    int32_t prevY = prevPosition.y - static_cast<int32_t>(character.height / 2);

    bool goRight = (updateDirection & 1) != 0;
    bool goLeft  = (updateDirection & 2) != 0;
    bool goUp    = (updateDirection & 4) != 0;
    bool goDown  = (updateDirection & 8) != 0;

    size_t offsetH = coolingRatio * ((goRight || goLeft) ? character.width / 2 : 0);
    size_t offsetV = coolingRatio * ((goUp || goDown) ? character.height / 2 : 0);

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

template<typename Character>
Position simulatedAnnealingSearch(const cv::Mat&    frame,
                                  const cv::Mat&    refCharacterCrop,
                                  const Position&   lastPosition,
                                  const Character&  character,
                                  cv::Mat          *nextCharacterCrop,
                                  cv::Mat          *nextSortedCharacterCrop)
{

    static std::vector<Direction> updateDirections {
        k_IDENT,
        k_RIGHT,
        k_LEFT,
        k_UP,
        k_DOWN,
        k_UPLEFT,
        k_UPRIGHT,
        k_DOWNRIGHT,
        k_DOWNLEFT
    };
    
    // Here 'local' refer to minimum as they are found in an iteration of 
    // the search, while 'global' is the overall minimum
    // nextGlobal is used to track the new global min when exploring locally
   
    cv::Mat globalCrop;
    cv::Mat globalTransformedCrop;
    Position globalPosition = lastPosition;
    double globalDistance = std::numeric_limits<double>::max();
    double descentStep;

    do
    {
        double coolingFactor = 1.0;
        cv::Mat localCrop;
        std::cout << "Entering local search with global min distance " << globalDistance
                  << " and global min position " << globalPosition << std::endl;
        Position nextGlobalPosition;
        double nextGlobalDistance = globalDistance;
        for(auto&& direction : updateDirections)
        {
            cv::Mat localCrop;
            Position localPosition = getCroppedArea(frame,
                                                    globalPosition,
                                                    direction,
                                                    character,
                                                    &localCrop);
            cv::Mat localTransformedCrop;
            localCrop.copyTo(localTransformedCrop);
            FrameTransformer::removeGreyPixels(&localTransformedCrop);
            FrameTransformer::sortFramePixels(&localTransformedCrop); 
            double localDistance = norm(localTransformedCrop, refCharacterCrop);
            std::cout << "Local distance to " << direction << " " << localDistance << '\n'; 
            if (localDistance < nextGlobalDistance)
            {
                nextGlobalDistance = localDistance;
                nextGlobalPosition = localPosition;
                localCrop.copyTo(globalCrop); // FIXME straigth to nextCharacterCrop ?
                localTransformedCrop.copyTo(globalTransformedCrop);
                std::cout << "Picking " << direction << " move from " 
                          << globalPosition << " to " << localPosition 
                          << " with local distance " << localDistance << "\n";
            } 
        }

        std::cout << "New global min " << nextGlobalDistance << " at position "
                  << nextGlobalPosition << std::endl;
        globalPosition = nextGlobalPosition;
        descentStep = std::abs(nextGlobalDistance - globalDistance);
        globalDistance = nextGlobalDistance;
        coolingFactor *= 0.9;
    } while (descentStep > property::k_descentThreshold);
    std::cout << "Ending annealing search with position " << globalPosition
              << " with distance " << globalDistance 
              << " and descentStep " << descentStep << std::endl;

    globalCrop.copyTo(*nextCharacterCrop);
    globalTransformedCrop.copyTo(*nextSortedCharacterCrop);
    return globalPosition;
}


// That's where the magic happens... 
template <typename FirstCharacter,
          typename SecondCharacter,
          typename Measure>
void getCharacterTrajectories(const std::string&     videoFilename,
                              std::vector<Position> *trajectoryP1,
                              FirstCharacter         characterP1,
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
    cv::Mat characterP1Crop;

    // Prime the pump, starting position is fixed 
	cv::Mat extractedFrame;
    size_t currentlyTrackedFrame  = 0;
    size_t prevTrackedFrame = 0;
	frameExtractor >> extractedFrame;
    frameExtractor.discardNextNFrames(property::k_trackStep);
    
    // Translate from top left corner to center of the character
    (*trajectoryP1)[currentlyTrackedFrame] = { characterP1.p1StartX
                                               + characterP1.width / 2,
                                               characterP1.p1StartY 
                                               + characterP1.height / 2};
	getStartingCharVectors(extractedFrame,
	                       &characterP1Crop,
	                       characterP1);
    FrameTransformer::removeGreyPixels(&characterP1Crop);
    FrameTransformer::sortFramePixels(&characterP1Crop);

    while (!frameExtractor.isLastFrame())
    {
        std::cout << "Frame " << currentlyTrackedFrame << std::endl;
        cv::Mat nextCrop; 
        cv::Mat nextSortedCrop; 
        Position nextPosition = simulatedAnnealingSearch(extractedFrame,
                                                         characterP1Crop,
                                                         (*trajectoryP1)[currentlyTrackedFrame],
                                                         characterP1,
                                                         &nextCrop,
                                                         &nextSortedCrop);
        (*trajectoryP1)[currentlyTrackedFrame] = nextPosition;
        extractedFrameFilename << property::k_frameOutputFolder
                               << outFilePrefix
                               << std::setw(10) << std::setfill('0')
                               << currentlyTrackedFrame << ".jpg";
        prevTrackedFrame = currentlyTrackedFrame;
        nextCrop.reshape(characterP1.width, characterP1.height);
        cv::imwrite(extractedFrameFilename.str(), nextCrop);
        cv::imwrite(extractedFrameFilename.str() + "_sorted.jpg", nextSortedCrop);
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
