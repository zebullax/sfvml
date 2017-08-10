#ifndef SFVML_CHARACTERTRACKER
#define SFVML_CHARACTERTRACKER

// sfvml
#include "sfvml_charactertraits.h"
#include "sfvml_property.h"
#include "sfvml_type.h"
#include "sfvml_frameextractor.h"
#include "sfvml_frametransformer.h"
// std
#include <vector>
#include <cmath>
// opencv
#include "opencv2/opencv.hpp"

namespace sfvml
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

    // TODO change hardcoded 1200 * 720
    int32_t newX = prevX + (goLeft  ? -character.velocity :
                            goRight ?  character.velocity :
                            0);
    int32_t newY = prevY + (goDown ?  character.velocity :
                            goUp   ? -character.velocity :
                            0);

    // clip to min max
    newX = std::max<int32_t>(newX, 0);
    newX = std::min<int32_t>(newX, static_cast<int32_t>(1200 - character.width));
    newY = std::max<int32_t>(newY, 0);
    newY = std::min<int32_t>(newY, static_cast<int32_t>(720 - character.height));
    
    std::cout << updateDirection << ": X " << prevX << "->" << newX 
              << ", Y " << prevY << "->" << newY << std::endl;
    
    frame(cv::Rect(newX,
                   newY,
                   character.width,
                   character.height)).copyTo(*croppedROI);        
    croppedROI->resize(character.width * character.height);
    return { static_cast<double>(newX + character.width / 2), 
             static_cast<double>(newY + character.height / 2) };
}

// That's where the magic happens... 
template <typename FirstCharacter,
          typename SecondCharacter,
          typename Measure>
void getCharacterTrajectories(const std::string&     videoFilename,
                              std::vector<Position> *trajectoryCharacter1,
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
    trajectoryCharacter1->resize(frameExtractor.nbOfFrames());
    trajectoryCharacter2->resize(frameExtractor.nbOfFrames());

    std::string outFilePrefix(videoFilename);
    std::replace(outFilePrefix.begin(),
                 outFilePrefix.end(),
                 '.',
                 '_');
    std::ostringstream extractedFrameFilename(outFilePrefix);

	// Those will hold the square supposedly framing the characters
    cv::Mat firstCharacterCrop;
	cv::Mat secondCharacterCrop;

    // Prime the pump, starting position is fixed 
	cv::Mat extractedFrame;
    size_t lastExtractedFrame  = 0;
    size_t previousTrackedFrame = 0;
	frameExtractor >> extractedFrame;
    // Translate from top left corner to center of the character
    (*trajectoryCharacter1)[lastExtractedFrame] = { firstCharacter.p1StartX
                                                        + firstCharacter.width / 2,
                                                    firstCharacter.p1StartY 
                                                        + firstCharacter.height / 2};
    (*trajectoryCharacter2)[lastExtractedFrame] = { secondCharacter.p2StartX 
                                                        + secondCharacter.width / 2,
                                                    secondCharacter.p2StartY 
                                                        + secondCharacter.height / 2};
	getStartingCharVectors(extractedFrame,
	                       &firstCharacterCrop,
	                       firstCharacter,
	                       &secondCharacterCrop,
	                       secondCharacter);
    
    FrameExtractor::removeGrey(&firstCharacterCrop);
    FrameExtractor::removeGrey(&secondCharacterCrop);
    
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

    while (!frameExtractor.isLastFrame())
    {
        frameExtractor >> extractedFrame;
        FrameExtractor::removeGrey(&extractedFrame);
        lastExtractedFrame = frameExtractor.currentFrame();
        frameExtractor.discardNextNFrames(property::k_trackStep);

        // TODO character 2 ?!

        // Check surrounding crops and pick the minimal distance
        cv::Mat minCrop;
        Position minPosition = getCroppedArea(extractedFrame,
                                              (*trajectoryCharacter1)[previousTrackedFrame],
                                              Direction::k_IDENT,
                                              firstCharacter,
                                              &minCrop);
        std::cout << "Picking " << Direction::k_IDENT << " move from " << (*trajectoryCharacter1)[previousTrackedFrame] << " to " << minPosition << " \n";
        double minDistance = norm(minCrop, firstCharacterCrop);
        for(auto&& direction : updateDirections)
        {
            cv::Mat candidateCrop;
            Position candidatePosition = getCroppedArea(extractedFrame,
                                                        (*trajectoryCharacter1)[previousTrackedFrame],
                                                        direction,
                                                        firstCharacter,
                                                        &candidateCrop);
            double candidateDistance = norm(candidateCrop, firstCharacterCrop);
            if (candidateDistance < minDistance)
            {
                minDistance = candidateDistance;
                candidateCrop.copyTo(minCrop);
                minPosition = candidatePosition;
                std::cout << "Picking " << direction << " move from " << (*trajectoryCharacter1)[previousTrackedFrame] << " to " << minPosition << " \n";
            }            
        }

        (*trajectoryCharacter1)[lastExtractedFrame] = minPosition;
        minCrop.copyTo(firstCharacterCrop);
        extractedFrameFilename << property::k_frameOutputFolder
                               << outFilePrefix
                               << std::setw(10) << std::setfill('0')
                               << lastExtractedFrame << ".jpg";
        previousTrackedFrame = lastExtractedFrame;
        firstCharacterCrop.resize(firstCharacter.height);
        cv::imwrite(extractedFrameFilename.str(), firstCharacterCrop);
        firstCharacterCrop.resize(firstCharacter.height * firstCharacter.width);
        extractedFrameFilename.str("");
        // TODO interpolate in between
        
        //displayCharacter(firstCharacterVector, firstCharacter);
        //displayCharacter(secondCharacterVector, secondCharacter);
    }
}


template <typename FirstCharacter,
	      typename SecondCharacter>
void getStartingCharVectors(const cv::Mat&  firstFrame,
                            cv::Mat        *firstCharacterVector,
                            FirstCharacter  firstCharacter,
                            cv::Mat        *secondCharacterVector,
                            SecondCharacter secondCharacter)
{

	// First character		
	firstFrame(cv::Rect(firstCharacter.p1StartX, 
                        firstCharacter.p1StartY, 
                        firstCharacter.width, 
                        firstCharacter.height)).copyTo(*firstCharacterVector);
    firstCharacterVector->resize(firstCharacter.width * firstCharacter.height);

    // Second character
	firstFrame(cv::Rect(secondCharacter.p2StartX, 
                        secondCharacter.p2StartY, 
                        secondCharacter.width, 
                        secondCharacter.height)).copyTo(*secondCharacterVector);
    secondCharacterVector->resize(secondCharacter.width * secondCharacter.height);

}

template <CharacterName characterName>
void displayCharacter(const cv::Mat&                characterVector, 
                      CharacterTrait<characterName> characterTrait)
{
	cv::Mat reconstructedImg;
    characterVector.copyTo(reconstructedImg);
    reconstructedImg.resize(characterTrait.height);
	cv::imshow(characterTrait.name, reconstructedImg);
	cv::waitKey();
}

} // sfvml::


#endif
