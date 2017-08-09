#ifndef SFVML_CHARACTERTRACKER
#define SFVML_CHARACTERTRACKER

// sfvml
#include "sfvml_charactertraits.h"
#include "sfvml_property.h"
#include "sfvml_type.h"
#include "sfvml_frameextractor.h"
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
    int32_t prevX = prevPosition.x - character.width / 2;
    int32_t prevY = prevPosition.y - character.height / 2;


    bool goRight = updateDirection & 1;
    bool goLeft  = updateDirection & 2;
    bool goUp    = updateDirection & 4;
    bool goDown  = updateDirection & 8;

    // TODO change hardcoded 1200 * 720
    int32_t newX = prevX + (goLeft  ? -character.velocity :
                            goRight ?  character.velocity :
                            0);
    int32_t newY = prevY + (goDown ?  character.velocity :
                            goUp   ? -character.velocity :
                            0);

    // clip to min max
    newX = std::max<int32_t>(newX, 0);
    newX = std::min<int32_t>(newX, 1200 - character.width);
    newY = std::max<int32_t>(newY, 0);
    newY = std::min<int32_t>(newY, 720 - character.height);
    
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
          typename SecondCharacter>
void getCharacterTrajectories(const std::string&     videoFilename,
                              std::vector<Position> *trajectoryCharacter1,
                              FirstCharacter         firstCharacter,
                              std::vector<Position> *trajectoryCharacter2,
                              SecondCharacter        secondCharacter)
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
    cv::Mat firstCharacterVector;
	cv::Mat secondCharacterVector;

	cv::Mat extractedFrame;
    
    // Prime the pump, starting position is fixed 
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
	                       &firstCharacterVector,
	                       firstCharacter,
	                       &secondCharacterVector,
	                       secondCharacter);
    
    // TODO Add progress status
    while (!frameExtractor.isLastFrame())
    {
        frameExtractor >> extractedFrame;
        lastExtractedFrame = frameExtractor.currentFrame();
        frameExtractor.discardNextNFrames(property::k_trackStep);

        // TODO character 2 ?!

        // Check surrounding crops
        // TODO Factor out
        cv::Mat identUpdate;
        cv::Mat rightUpdate;
        cv::Mat leftUpdate;
        cv::Mat upUpdate;
        cv::Mat downUpdate; 
        cv::Mat upRightUpdate;
        cv::Mat upLeftUpdate;
        cv::Mat downRightUpdate;
        cv::Mat downLeftUpdate;

        Position identPosition = getCroppedArea(extractedFrame,
                                 (*trajectoryCharacter1)[previousTrackedFrame],
                                 Direction::k_IDENT,
                                 firstCharacter,
                                 &identUpdate);

        Position rightPosition = getCroppedArea(extractedFrame,
                       (*trajectoryCharacter1)[previousTrackedFrame],
                       Direction::k_RIGHT,
                       firstCharacter,
                       &rightUpdate);

        Position leftPosition = getCroppedArea(extractedFrame,
            (*trajectoryCharacter1)[previousTrackedFrame],
            Direction::k_LEFT,
            firstCharacter,
            &leftUpdate);

        Position downPosition = getCroppedArea(extractedFrame,
            (*trajectoryCharacter1)[previousTrackedFrame],
            Direction::k_DOWN,
            firstCharacter,
            &downUpdate);

        Position upPosition = getCroppedArea(extractedFrame,
            (*trajectoryCharacter1)[previousTrackedFrame],
            Direction::k_UP,
            firstCharacter,
            &upUpdate);
        
        Position upRightPosition = getCroppedArea(extractedFrame,
            (*trajectoryCharacter1)[previousTrackedFrame],
            Direction::k_UPRIGHT,
            firstCharacter,
            &upRightUpdate);
        
        Position upLeftPosition = getCroppedArea(extractedFrame,
            (*trajectoryCharacter1)[previousTrackedFrame],
            Direction::k_UPLEFT,
            firstCharacter,
            &upLeftUpdate);
        
        Position downLeftPosition = getCroppedArea(extractedFrame,
            (*trajectoryCharacter1)[previousTrackedFrame],
            Direction::k_DOWNLEFT,
            firstCharacter,
            &downLeftUpdate);
        
        Position downRightPosition = getCroppedArea(extractedFrame,
            (*trajectoryCharacter1)[previousTrackedFrame],
            Direction::k_DOWNRIGHT,
            firstCharacter,
            &downRightUpdate);
        
        // Update with lowest distance
        double identDistance = cv::norm(identUpdate, firstCharacterVector);
        double rightDistance = cv::norm(rightUpdate, firstCharacterVector);
        double leftDistance  = cv::norm(leftUpdate, firstCharacterVector);
        double upDistance    = cv::norm(upUpdate, firstCharacterVector);
        double downDistance  = cv::norm(downUpdate, firstCharacterVector); 
        double upRightDistance = cv::norm(upRightUpdate, firstCharacterVector);
        double upLeftDistance  = cv::norm(upLeftUpdate, firstCharacterVector);
        double downLeftDistance    = cv::norm(downLeftUpdate, firstCharacterVector);
        double downRightDistance  = cv::norm(downRightUpdate, firstCharacterVector);

        double minDistance = identDistance;
        std::cout << "Picking no move\n";
        cv::Mat* closestUpdate = &identUpdate;
        (*trajectoryCharacter1)[lastExtractedFrame] = identPosition;

        if (rightDistance < minDistance)
        {
            std::cout << "Picking right move\n" ;
            minDistance = rightDistance;
            closestUpdate = &rightUpdate;
            (*trajectoryCharacter1)[lastExtractedFrame] = rightPosition;
        }
        if (leftDistance < minDistance)
        {
            std::cout << "Picking left move\n" ;
            minDistance = leftDistance;
            closestUpdate = &leftUpdate;
            (*trajectoryCharacter1)[lastExtractedFrame] = leftPosition;
        }
        if (upDistance < minDistance)
        {
            std::cout << "Picking up move\n" ;
            minDistance = upDistance;
            closestUpdate = &upUpdate;
            (*trajectoryCharacter1)[lastExtractedFrame] = upPosition;
        }
        if (downDistance < minDistance)
        {
            std::cout << "Picking down move\n" ;
            minDistance = downDistance;
            closestUpdate = &downUpdate;
            (*trajectoryCharacter1)[lastExtractedFrame] = downPosition;
        }
        
        if (downLeftDistance < minDistance)
        {
            std::cout << "Picking downLeft move\n" ;
            minDistance = downLeftDistance;
            closestUpdate = &downLeftUpdate;
            (*trajectoryCharacter1)[lastExtractedFrame] = downLeftPosition;
        }
        if (downRightDistance < minDistance)
        {
            std::cout << "Picking downRight move\n" ;
            minDistance = downRightDistance;
            closestUpdate = &downRightUpdate;
            (*trajectoryCharacter1)[lastExtractedFrame] = downRightPosition;
        }
        if (upLeftDistance < minDistance)
        {
            std::cout << "Picking upLeft move\n" ;
            minDistance = upLeftDistance;
            closestUpdate = &upLeftUpdate;
            (*trajectoryCharacter1)[lastExtractedFrame] = upLeftPosition;
        }
        if (downRightDistance < minDistance)
        {
            std::cout << "Picking downRight move\n" ;
            minDistance = downRightDistance;
            closestUpdate = &downRightUpdate;
            (*trajectoryCharacter1)[lastExtractedFrame] = downRightPosition;
        }

        closestUpdate->copyTo(firstCharacterVector);
        extractedFrameFilename << property::k_frameOutputFolder
                               << outFilePrefix
                               << std::setw(10) << std::setfill('0')
                               << lastExtractedFrame << ".jpg";
        previousTrackedFrame = lastExtractedFrame;
        firstCharacterVector.resize(firstCharacter.height);
        cv::imwrite(extractedFrameFilename.str(), firstCharacterVector);
        firstCharacterVector.resize(firstCharacter.height * firstCharacter.width);
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
