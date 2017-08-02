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
                        const property::MoveDirection& updateDirection,
                        const Character& character,
                        cv::Mat* croppedROI)
    {
        
        // TODO change hardcoded 1200 * 720
        int32_t newX = prevPosition.x + (updateDirection == property::MoveDirection::k_LEFT_UPDATE ? -character.velocity :
                                        updateDirection == property::MoveDirection::k_RIGHT_UPDATE ? character.velocity :
                                        0);
        int32_t newY = prevPosition.y + (updateDirection == property::MoveDirection::k_DOWN_UPDATE ? character.velocity :
                                        updateDirection == property::MoveDirection::k_UP_UPDATE ? -character.velocity :
                                        0);
    
        // clip to min max
        newX = std::max<int32_t>(newX, 0);
        newX = std::min<int32_t>(newX, 1200 - character.width);
        newY = std::max<int32_t>(newY, 0);
        newY = std::min<int32_t>(newY, 720 - character.height);
        
        std::cout << updateDirection << ":" <<prevPosition.x << "->" << newX << std::endl;
        std::cout << updateDirection << ":" << prevPosition.y << "->" << newY << std::endl;
        
        frame(cv::Rect(newX,
                       newY,
                       character.width,
                       character.height)).copyTo(*croppedROI);        
        croppedROI->resize(character.width * character.height);
        return { static_cast<double>(newX), static_cast<double>(newY) };
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
		FrameExtractor frameExtractor(videoFilename);
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
            if (frameExtractor.currentFrame() % property::k_trackStep != 0) {
                continue;
            }
            
            // TODO character 2 ?!

            // Check surrounding crops
            // TODO Factor out
            cv::Mat identUpdate;
            cv::Mat rightUpdate;
            cv::Mat leftUpdate;
            cv::Mat upUpdate;
            cv::Mat downUpdate;

            Position identPosition = getCroppedArea(extractedFrame,
                (*trajectoryCharacter1)[previousTrackedFrame],
                property::MoveDirection::k_IDENT_UPDATE,
                firstCharacter,
                &identUpdate);

            Position rightPosition = getCroppedArea(extractedFrame,
                           (*trajectoryCharacter1)[previousTrackedFrame],
                           property::MoveDirection::k_RIGHT_UPDATE,
                           firstCharacter,
                           &rightUpdate);

            Position leftPosition = getCroppedArea(extractedFrame,
                (*trajectoryCharacter1)[previousTrackedFrame],
                property::MoveDirection::k_LEFT_UPDATE,
                firstCharacter,
                &leftUpdate);

            Position downPosition = getCroppedArea(extractedFrame,
                (*trajectoryCharacter1)[previousTrackedFrame],
                property::MoveDirection::k_DOWN_UPDATE,
                firstCharacter,
                &downUpdate);

            Position upPosition = getCroppedArea(extractedFrame,
                (*trajectoryCharacter1)[previousTrackedFrame],
                property::MoveDirection::k_UP_UPDATE,
                firstCharacter,
                &upUpdate);
            
            // Update with lowest distance
            double identDistance = cv::norm(identUpdate, firstCharacterVector);
            double rightDistance = cv::norm(rightUpdate, firstCharacterVector);
            double leftDistance  = cv::norm(leftUpdate, firstCharacterVector);
            double upDistance = cv::norm(upUpdate, firstCharacterVector);
            double downDistance = cv::norm(downUpdate, firstCharacterVector);

            double minDistance = identDistance;
            cv::Mat* closestUpdate = &identUpdate;
            (*trajectoryCharacter1)[lastExtractedFrame] = identPosition;

            if (rightDistance < minDistance)
            {
                minDistance = rightDistance;
                closestUpdate = &rightUpdate;
                (*trajectoryCharacter1)[lastExtractedFrame] = rightPosition;
            }
            if (leftDistance < minDistance)
            {
                minDistance = leftDistance;
                closestUpdate = &leftUpdate;
                (*trajectoryCharacter1)[lastExtractedFrame] = leftPosition;
            }
            if (upDistance < minDistance)
            {
                minDistance = upDistance;
                closestUpdate = &upUpdate;
                (*trajectoryCharacter1)[lastExtractedFrame] = upPosition;
            }
            if (downDistance < minDistance)
            {
                minDistance = downDistance;
                closestUpdate = &downUpdate;
                (*trajectoryCharacter1)[lastExtractedFrame] = downPosition;
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


template <
		typename FirstCharacter,
		typename SecondCharacter
	>
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
