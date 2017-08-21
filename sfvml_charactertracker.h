#ifndef SFVML_CHARACTERTRACKER
#define SFVML_CHARACTERTRACKER

// sfvml
#include "sfvml_charactertraits.h"
#include "sfvml_property.h"
#include "sfvml_type.h"
#include "sfvml_frameextractor.h"
#include "sfvml_frametransformer.h"
#include "sfvml_frametransformerpipeline.h"
// std
#include <array>
#include <vector>
#include <cmath>
#include <utility>
#include <iomanip>
// opencv
#include "opencv2/opencv.hpp"

namespace Sfvml {

// Linearly interpolate character positions between 2 keyframe
// trajectoryP1 -> vector holding the positions of the character 
// prevIdx -> last known position
// currIdx -> most recently detected position
void interpolatePositionGap(std::vector<Position> *trajectoryP1,
                            size_t prevIdx, 
                            size_t currIdx);

void interpolatePositionInTrajectory(std::vector<Position> *trajectoryP1,
                                     const std::vector<size_t>& validFrames);

// Add a small visual clue representing detected position
void addTracker(cv::Mat *frame,
                const std::vector<Position>& trajectory,
                size_t frameIdx);

// OVerlay the detected trajectory on top of the video
void addTrajectoryToVideo(const std::string& inVideoFilename,
                          const std::string& outVideoFilename,
                          const std::vector<Position>& trajectory);

bool detectPreRound(const cv::Mat& frame);

// Return a shifter crop of the image relative to prevPosition
// The return cropped is a row vector
template <typename Character>
Position getShiftedCrop(const cv::Mat&    frame,
                        const Position&   prevPosition,
                        const Direction&  updateDirection,
                        const Character&  character,
                        cv::Mat          *croppedROI,
                        double            offsetScaleDownRatio = 1.0)
{
   
    // Translate from center of character to top left corner
    int32_t prevX = prevPosition.x - static_cast<int32_t>(character.width / 2);
    int32_t prevY = prevPosition.y - static_cast<int32_t>(character.height / 2);

    bool goRight = (updateDirection & 1) != 0;
    bool goLeft  = (updateDirection & 2) != 0;
    bool goUp    = (updateDirection & 4) != 0;
    bool goDown  = (updateDirection & 8) != 0;

    size_t offsetH = offsetScaleDownRatio * ((goRight || goLeft) ? character.width / 2 : 0);
    size_t offsetV = offsetScaleDownRatio * ((goUp || goDown) ? character.height / 2 : 0);

    int32_t newX = prevX + (goLeft  ? -offsetH :
                            goRight ?  offsetH :
                            0);
    int32_t newY = prevY + (goDown ?  offsetV :
                            goUp   ? -offsetV  :
                            0);

    // clip to min max
    // FIXME hardcoded video size
    newX = std::max<int32_t>(newX, 0);
    newX = std::min<int32_t>(newX, static_cast<int32_t>(1280 - character.width));
    newY = std::max<int32_t>(newY, 0);
    newY = std::min<int32_t>(newY, static_cast<int32_t>(720 - character.height));
    
    std::cout << "getShiftedCrop: " << updateDirection << ": X " << prevX << "->" << newX 
              << ", Y " << prevY << "->" << newY << '\n';

    // Is it shit ? Yep...
    // TODO Benchmark against other approach
    frame(cv::Rect(newX,
                   newY,
                   character.width,
                   character.height))
        .clone()
        .reshape(0, 1)
        .copyTo(*croppedROI);        

    return { static_cast<double>(newX + character.width / 2), 
             static_cast<double>(newY + character.height / 2) };
}

// Do a linear scan of the frame with a sliding window to find the best match
// Return the top left corner of the best position found if the match score
// is under the threshold set in the character, otherwise 'k_characterNotFound'
template<typename Character>
Position scanFullScreen(const cv::Mat&    frame,
                        const cv::Mat&    refCharacterCrop,
                        const Character&  character,
                        const FrameTransformerPipeline& transformer,
                        double           *minDistance,
                        double                          offsetScaleDownRatio = 2.0)
{
    cv::Mat scratchVect;
    std::cout << "Entering linear scan of the frame" << std::endl;
    double globalDistance = std::numeric_limits<double>::max();
    Position globalPosition;
    for (size_t x = 0; 
         x < 1280 - character.width; 
         x += character.width / offsetScaleDownRatio)
    {
        for (size_t y = 0; 
             y < 720 - character.height; 
             y += character.height / (offsetScaleDownRatio /* Todo account for crop h/w */))
        {
            frame(cv::Rect(x,
                           y,
                           character.width,
                           character.height))
                .clone()
                .reshape(0, 1)
                .copyTo(scratchVect);
            transformer(&scratchVect);
            double localDistance = norm(scratchVect, refCharacterCrop);
            std::cout << "Distance at " << x << ',' << y 
                      << " = " <<localDistance << std::endl;
            if (localDistance < globalDistance)
            {
                std::cout << "New minimum at " << x << "," << y 
                          << " with distance " << localDistance << '\n';
                globalDistance = localDistance;
                globalPosition = {static_cast<double>(x), 
                                  static_cast<double>(y)};
            }
        } 
    }
    if (globalDistance > character.matchThreshold) {
        std::cout << "Failure to match while doing a full scan " << std::endl;
    }
    *minDistance = globalDistance;
    return (globalDistance > character.matchThreshold ? property::k_characterNotFound : 
                                                        globalPosition);
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
                              Measure                norm,
                              bool                   saveIntermediateCrop = true)
{
	FrameExtractor frameExtractor(videoFilename, true);
    if (!frameExtractor) {
        std::cerr << "Error opening '" << videoFilename << "'\n";
        return;
    }
   
    FrameTransformerPipeline vectTransformer{//removeGreyPixels,
                                             sortFramePixels};
                                                // removeBlackPixel TODO
    // Holds the x,y at the center of the character
    trajectoryP1->resize(frameExtractor.nbOfFrames());
    // Holds the frames idx with valid match and frames with no matches found
    std::vector<size_t> validFrames;
    std::vector<size_t> unmatchedFrames;
    // Holds the distance to best match at each frames
    std::vector<std::pair<size_t, double>> distances;

    std::string outFilePrefix(videoFilename);
    std::replace(outFilePrefix.begin(),
                 outFilePrefix.end(),
                 '.',
                 '_');
    std::ostringstream extractedFrameFilename;

	// Those will hold the square supposedly framing the characters
    cv::Mat characterP1Vect;

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
    validFrames.push_back(0); 
    // Load reference crops used to match subsequent crops
    getReferenceCharVect(&characterP1Vect,
	                     characterP1,
                         1);
    vectTransformer(&characterP1Vect);
    
    while (!frameExtractor.isLastFrame() && currentlyTrackedFrame < property::k_sampleOutput)
    {
        frameExtractor >> extractedFrame;
        // current points to one past read, hence - 1 
        currentlyTrackedFrame = frameExtractor.currentFrame() - 1;         
        frameExtractor.discardNextNFrames(property::k_trackStep);
        
        std::cout << "\n================== Frame " << currentlyTrackedFrame 
                  << " (from " << prevTrackedFrame << ")" << std::endl;

        Position nextPosition = (*trajectoryP1)[prevTrackedFrame];
        double distanceToBestMatch = .0;
        // If we detect a pre round sequence, we just copy over previous position
        if (!detectPreRound(extractedFrame))
        {
            nextPosition = scanFullScreen(extractedFrame,
                                  characterP1Vect,
                                  characterP1,
                                  vectTransformer,
                                  &distanceToBestMatch); 
        }
        (*trajectoryP1)[currentlyTrackedFrame] = nextPosition;
        // Save valid matches
        if (nextPosition != property::k_characterNotFound) {
            validFrames.push_back(currentlyTrackedFrame);
        }
        else {
            unmatchedFrames.push_back(currentlyTrackedFrame);
        }
        distances.emplace_back(currentlyTrackedFrame, distanceToBestMatch);
        prevTrackedFrame = currentlyTrackedFrame;
    }

    // TODO remove from prod
    for (auto&& u : unmatchedFrames) {
        std::cout << "Unmatched : " << u << std::endl;
    }
    for (auto&& d : distances){
        std::cout << "Best distance in frame " << d.first << " = " << d.second << std::endl;
    }
    interpolatePositionInTrajectory(trajectoryP1,
                                    validFrames);
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
    characterVector->reshape(0, 1);

}

template <typename Character>
void getReferenceCharVect(cv::Mat         *characterVector,
                          const Character& character,
                          size_t           colorIdx)
{
    std::ostringstream filename;
    filename << property::k_referenceCropFolder
             << character.name
             << "_ccc" << colorIdx 
             << ".png";
    std::cout << "Loading reference character '"
              << character.name
              << "' from '" 
              << filename.str() << '\'' << std::endl;
    
    cv::imread(filename.str())
        .reshape(0, 1)
        .copyTo(*characterVector);
}

template <CharacterName characterName>
void displayCharacterVect(const cv::Mat&                characterVector, 
                         CharacterTrait<characterName> characterTrait)
{
	cv::imshow(characterTrait.name, 
               characterVector.reshape(characterTrait.width, 
                                       characterTrait.height));
	cv::waitKey();
}

} // Sfvml::


#endif
