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
#include <numeric>
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

void saveTrajectoryToFile(const std::vector<Position>& trajectory,
                          const std::string& filename);

std::vector<Position> getTrajectoryFromFile(const std::string& filename);

// Add a small visual clue representing detected position
void addTracker(cv::Mat *frame,
                const std::vector<Position>& trajectory,
                size_t frameIdx);

// Overlay the detected trajectory on top of the video
void addTrajectoryToVideo(const std::string& inVideoFilename,
                          const std::string& outVideoFilename,
                          const std::vector<Position>& trajectory);

void addTrajectoryToVideo(const std::string& inVideoFilename,
                          const std::string& outVideoFilename,
                          const std::string& trajectoryFilename);

// Mark as valid, points whose distance are no further than 1 stddev from the mean
void removeStatisticalUnderliers(const std::vector<double>& distances,
                                 std::vector<size_t> *validFrames,
                                 std::vector<size_t> *unmatchedFrames);


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
    int32_t prevX = prevPosition.x - static_cast<int32_t>(character.cropSize / 2);
    int32_t prevY = prevPosition.y - static_cast<int32_t>(character.cropSize / 2);

    bool goRight = (updateDirection & 1) != 0;
    bool goLeft  = (updateDirection & 2) != 0;
    bool goUp    = (updateDirection & 4) != 0;
    bool goDown  = (updateDirection & 8) != 0;

    size_t offsetH = offsetScaleDownRatio * ((goRight || goLeft) ? character.cropSize / 2 : 0);
    size_t offsetV = offsetScaleDownRatio * ((goUp || goDown) ? character.cropSize / 2 : 0);

    int32_t newX = prevX + (goLeft  ? -offsetH :
                            goRight ?  offsetH :
                            0);
    int32_t newY = prevY + (goDown ?  offsetV :
                            goUp   ? -offsetV  :
                            0);

    // clip to min max
    // FIXME hardcoded video size
    newX = std::max<int32_t>(newX, 0);
    newX = std::min<int32_t>(newX, static_cast<int32_t>(1280 - character.cropSize));
    newY = std::max<int32_t>(newY, 0);
    newY = std::min<int32_t>(newY, static_cast<int32_t>(720 - character.cropSize));
    
    std::cout << "getShiftedCrop: " << updateDirection << ": X " << prevX << "->" << newX 
              << ", Y " << prevY << "->" << newY << '\n';

    // Is it shit ? Yep...
    // TODO Benchmark against other approach
    frame(cv::Rect(newX,
                   newY,
                   character.cropSize,
                   character.cropSize))
        .clone()
        .reshape(0, 1)
        .copyTo(*croppedROI);        

    return { static_cast<double>(newX + character.cropSize / 2), 
             static_cast<double>(newY + character.cropSize / 2) };
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
         x < 1280 - character.cropSize; 
         x += character.cropSize / offsetScaleDownRatio)
    {
        for (size_t y = 0; 
             y < 720 - character.cropSize; 
             y += character.cropSize / (offsetScaleDownRatio /* Todo account for crop h/w */))
        {
            frame(cv::Rect(x,
                           y,
                           character.cropSize,
                           character.cropSize))
                .clone()
                .reshape(0, 1)
                .copyTo(scratchVect);
            transformer(&scratchVect);
            double localDistance = norm(scratchVect, refCharacterCrop);
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
    *minDistance = globalDistance;
    return globalPosition;
}

template<typename Character> 
std::map<Position, double> 
    getFullScreenDistribution(const cv::Mat&                  frame,
                   const cv::Mat&                  refCharacterCrop,
                   const Character&                character,
                   const FrameTransformerPipeline& transformer,
                   double                         *maxDistance,
                   double                          offsetScaleDownRatio = 2.0)
{
    cv::Mat scratchVect;
    std::map<Position, double> distances;
    double globalMax = std::numeric_limits<double>::min();
    for (size_t x = 0; 
         x < 1280 - character.cropSize; 
         x += character.cropSize / offsetScaleDownRatio)
    {
        for (size_t y = 0; 
             y < 720 - character.cropSize; 
             y += character.cropSize / (offsetScaleDownRatio /* Todo account for crop h/w */))
        {
            frame(cv::Rect(x,
                           y,
                           character.cropSize,
                           character.cropSize))
                .clone()
                .reshape(0, 1)
                .copyTo(scratchVect);
            transformer(&scratchVect);
            double localDistance = norm(scratchVect, refCharacterCrop);
            globalMax = std::max(localDistance, globalMax);
            distances[Position{static_cast<double>(x), 
                               static_cast<double>(y)}] = globalMax;
        } 
    }
    *maxDistance = globalMax;
    return distances;
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
                              bool                   saveUnmatchedFrame = true)
{
	FrameExtractor frameExtractor(videoFilename, true);
    if (!frameExtractor) {
        std::cerr << "Error opening '" << videoFilename << "'\n";
        return;
    }
   
    FrameTransformerPipeline vectTransformer{removeGreyPixels,
                                             sortFramePixels};
                                             // removeBlackPixel TODO
    // Holds the x,y at the center of the character
    trajectoryP1->resize(frameExtractor.nbOfFrames());
    // Holds the frames idx with valid match and frames with no matches found
    std::vector<size_t> validFrames;
    std::vector<size_t> unmatchedFrames;
    // Holds the distance to best match at each frames
    std::vector<double> distances(frameExtractor.nbOfFrames());

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
    size_t currentlyTrackedFrame = 0;
    size_t prevTrackedFrame      = 0;
    
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
        nextPosition = scanFullScreen(extractedFrame,
                              characterP1Vect,
                              characterP1,
                              vectTransformer,
                              &distanceToBestMatch); 
        (*trajectoryP1)[currentlyTrackedFrame] = nextPosition;
        distances[currentlyTrackedFrame] = distanceToBestMatch;
        prevTrackedFrame = currentlyTrackedFrame;
    }
    trajectoryP1->resize(property::k_sampleOutput);
    distances.resize(property::k_sampleOutput);
    
    removeStatisticalUnderliers(distances,
                                &validFrames,
                                &unmatchedFrames);
    interpolatePositionInTrajectory(trajectoryP1,
                                    validFrames);
    saveTrajectoryToFile(*trajectoryP1, "testUrienSample.txt");

}


template <typename Character>
void getReferenceCharVect(cv::Mat         *characterVector,
                          const Character& character,
                          size_t           colorIdx)
{
    std::ostringstream filename;
    filename << property::k_referenceCropInFolder
             << character.name << "_"
             << "c" << colorIdx << "_"
             << "1" /* this one should loop over various crops */
             << ".png";
    std::cout << "Loading reference character '"
              << character.name
              << "' from '" 
              << filename.str() << '\'' << std::endl;
    
    cv::imread(filename.str())
        .reshape(0, 1)
        .copyTo(*characterVector);
}

} // Sfvml::


#endif
