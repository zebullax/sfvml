#ifndef SFVML_CHARACTERTRACKER
#define SFVML_CHARACTERTRACKER

// sfvml
#include "sfvml_charactertraits.h"
#include "sfvml_property.h"
#include "sfvml_type.h"
#include "sfvml_frameextractor.h"
#include "sfvml_frametransformer.h"
#include "sfvml_frametransformerpipeline.h"
#include "sfvml_trajectory.h"
// std
#include <array>
#include <vector>
#include <cmath>
#include <fstream>
#include <numeric>
// opencv
#include "opencv2/opencv.hpp"

namespace Sfvml {

class CharacterTracker
{

public:

    // CREATORS
    CharacterTracker() = default;
 
    // UTIL
    template<CharacterName name>
    void extractTrajectoryP1FromVideo(const std::string&   videoFilename,
                                      CharacterTrait<name> character, 
                                      Trajectory          *trajectory,
                                      bool                 saveUnmatchedFrame = true);

private:
    
    // UTIL

    // Do a linear scan of the frame with a sliding window to find the best match
    // wrt. the character index-th reference crop.
    // Return the top left corner of the best position found if the match score
    // is under the threshold set in the character, otherwise 'k_characterNotFound' 
    template<CharacterName characterName> 
    Position scanFullScreen(const cv::Mat&                  frame,
                            CharacterTrait<characterName>   character,
                            size_t                          cropIdx,
                            const FrameTransformerPipeline& transformer,
                            double                          *minDistance,
                            double                          offsetScaleDownRatio = 2.0);
    
    // Load all reference crops for the specified character using its color idx
    // FIXME what about alternative costume.... 
    template<CharacterName characterName> 
    void loadReferenceCharVect(size_t                         colorIdx,
                               CharacterTrait<characterName> *character);

    // DATA
    std::vector<cv::Mat> refCrops; // !! crop is assumed to be square (#cols = #rows)
    std::vector<size_t>  refCropsSizes;
};

// Mark as valid, points whose distance are no further than stddevOffset* stddev 
// from the mean, invalid the others
void removeStatisticalUnderliers(const std::vector<double>& distances,
                                 std::vector<size_t>       *validFrames,
                                 std::vector<size_t>       *unmatchedFrames,
                                 double                     stddevOffset = 1.5);

// Used when matching several reference crops
// Holds the position for the best match and the min distance value
using CropMatch = std::pair<Position, double>; 
Position getBarycentre(const std::vector<CropMatch>& CropMatches);

// Do a linear scan of the frame with a sliding window to find the best match
// Return the top left corner of the best position found if the match score
// is under the threshold set in the character, otherwise 'k_characterNotFound'
template<CharacterName characterName> 
Position CharacterTracker::scanFullScreen(const cv::Mat&                  frame,
                                          CharacterTrait<characterName>   character,
                                          size_t                          cropIdx,
                                          const FrameTransformerPipeline& transformer,
                                          double                          *minDistance,
                                          double                          offsetScaleRatio)
{
    cv::Mat scratchVect;
    cv::Mat refCharacterCrop = refCrops[cropIdx];
    size_t cropSize          = refCropsSizes[cropIdx]; 
    std::cout << "Entering linear scan of the frame " << cropSize << std::endl;
    double globalDistance = std::numeric_limits<double>::max();
    Position globalPosition;
    for (size_t x = 0; 
         x < 1280 - cropSize; 
         x += cropSize / offsetScaleRatio)
    {
        for (size_t y = 0; 
             y < 720 - cropSize; 
             y += cropSize / (offsetScaleRatio /* Todo account for crop h/w */))
        {
            frame(cv::Rect(x,
                           y,
                           cropSize,
                           cropSize))
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

template<CharacterName name>
void CharacterTracker::extractTrajectoryP1FromVideo(const std::string&   videoFilename,
                                                    CharacterTrait<name> character, 
                                                    Trajectory          *trajectory,
                                                    bool                 saveUnmatchedFrame)

{
	FrameExtractor frameExtractor(videoFilename, true);
    if (!frameExtractor) {
        std::cerr << "Error opening '" << videoFilename << "'\n";
        return;
    }
   
    FrameTransformerPipeline vectTransformer{removeGreyPixels,
                                             sortFramePixels};
    
    // Holds the x,y at the center of the character
    trajectory->resize(frameExtractor.nbOfFrames());
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
    cv::Mat characterVect;

    // Prime the pump, starting position is fixed 
	cv::Mat extractedFrame;
    size_t currentlyTrackedFrame = 0;
    size_t prevTrackedFrame      = 0;
    
    loadReferenceCharVect(1, &character);
    
    for(auto&& crop: refCrops) {
        vectTransformer(&crop);
    }
    // Used when matching individual crops, where we keep the position and 
    // the match distance
    std::vector<CropMatch> cropPositions(refCropsSizes.size());

    // For all frames we do not skip...
    while (!frameExtractor.isLastFrame() 
            && currentlyTrackedFrame < property::k_sampleOutput)
    {
        frameExtractor >> extractedFrame;
        // current points to one past read, hence - 1 
        currentlyTrackedFrame = frameExtractor.currentFrame() - 1;         
        frameExtractor.discardNextNFrames(property::k_trackStep);
        
        std::cout << "\n================== Frame " << currentlyTrackedFrame 
                  << " (from " << prevTrackedFrame << ")" << std::endl;
        
        double distanceToBestMatch = std::numeric_limits<double>::max();

        // For all our reference crops...
        for(size_t i = 0; i != refCrops.size(); ++i)
        {
            double cropDistance;
            Position cropPosition = scanFullScreen(extractedFrame,
                                                   character,
                                                   i,
                                                   vectTransformer,
                                                   &cropDistance);
            distanceToBestMatch = std::min(distanceToBestMatch, cropDistance);
            cropPositions[i] = {cropPosition, cropDistance};
        }

        (*trajectory)[currentlyTrackedFrame] = getBarycentre(cropPositions);
        distances[currentlyTrackedFrame] = distanceToBestMatch;
        prevTrackedFrame = currentlyTrackedFrame;
    }
    trajectory->resize(property::k_sampleOutput);
    distances.resize(property::k_sampleOutput);
   
    removeStatisticalUnderliers(distances,
                                &validFrames,
                                &unmatchedFrames);
    trajectory->interpolatePosition(validFrames);
    trajectory->saveToFile("testUrienSample.txt");

}


template<CharacterName characterName> 
void CharacterTracker::loadReferenceCharVect(size_t                         colorIdx,
                                             CharacterTrait<characterName> *character)
{
    for(size_t cropIdx = 0; /* none */; ++cropIdx)
    {
        std::ostringstream filename;
        filename << property::k_referenceCropInFolder
                 << character->name << "_"
                 << "c" << colorIdx << "_"
                 << cropIdx 
                 << ".png";
        {
            std::ifstream ifs(filename.str());
            if (!ifs) {
                break;
            }
        }
        std::cout << "Loading reference crop for '"
                  << character->name
                  << "' from '" 
                  << filename.str() << '\'' << std::endl;
        cv::Mat refCrop = cv::imread(filename.str());
        refCropsSizes.push_back(refCrop.cols); // Assumed = rows
        refCrops.push_back(refCrop.reshape(0, 1).clone());
    } 
}

} // Sfvml::


#endif
