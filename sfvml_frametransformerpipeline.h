#ifndef SFVML_FRAMETRANSFORMERPIPELINE
#define SFVML_FRAMETRANSFORMERPIPELINE

// std
#include <vector>
#include <functional>
// opencv
#include "opencv2/opencv.hpp"

namespace Sfvml {

using FrameTransformerFunction = std::function<void(cv::Mat*)>;

struct FrameTransformerPipeline 
{
    // CREATORS

    FrameTransformerPipeline() = default;

    FrameTransformerPipeline(std::initializer_list<FrameTransformerFunction> transformers);

    // MODIFICATORS
    
    void addTransformer(const FrameTransformerFunction& transformer);

    void addTransformer(FrameTransformerFunction&& transformer);

    void operator() (cv::Mat* fram) const;


private:

    // DATA
    std::vector<FrameTransformerFunction> d_transformers;
};

} // Sfvml::

#endif
