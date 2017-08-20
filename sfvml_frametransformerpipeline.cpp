#include "sfvml_frametransformerpipeline.h"

namespace Sfvml {

FrameTransformerPipeline::FrameTransformerPipeline(std::initializer_list<FrameTransformerFunction> transformers): d_transformers(transformers)
{
       
}

void FrameTransformerPipeline::addTransformer(const FrameTransformerFunction& transformer) {
    d_transformers.push_back(transformer);
}

void FrameTransformerPipeline::addTransformer(FrameTransformerFunction&& transformer) 
{ 
    d_transformers.push_back(std::move(transformer));
}

void FrameTransformerPipeline::operator() (cv::Mat* frame) const
{
    for (auto&& transformer : d_transformers) {
        transformer(frame);
    }
}


} // Sfvml::
