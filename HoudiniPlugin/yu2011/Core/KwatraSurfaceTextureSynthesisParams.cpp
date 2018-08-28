#include "KwatraSurfaceTextureSynthesisParams.h"
#include <cassert>
#include <iostream>

using namespace Mokko;

//=====================
// DEFAULT CONSTRUCTOR
//=====================

KwatraSurfaceTextureSynthesisParams::KwatraSurfaceTextureSynthesisParams()
{
    setOutputImageSize(1);
}

//==========
// MUTATORS
//==========

void KwatraSurfaceTextureSynthesisParams::setAverageAreas(const std::vector<float> &areas)
{
    assert(areas.size() > 0);
    averageAreas = areas;

    if (outputStateChanges)
    {
        std::cout << "[DEBUG] New average areas (per level): [ ";
        for (std::vector<float>::const_iterator currentArea = averageAreas.begin(); currentArea != averageAreas.end(); ++currentArea)
            std::cout << *currentArea << " ";
        std::cout << "]" << std::endl;
    }
}

//=================
// COMPUTED VALUES
//=================

float KwatraSurfaceTextureSynthesisParams::getCellSize(int level) const
{
    assert(level >= 0 && level < averageAreas.size());
    return getScalingFactor(level);
}

std::vector<float> KwatraSurfaceTextureSynthesisParams::getCellSizes() const
{
    std::vector<float> cellSizes;

    for (int i = 0; i < averageAreas.size(); ++i)
        cellSizes.push_back(getCellSize(i));

    if (outputStateChanges)
    {
        std::cout << "[DEBUG] Cell sizes (per level): [ ";
        for (std::vector<float>::const_iterator currentSize = cellSizes.begin(); currentSize != cellSizes.end(); ++currentSize)
            std::cout << *currentSize << " ";
        std::cout << "]" << std::endl;
    }

    return cellSizes;
}

float KwatraSurfaceTextureSynthesisParams::getScalingFactor(int level) const
{
    assert(level >= 0 && level < averageAreas.size());
    return std::sqrt(2.f * averageAreas[level]);
}

float KwatraSurfaceTextureSynthesisParams::getNghbrhdRadius(int level) const
{
    assert(level >= 0 && level < averageAreas.size());
    return getScalingFactor(level) * (getCurrentKernelSize() - 1.f) / 2.f;
            //+ 2.f * getScalingFactor(level);
}

float KwatraSurfaceTextureSynthesisParams::getUpsampleRadius(int level) const
{
    assert(level >= 0 && level < averageAreas.size());
    return getScalingFactor(level);
}
