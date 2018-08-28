#include "KwatraTextureSynthesisParams.h"
#include <cassert>
#include <cmath>

using namespace Mokko;

bool KwatraTextureSynthesisParams::outputStateChanges = false;

//=====================
// DEFAULT CONSTRUCTOR
//=====================

KwatraTextureSynthesisParams::KwatraTextureSynthesisParams() :
    outputImageSize(64), inputImageSize(-1, -1), currentKernelSizeLevel(0),
    maxIterationCount(3), overlappingFactor(4),
    breakOnConvergence(false), usesWarping(false), softConstraintFactor(0.5),
    regressorValue(0.8)
{
    // Default kernel sizes
    kernelSizes.push_back(64);
    kernelSizes.push_back(32);
    kernelSizes.push_back(16);

    // Compute the generated attributes
    computeFalloffKernel();
    computeNghbrhdInterval();
    computeNghbrhdSize();
}

//=================
// COMPUTE METHODS
//=================

void KwatraTextureSynthesisParams::computeFalloffKernel()
{
    falloffKernel = cv::Mat_<double>(getCurrentKernelSize(), getCurrentKernelSize());

    // TODO Sigma should be set by user
    int w = getCurrentKernelSize();
    double sigma = w / 4.0;
    double mean = w / 2.0;
    double sum = 0.0; // For accumulating the kernel values
    for (int x = 0; x < w; ++x)
    {
        for (int y = 0; y < w; ++y)
        {
            falloffKernel(x, y) =
                    exp(-0.5 * (pow((x + 0.5 - mean) / sigma, 2.0) + pow((y + 0.5 - mean) / sigma, 2.0)))
                    / (2 * M_PI * sigma * sigma);

            // Accumulate the kernel values
            sum += falloffKernel(x, y);
        }
    }

    // Normalize the kernel
    for (int x = 0; x < w; ++x)
        for (int y = 0; y < w; ++y)
            falloffKernel(x, y) /= sum;
}

void KwatraTextureSynthesisParams::computeNghbrhdInterval()
{
    nghbrhdInterval = static_cast<int>(floor(.5 + getCurrentKernelSize() / getOverlappingFactor()));
    if (outputStateChanges)
        std::cout << "[DEBUG] New neighbourhood interval: " << nghbrhdInterval << std::endl;
}

/**
 * @brief Computes the number of full overlapping windows positioned at the computed interval
 * that can fit in the provided output size
 */
void KwatraTextureSynthesisParams::computeNghbrhdSize()
{
    if (getNghbrhdInterval() == 0)
    {
        std::cerr << "[WARNING] Neighbourhood interval is 0: cannot compute a neighbourhood size" << std::endl;
        return;
    }

    nghbrhdSize = (getOutputImageSize() - getCurrentKernelSize() + getNghbrhdInterval()) / getNghbrhdInterval();
    if (outputStateChanges)
        std::cout << "[DEBUG] New neighbourhood size: " << nghbrhdSize << std::endl;
}

//===========
// ACCESSORS
//===========

int KwatraTextureSynthesisParams::getOutputImageSize() const
{
    return outputImageSize;
}

cv::Vec2i KwatraTextureSynthesisParams::getInputImageSize() const
{
    return inputImageSize;
}

int KwatraTextureSynthesisParams::getMaxIterationCount() const
{
    return maxIterationCount;
}

int KwatraTextureSynthesisParams::getOverlappingFactor() const
{
    return overlappingFactor;
}

bool KwatraTextureSynthesisParams::getBreakOnConvergence() const
{
    return breakOnConvergence;
}

bool KwatraTextureSynthesisParams::getUsesWarping() const
{
    return usesWarping;
}

double KwatraTextureSynthesisParams::getSoftConstraintFactor() const
{
    return softConstraintFactor;
}

double KwatraTextureSynthesisParams::getRegressorValue() const
{
    return regressorValue;
}

int KwatraTextureSynthesisParams::getCurrentKernelSize() const
{
    assert(currentKernelSizeLevel >= 0 && currentKernelSizeLevel < kernelSizes.size());
    return kernelSizes.at(currentKernelSizeLevel);
}

int KwatraTextureSynthesisParams::getCurrentKernelSizeLevel() const
{
    assert(currentKernelSizeLevel >= 0 && currentKernelSizeLevel < kernelSizes.size());
    return currentKernelSizeLevel;
}

//==========
// MUTATORS
//==========

void KwatraTextureSynthesisParams::setOutputImageSize(int value)
{
    assert(value > 0);
    outputImageSize = value;
    if (outputStateChanges)
        std::cout << "[DEBUG] New output size: " << outputImageSize << std::endl;
    computeNghbrhdSize();
}

void KwatraTextureSynthesisParams::setInputImageSize(cv::Vec2i value)
{
    assert(value[0] > 0 && value[1] > 0);
    inputImageSize = value;
    if (outputStateChanges)
        std::cout << "[DEBUG] New input size: " << inputImageSize << std::endl;
}

void KwatraTextureSynthesisParams::setCurrentKernelSizeLevel(int value)
{
    assert(value >= 0 && value < kernelSizes.size());
    currentKernelSizeLevel = value;
    if (outputStateChanges)
        std::cout << "[DEBUG] New kernel size: " << getCurrentKernelSize() << std::endl;
    computeFalloffKernel();
    computeNghbrhdInterval();
    computeNghbrhdSize();
}

void KwatraTextureSynthesisParams::setMaxIterationCount(int value)
{
    assert(value > 0);
    maxIterationCount = value;
}

void KwatraTextureSynthesisParams::setOverlappingFactor(int value)
{
    assert(value > 0);
    overlappingFactor = value;
    if (outputStateChanges)
        std::cout << "[DEBUG] New overlapping: " << overlappingFactor << std::endl;
    computeNghbrhdInterval();
    computeNghbrhdSize();
}

void KwatraTextureSynthesisParams::setBreakOnConvergence(bool value)
{
    breakOnConvergence = value;
}

void KwatraTextureSynthesisParams::setUsesWarping(bool value)
{
    usesWarping = value;
    if (outputStateChanges && usesWarping)
        std::cout << "Now using warping/advection" << std::endl;
}

void KwatraTextureSynthesisParams::setSoftConstraintFactor(double value)
{
    assert(value >= 0.0);
    softConstraintFactor = value;
}

void KwatraTextureSynthesisParams::setRegressorValue(double value)
{
    assert(value >= 0.0 && value <= 2.0);
    regressorValue = value;
}

void KwatraTextureSynthesisParams::setKernelSizes(const std::vector<int> &values)
{
    assert(values.size() > 0);
    kernelSizes = values;
    if (currentKernelSizeLevel >= kernelSizes.size())
        currentKernelSizeLevel = 0;
}

//=================
// COMPUTED VALUES
//=================

const cv::Mat_<double>& KwatraTextureSynthesisParams::getFalloffKernel() const
{
    return falloffKernel;
}

int KwatraTextureSynthesisParams::getNghbrhdInterval() const
{
    return nghbrhdInterval;
}

int KwatraTextureSynthesisParams::getNghbrhdSize() const
{
    return nghbrhdSize;
}
