#ifndef KWATRATEXTURESYNTHESISPARAMS_H
#define KWATRATEXTURESYNTHESISPARAMS_H

#include <opencv2/opencv.hpp>
#include <vector>


namespace Mokko {

/**
 * @brief Parameters for the Kwatra et al. '05 texture synthesis
 *
 * Default values for all fields - important settings to check/set:
 * - inputImageSize
 * - outputImageSize
 * - kernelSizes
 * - usesWarping (and softConstraintFactor if true)
 */
class KwatraTextureSynthesisParams
{
protected:

    // Debug

    static bool outputStateChanges;

private:

    // Parameters that will have to be changed in multiple-resolution mode

    int outputImageSize; ///< The size (width and height) of the output texture
    cv::Vec2i inputImageSize; ///< The size (width x height) of the input sample
    int currentKernelSizeLevel; ///< In the vector of kernel sizes (0 is the largest)

    // Parameters that will be recomputed in multiple-resolution mode

    cv::Mat_<double> falloffKernel; ///< Gaussian fall-off kernel used by the robust optimization (same size as kernel)
    int nghbrhdInterval;
    int nghbrhdSize;

    // Fixed parameters

    int maxIterationCount; ///< [N]

    std::vector<int> kernelSizes; ///< The kernel takes different resolutions
    int overlappingFactor; ///< Divider of the kernel size to set the interval between the neighbourhoods [4]

    bool breakOnConvergence; ///< If true, a check for convergence will be performed until maxIterationCount

    bool usesWarping; ///< If true, the synthesis will take into account the previous warped frame if required
    double softConstraintFactor; ///< Factor of the soft constraint (the warped pixels) [lambda]
    double regressorValue; ///< Robust regressor factor, between 0 and 2 [r]


    // Compute methods

    void computeCurrentKernelSize();
    void computeFalloffKernel();
    void computeNghbrhdInterval();
    void computeNghbrhdSize();

public:
    KwatraTextureSynthesisParams();

    // Accessors
    int getOutputImageSize() const;
    cv::Vec2i getInputImageSize() const;
    int getMaxIterationCount() const;
    int getOverlappingFactor() const;
    bool getBreakOnConvergence() const;
    bool getUsesWarping() const;
    double getSoftConstraintFactor() const;
    double getRegressorValue() const;
    int getCurrentKernelSize() const; ///< Make sure you set the kernelSizes before calling this method
    int getCurrentKernelSizeLevel() const;

    // Mutators
    void setOutputImageSize(int value);
    void setInputImageSize(cv::Vec2i value);
    void setCurrentKernelSizeLevel(int value);
    void setMaxIterationCount(int value);
    void setOverlappingFactor(int value);
    void setBreakOnConvergence(bool value);
    void setUsesWarping(bool value);
    void setSoftConstraintFactor(double value);
    void setRegressorValue(double value);
    void setKernelSizes(const std::vector<int> &values);

    // Computed values
    const cv::Mat_<double>& getFalloffKernel() const; ///< Make sure you set the kernelSizeDivider before calling this method
    int getNghbrhdInterval() const; ///< Make sure you set the kernelSizeDivider before calling this method
    int getNghbrhdSize() const; ///< Make sure you set the kernelSizeDivider before calling this method
};

} // End namespace

#endif // KWATRATEXTURESYNTHESISPARAMS_H
