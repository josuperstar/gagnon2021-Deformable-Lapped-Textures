#ifndef __KWATRATEXTURESYNTHESIS_h__
#define __KWATRATEXTURESYNTHESIS_h__
#include "Image.h"
#include "GaussianPyramidMesh.h"
#include "GaussianPyramid.h"
#include "KwatraTextureSynthesisParams.h"

#include <ANN/ANN.h>

namespace Mokko
{

class KwatraTextureSynthesis
{
private:

    // Debug

    static bool outputNeighbours;

    // Best-match search

    static const int channelCount = 3; ///< Number of channels used in the best-match search
    static cv::Point convertPixCoordsToNgbhdCoords(KwatraTextureSynthesisParams params, int i, int j);
    static cv::Mat_<cv::Point> buildRandNghbrhds(KwatraTextureSynthesisParams params);
    static ANNkd_tree * createSearchTree(int level, const GaussianPyramid &inputPyramid, const KwatraTextureSynthesisParams params);
    static void findBestMatchNghbrhds(const ImageCV *outputImage, const ImageCV *warpedImage,
                                      const KwatraTextureSynthesisParams params,
                                      cv::Mat_<cv::Point> &neighbourhoods, ANNkd_tree *searchTree);

    // Synthesis

    static void updateOutputImage(ImageCV *outputImage, const ImageCV *inputImage, const ImageCV *warpedImage,
                                  const KwatraTextureSynthesisParams params, const cv::Mat_<cv::Point> &neighbourhoods);


public:

    static void RunLevelIterations(int level,
                                   const GaussianPyramid &inputPyramid, GaussianPyramid &outputPyramid, const GaussianPyramid &warpedPyramid,
                                   KwatraTextureSynthesisParams &params, bool perfomInitialBestMatchSearch);


};

}

#endif
