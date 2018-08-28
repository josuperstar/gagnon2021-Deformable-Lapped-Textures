#ifndef __KWATRASURFACETEXTURESYNTHESIS_h__
#define __KWATRASURFACETEXTURESYNTHESIS_h__
#include "Image.h"
#include "GaussianPyramidMesh.h"
#include "GaussianPyramid.h"
#include "KwatraSurfaceTextureSynthesisParams.h"

#include <memory>
#include <ANN/ANN.h>

namespace Mokko
{

class KwatraSurfaceTextureSynthesis
{
private:

    static const int channelCount = 3; ///< Number of channels used in the best-match search
    static int computeTreeDimension(const KwatraSurfaceTextureSynthesisParams params);
    static std::unique_ptr<ANNkd_tree> createSearchTree(int level, const GaussianPyramid &inputPyramid, const KwatraSurfaceTextureSynthesisParams params);
    static ANNkd_tree * createSearchTree_PCA(int level, const GaussianPyramid &inputPyramid, const KwatraSurfaceTextureSynthesisParams params,  cv::PCA *pca);

    static double getHardConstraintFactor(double x, double y);

    static double findBestMatchesAndUpdateAnn(int level, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                                              const KwatraSurfaceTextureSynthesisParams params, ANNkd_tree &searchTree);
    static double findBestMatchesAndUpdateBruteforce(int level, const GaussianPyramid &inputPyramid, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                                                     const KwatraSurfaceTextureSynthesisParams params);
    static double findBestMatchesAndUpdate_PCA(int level, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                                           const KwatraSurfaceTextureSynthesisParams params, ANNkd_tree *searchTree, cv::PCA *pca);


    //static void updateOutputMesh(int level,GaussianPyramidMesh &Gs, const ImageCV *inputImage, const vector<Vertex> warpedMesh,
    //                              const KwatraSurfaceTextureSynthesisParams params, GA_RWHandleV3 &attColor, double r = 0.8);
    static void updateMesh(int level, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                           const KwatraSurfaceTextureSynthesisParams params,
                           const vector<Vec3d> &accumulatedColours, const vector<double> &accumulatedFactors);

    static void RunLevelIterationsAnn(int level,
                                   const GaussianPyramid &inputPyramid, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                                   KwatraSurfaceTextureSynthesisParams &params);
    static void RunLevelIterationsBruteforce(int level,
                                   const GaussianPyramid &inputPyramid, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                                   KwatraSurfaceTextureSynthesisParams &params);

public:

    static void RunLevelIterations(int level,
                                   const GaussianPyramid &inputPyramid, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                                   KwatraSurfaceTextureSynthesisParams &params);

};

}

#endif
