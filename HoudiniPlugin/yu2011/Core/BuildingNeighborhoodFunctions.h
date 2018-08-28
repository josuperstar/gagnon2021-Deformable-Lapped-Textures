#ifndef __BUILDINGNEIGHBORHOODFUNCTIONS_H__
#define __BUILDINGNEIGHBORHOODFUNCTIONS_H__
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
//#include <GA_ElementGroup.h>
#include <Math/Vec3.h>
#include "Image.h"
#include "Set/SpatialGrid.h"
//#include "OldApproach/TextureSynthesis.h"
#include "GaussianPyramidMesh.h"
#include <Eigen/Eigen/Core>
#include <Eigen/Eigen/Dense>
#include "GaussianPyramid.h"
#include "WeyAndLevoyStructure.h"

#include <ANN/ANN.h>

using namespace Mokko;

namespace Mokko {


class BuildingNeighborhood
{

public:



    //===============================================================
    //                     BUILD NEIGHBOORHOOD
    //===============================================================

    std::vector<Pixel> BuildMeshNeighborhood(GaussianPyramidMesh &mesh, int level, bool useLowerLevel, Pixel p, Vec3f s, Vec3f t, Vec3f n,float scaling, float radius, std::vector<int> windowSizes, bool useCenterPixel);
    std::vector<Pixel> BuildMeshNeighborhood(const GaussianPyramidMesh &mesh, int level, Pixel p, Vec3f s, Vec3f t, Vec3f n, float scaling,float radius, int windowSize, bool useCenterPixel);
    std::vector<Pixel> BuildImageNeighborhood(GaussianPyramid &pyramid, int level,bool useLowerLevel, Pixel p_i, int numberOfLevel, std::vector<int> windowSizes,std::vector<Pixel> &lowerLevelNeighbors, bool useCenteredPixel);
    std::vector<Pixel> BuildImageNeighborhoodUsingLowerLevel(GaussianPyramid &pyramid, int level,bool useLowerLevel, Pixel p_i, int numberOfLevel, std::vector<int> windowSizes);

    //===============================================================
    //                     ANN TREE
    //===============================================================
    void BuildTreeOneLevel(GaussianPyramid &gaussianImage, int level, std::vector<int> windowSizes, ANNpointArray dataPts);
    void BuildTreeTwoLevels(GaussianPyramid &gaussianImage, int level, std::vector<int> windowSizes, ANNpointArray dataPts);

    //void BuildTreeOneLevelMesh(GaussianPyramidMesh &mesh, int level, float d, float cellSize, std::vector<int> windowSizes, ANNpointArray dataPts);
    //void BuildTreeTwoLevelsMesh(GaussianPyramidMesh &mesh, int level, float d, float cellSize, std::vector<int> windowSizes, ANNpointArray dataPts);

    //===============================================================
    //                     ANN + PCA TREE
    //===============================================================
    void BuildTreeOneLevel_PCA(GaussianPyramid &gaussianImage, int level, std::vector<int> windowSizes, int dim, int nPts, cv::Mat dataPca);
    void BuildTreeTwoLevels_PCA(GaussianPyramid &gaussianImage, int level, std::vector<int> windowSizes, int dim, int nPts, cv::Mat dataPca);


	
};
} // End HDK_Sample namespace

#endif
