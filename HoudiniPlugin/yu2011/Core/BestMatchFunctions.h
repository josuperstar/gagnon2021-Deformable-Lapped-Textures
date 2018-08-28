#ifndef __BESTMATCHFUNCTIONS_H__
#define __BESTMATCHFUNCTIONS_H__
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
#include "BuildingNeighborhoodFunctions.h"

using namespace Mokko;

namespace Mokko {


class BestMatchFunctions
{

public:


    //=================================================================
    //                          BEST MATCH
    //=================================================================

    Pixel BestMatch(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh,  int level, Pixel p, float d, float cellSize, Vec3f s,Vec3f t,Vec3f n, std::vector <int> windowSizes,bool useLowerLevel, float &error);
    Pixel BestMatchOpenCVMinMaxLoc(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh,  int level, Pixel p, float d, float cellSize,Vec3f s,Vec3f t,Vec3f n, std::vector <int> windowSizes,bool useLowerLevel, float &error);
    //Pixel BestMatchANN(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh,  int level, Pixel p, float d, float cellSize, Vec3f s,Vec3f t,Vec3f n, std::map<int, int> windowSizes,bool useLowerLevel, float &error);


    Pixel BestMatchOnLowerLevelOpenCVMinMaxLoc(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh,  int level, Pixel p, float d, float cellSize, Vec3f s,Vec3f t,Vec3f n, std::vector <int> windowSizes,bool useLowerLevel, float &error);
    Pixel BestMatchOnLowerLevel(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh,  int level, Pixel p, float d, float cellSize,Vec3f s,Vec3f t,Vec3f n, std::vector <int> windowSizes, float &error);
    Pixel BestMatchANNOnLowerLevel(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh,  int level, Pixel p, float d, float cellSize, Vec3f s,Vec3f t,Vec3f n, std::vector <int> windowSizes,bool useLowerLevel, float &error);



    Pixel BestMatch(GaussianPyramid &gaussianImage, std::vector<Pixel> Neiborhoords, int level, Pixel p, float d, std::vector <int> windowSizes, float &error);
    Pixel BestMatchOnLowerLevel(GaussianPyramid &gaussianImage, std::vector<Pixel> Neiborhoords, int level, Pixel p, float d, std::vector <int> windowSizes, float &error);

    Pixel BestMatchANN(GaussianPyramid &gaussianImage, std::vector<Pixel> Neiborhoords, int level, Pixel p, float d, std::vector <int> windowSizes, float &error);
    Pixel BestMatchANNOnLowerLevel(GaussianPyramid &gaussianImage, std::vector<Pixel> Neiborhoords, int level, Pixel p, float d, std::vector <int> windowSizes, float &error);


    Pixel BestMatchTemplateMtching(GaussianPyramid &gaussianImage, std::vector<Pixel> Ns, int level, Pixel p, std::vector <int> windowSizes, float &error);


	
};
}

#endif
