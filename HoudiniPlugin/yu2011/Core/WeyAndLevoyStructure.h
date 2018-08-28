#ifndef __WeyAndLevoyStructure_h__
#define __WeyAndLevoyStructure_h__

#include <string>
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

using namespace Mokko;

namespace Mokko {


class WeyAndLevoyStructure
{

public:


    WeyAndLevoyStructure(){}
    ~WeyAndLevoyStructure(){}


    //=================================================================
    //                          GAUSSIAN PYRAMID
    //=================================================================

    GaussianPyramid BuildImagePyramid(std::string filename, int numberOfLevel);
    GaussianPyramidMesh BuildMeshPyramid(GU_Detail *gdp, float cellSize, int numberOfLevel);
    GaussianPyramidMesh BuildMeshPyramid(GU_Detail *gdp, std::vector<float> &cellSizes, int numberOfLevel);




    //=================================================================
    //                          PATCH CREATION
    //=================================================================

    std::vector<Pixel> ResampleNeighborhood(std::vector<Vertex> &vertices, Pixel p, Vec3f s, Vec3f t, Vec3f n, int windowSize, bool useCenterPixel, float scaling);
    std::vector<Vertex> FlattenLocalPatch(const GaussianPyramidMesh &mesh,int level, Pixel p, Vec3f s, Vec3f t, Vec3f n ,float radius );
    float ComputeScaling(GaussianPyramidMesh &mesh, int level);

//    void TransferColorToVertices(std::vector<Vertex> &vertices, std::vector<Pixel> neighPixel, Vertex v, Pixel p, Vec3f s, Vec3f t, Vec3f n, int windowSize, bool useCenterPixel, float scaling);
    void TransferColorToVertices(std::vector<Vertex> &vertices, std::vector<Pixel> neighPixel);


	
};
} // End HDK_Sample namespace

#endif
