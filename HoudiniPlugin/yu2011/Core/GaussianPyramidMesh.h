#ifndef __GaussianPyramidMesh_h__
#define __GaussianPyramidMesh_h__
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
//#include <GA_ElementGroup.h>
#include <Math/Vec3.h>
#include "Image.h"
#include "Set/SpatialGrid.h"
#include "Vertex.h"
#include "Primitive.h"


using namespace Mokko;

namespace Mokko {


class GaussianPyramidMesh
{

public:

    GaussianPyramidMesh();
    ~GaussianPyramidMesh();

    bool BuildGaussianPyramidReduction(GU_Detail*gdp,int numberOfLevel, float cellSize,float gaussianFuseLenght);
    bool BuildGaussianPyramidAugmentation(GU_Detail*gdp,int numberOfLevel, float cellSize,float gaussianFuseLenght);
    bool FillGaussianPyramid(GU_Detail*gdp, float cellSize,int numberOfLevel);
    bool FillGaussianPyramid(GU_Detail*gdp, std::vector<float> &cellSizes,int numberOfLevel);

    void rebuildGrid(GU_Detail *gdp, float cellSize);

    //void CreateVertexNeighborhood(long pointIndex, int level);

    std::vector<Vertex*> GetVertexNeighborhood(long pointIndex, int level, float radius);
    std::vector<Vertex*> GetVertexAroundPosition(Vec3f position, int level, float radius);
    std::vector<const Vertex*> GetVertexAroundPosition(Vec3f position, int level, float radius) const;

    void TurkRetiling();

    Vertex* GetVertex(long level, long id);//{return &(this->data[level][id]);}
    const Vertex* GetVertex(long level, long id) const;

    std::vector<Vertex>& GetMeshAtLevel(int level)
    {
        std::vector<Vertex> &ref = data[level];
        return ref;
    }

    const std::vector<Vertex>& GetMeshAtLevel(int level) const
    {
        return data.at(level);
    }

    std::vector<Primitive> GetPrimitivesAtLevel(int level) {return this->primitives[level];}
    int GetNumberOfLevel(){return this->numberOfLevel;}
    int Size(){return this->data.size();}


private:
    std::vector<SpatialGrid<Vertex> > dataGridSet;
    //std::map<long, std::map<int, std::vector<Vertex> > > vertexNeighborhood;
    std::map<long, std::vector<Vertex> > data;
    std::map<int,std::vector<Primitive> > primitives;

    int numberOfLevel;

    Vec3f CreateVector(UT_Vector3 v);
    Vec3d CreateVectorD(UT_Vector3 v);
    static UT_Vector3 Projection(UT_Vector3 PointOfTheReference, UT_Vector3 PointToProject, UT_Vector3 N);


};
} // End HDK_Sample namespace

#endif
