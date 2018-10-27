#ifndef __Bridson2012PoissonDiskDistribution_h_
#define __Bridson2012PoissonDiskDistribution_h_
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <GEO/GEO_PointTree.h>
//#include <GA_ElementGroup.h>
#include <vector>

#include "TreeDGrid.h"

#include <openvdb/openvdb.h>
#include <GU/GU_PrimVDB.h>
#include <openvdb/tools/Interpolation.h>


using namespace std;
namespace Mokko {


//Based on paper from Bridson "Fast Poisson Disk Sampling in Arbritrary Dimensions"
// http://people.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf


/*
The algorithm takes as input the extent of the sample domain in
Rn, the minimum distance r between samples, and a constant k
as the limit of samples to choose before rejection in the algorithm
(typically k = 30).
*/

class Bridson2012PoissonDiskDistribution
{

public:

    //Bridson2012PoissonDiskDistribution(){}
    ~Bridson2012PoissonDiskDistribution()
    {
        cout << "[Bridson2012PoissonDiskDistribution] destrotying grid"<<endl;
        //backgroundGrid.~TreeDGrid();
        backgroundGrid.Clear();
        allPointsGrid.Clear();
        ProcessList.clear();
        SamplePoints.clear();
    }
    std::vector<PoissonDisk> PoissonDiskSampling(GU_Detail *gdp, float diskRadius, float angleNormalThreshold);
    void SetNumberOfPoint(int data){this->numberOfPoints = data;}
    void initializeGrid(std::vector<PoissonDisk> existingPoints,float diskRadius,  float angleNormalThreshold);


    void SetMaxId(long data){maxId = data;}

private:

    PoissonDisk projectPointOnLevelSet(openvdb::Vec3f point, float distance, openvdb::Vec3f grad );
    void InsertPoissonDisk(PoissonDisk disk, float diskRadius, bool existingPoint, float angleNormalThreshold);

    template <typename PRNG>
    PoissonDisk PopRandom( std::vector<PoissonDisk>& Points, PRNG& Generator )
    {
        const int Idx = Generator.RandomInt( Points.size()-1 );
        const PoissonDisk P = Points[ Idx ];
        Points.erase( Points.begin() + Idx );
        return P;
    }

    template <typename PRNG>
    PoissonDisk GenerateRandomPointAround( PoissonDisk& P, float MinDist, PRNG& Generator )
    {
        // start with non-uniform distribution
        float R1 = Generator.RandomFloat();
        float R2 = Generator.RandomFloat();

        // radius should be between MinDist and 2 * MinDist
        float Radius = MinDist * ( R1 + 1.0f );

        // random angle
        float Angle = 2 * 3.141592653589f * R2;

        // the new point is generated around the point (x, y)
        UT_Vector3 p = P.GetPosition();

        float X = p.x() + Radius * cos( Angle );
        float Y = p.y() + Radius * sin( Angle );
        float Z = p.y() + Radius * tan( Angle );

        return PoissonDisk( X, Y, Z );
    }


    float poissonDiskRadius;    //radius
    int k;      //the limit of samples to choose before rejection in the algorithm, typically k = 30
    int numberOfPoints;
    TreeDGrid backgroundGrid;
    TreeDGrid allPointsGrid;
    vector<PoissonDisk> ProcessList;
    vector<PoissonDisk> SamplePoints;
    int n = 3; // n-dimensional
    int t; // number of attemps
    float cellSize;
    DefaultPRNG Generator;
    bool Circle = true;


    std::vector<PoissonDisk> allpoints;
    long maxId = 0;


};
}

#endif
