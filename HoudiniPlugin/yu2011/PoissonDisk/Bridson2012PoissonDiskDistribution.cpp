#include "Bridson2012PoissonDiskDistribution.h"
//#include "HoudiniUtils.h"

#include <GU/GU_RandomPoint.h>
#include <GEO/GEO_PrimVDB.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/math/Transform.h>

using namespace Mokko;
using namespace std;


std::vector<PoissonDisk> Bridson2012PoissonDiskDistribution::PoissonDiskSampling(GU_Detail *gdp, float diskRadius)
{
    cout << "[Bridson2012PoissonDiskDistribution] on level set"<<endl;

    //hardcode life:
    //We start with a very small life to avoid popping artifact.
    float life = 0.01f;

    // Find first vdb primitive of input 0
    GEO_Primitive* prim;
    GEO_PrimVDB* phi = 0x0;
    GA_FOR_ALL_PRIMITIVES(gdp, prim)
    {
        if (phi = dynamic_cast<GEO_PrimVDB*>(prim))
            break;
    }

    if (!phi || !phi->hasGrid())
    {
        cout << "[Bridson2012PoissonDiskDistribution] Input geometry 0 has no VDB grid!"<<endl;
        return allpoints;
    }

    cout << "[Bridson2012PoissonDiskDistribution] We have a valid vdb"<<endl;

    //this->initializeGrid(points, diskRadius);

    /*
    //intialize sGrid
    k = 30;
    r = diskRadius;
    cellSize = r/(sqrt(n));

    //create the grid

    int gridSizeX = (int)ceil(1.0f/cellSize);
    int gridSizeY = (int)ceil(1.0f/cellSize);
    int gridSizeZ = (int)ceil(1.0f/cellSize);

    backgroundGrid = sGrid(gridSizeX,gridSizeY,gridSizeZ,cellSize);
    cout << "Cell size:"<<cellSize<<endl;
    cout << "radius "<<r<<endl;
    cout << "k:"<<k;
    */

    /*
    We assume the surface geometry is given as a signed distance function:
    this permits fast projection of points to the surface. Pseudocode
    is provided in Algorithm 1. In the outer loop we search
    for “seed” sample points on the surface, checking every grid cell
    that intersects the surface (i.e. where the level set changes sign) so
    we don’t miss any components: in a cell we take up to t attempts,
    projecting random points from the cell to the surface and stopping
    when one satisfies the Poisson disk criterion, i.e. is at least distance
    r from existing samples. Once we have a seed sample, we continue
    sampling from it, taking a step of size e · r from the previous sample
    along a random tangential direction d, again projecting to the
    surface and checking the Poisson disk criterion. Parameters t = 30
    and e = 1.085 worked well, but could be further tuned.
    */

    t = 30;
    //Input: Level set φ, radius r, # attempts t, extension e
    //Output: Sample set S

    /*
    1: for all grid cells C where φ changes sign do
    2:  for t attempts do
    3:      Generate random point p in C
    4:      Project p to surface of φ
    5:      if p meets the Poisson Disk criterion in S then
    6:          S ← S ∪ {p}
    7:          Break
    8:  if no point was found in C then
    9:      Continue
    10: while new samples are found do
    11:     Generate random tangential direction d to surface at p
    12:     q ← p + d · e · r
    13:     Project q to surface of φ
    14:     if q meets the Poisson Disk criterion in S then
    15:         S ← S ∪ {q}
    16:         p ← q
    */

    // Get the grids
    //std::cout << "getStorageType: " << phi->getStorageType() << std::endl;
    //std::cout << "getTypleSize: " << phi->getTupleSize() << std::endl;
    //std::cout << "getGridName: " << phi->getGridName() << std::endl;

    openvdb::GridBase::Ptr ptr = phi->getGridPtr();
    cout << "ptr "<<ptr<<endl;
    openvdb::FloatGrid::Ptr gridSurface = openvdb::gridPtrCast<openvdb::FloatGrid>(ptr);
    //openvdb::FloatGrid::ConstPtr gridSurface = openvdb::gridConstPtrCast<openvdb::FloatGrid>(phi->getGridPtr());
    cout << "gridSurface "<< gridSurface<<endl;
    if(!gridSurface)
    {
        cout << "[Bridson2012PoissonDiskDistribution] Surface grid can't be converted in FloatGrid"<<endl;
        return allpoints;
    }

    if ((gridSurface->getGridClass() != openvdb::GRID_LEVEL_SET))
    {
        cout<< "[Bridson2012PoissonDiskDistribution] Surface grid is not a Level-set FloatGrid!"<<endl;
        return allpoints;
    }

    //=================================================================
    //                         OPEN VDB ACCESSORS
    //=================================================================


    //openvdb::tools::PointSampler

    // Create the gradient field
    openvdb::tools::Gradient<openvdb::FloatGrid> gradientOperator(*gridSurface);
    openvdb::VectorGrid::ConstPtr gridGradient = gradientOperator.process();
    //gridGradient->setName(ssGrad.str());

    openvdb::VectorGrid::ConstAccessor accessorGradient = gridGradient->getConstAccessor();

    openvdb::tools::GridSampler<openvdb::VectorGrid::ConstAccessor, openvdb::tools::BoxSampler>
            samplerGradient(accessorGradient, gridGradient->transform());

     // Get the sampler for the boundary grid (tri-linear filtering, in surface grid index space)
    openvdb::FloatGrid::Accessor accessorSurface = gridSurface->getAccessor();

    openvdb::tools::GridSampler<openvdb::FloatGrid::Accessor, openvdb::tools::BoxSampler>
            samplerSurface(accessorSurface, gridSurface->transform());

    //=================================================================
    // 1: for all grid cells C where φ changes sign do
    //=================================================================
    //int i = 0;

    cout << "[Bridson2012PoissonDiskDistribution]"<<"There is "<< this->backgroundGrid.GetNumberOfPoissonDisk() << " Poisson disk" <<endl;

    bool useJustOnePoint = false;
    cout << "[Bridson2012PoissonDiskDistribution] Step 1: for all grid cells C where φ changes sign do"<<endl;
    for (openvdb::FloatGrid::ValueOnCIter gridCellIt = gridSurface->cbeginValueOn(); gridCellIt; ++gridCellIt)
    {
        if (useJustOnePoint)
            continue;

        float x = gridCellIt.getCoord().x();
        float y = gridCellIt.getCoord().y();
        float z = gridCellIt.getCoord().z();

        float offset = 0.5;

        openvdb::Vec3f cellPosition(x,y,z);
        float dist = *gridCellIt;
        //shall we take the middle of the cell ?

        openvdb::Vec3f worldCellPos = gridSurface->transform().indexToWorld(cellPosition);

        /*
        worldCellPos += openvdb::Vec3f(1.0,1.0,1.0);
        PoissonDisk poissonDisk(worldCellPos);
        backgroundGrid.Insert( poissonDisk, r );
        points.push_back(poissonDisk);
        continue;
        */

        float boundaryDist = samplerSurface.wsSample(worldCellPos);

        //openvdb::Vec3f p    = it.getCoord();
        //if (boundaryDist <= 0.0)// && grad.length() > 0.0)
        {
            //if it is not close to the surface, continue
            if (abs(boundaryDist) > r)
                continue;
            //=================================================================
            //2:  for t attempts do
            //=================================================================
            //cout << "Step 2:  for t attempts do"<<endl;
            bool ableToInsertPoint = false;
            for(int i =0; i < t; i++)
            {
                //=================================================================
                //3:      Generate random point p in C
                //=================================================================

                int seed = i;
                //we want it to oscillate between -0.5 and 0.5

                srand(seed);
                float rx = (((double) rand()/(RAND_MAX)-offset));
                srand(seed+1);
                float ry = (((double) rand()/(RAND_MAX)-offset));
                srand(seed+2);
                float rz = (((double) rand()/(RAND_MAX)-offset));

                openvdb::Vec3f randomPosition(rx,ry,rz);

                randomPosition *= r;

                openvdb::Vec3f p = worldCellPos+randomPosition;
                //hack test
                //openvdb::Vec3f p(0,0,0);

                float newPointDistance = samplerSurface.wsSample(p);
                if (abs(newPointDistance) > r*2)
                {
                    //cout << "random point is outside of range"<<endl;
                    continue;
                }

                //=================================================================
                //4:      Project p to surface of φ
                //=================================================================
                openvdb::Vec3f grad = samplerGradient.wsSample(p);
                if (grad.length() < 0.0001)
                    continue;
                PoissonDisk poissonDisk = projectPointOnLevelSet(p,newPointDistance,grad);
                UT_Vector3 gradH = UT_Vector3(grad.x(),grad.y(),grad.z());
                gradH /= grad.length();
                poissonDisk.SetNormal(gradH);

                //=================================================================
                //5:      if p meets the Poisson Disk criterion in S then
                //=================================================================
                std::vector<PoissonDisk> neighbors;
                bool meetPoissonDiskCriterion = !backgroundGrid.IsInNeighbourhood( poissonDisk, r, cellSize, neighbors);
                if (meetPoissonDiskCriterion)
                {
                    //=================================================================
                    //6:          S ← S ∪ {p}
                    //=================================================================
                    this->InsertPoissonDisk(poissonDisk,r, false);
                    if (poissonDisk.IsValid())
                    {
                        break;
                    }
                    //cout << "meet poisson disk criterion but can't insert point because it is not valid ..."<<endl;
                }
                else
                {
                    //cout << "Does not meet poisson disk criterion"<<endl;
                }
            }
            if (!ableToInsertPoint)
            {
                //cout << "after "<<t<<" attemps, there is no possible insertion."<<endl;
            }
        }
    }

    return allpoints;

}




//Step 0

/*
Initialize an n-dimensional background grid for storing
samples and accelerating spatial searches. We pick the cell size to
be bounded by r/√n
n, so that each grid cell will contain at most
one sample, and thus the grid can be implemented as a simple ndimensional
array of integers: the default −1 indicates no sample, a
non-negative integer gives the index of the sample located in a cell.
*/

void Bridson2012PoissonDiskDistribution::initializeGrid(std::vector<PoissonDisk> existingPoints, float diskRadius)
{
    cout << "[Bridson2012PoissonDiskDistribution] Step 0: initialize the grid"<<endl;
    cout << "[Bridson2012PoissonDiskDistribution] We have "<<existingPoints.size()<<" existing points"<<endl;
    cout << "[Bridson2012PoissonDiskDistribution]initializeGrid with radius "<<diskRadius<<endl;

    //intialize sGrid
    k = 30;
    r = diskRadius;
    cellSize = r/(sqrt(n));


    //section 3.3.1 of Yu 2011
    //We kill particles if they are at a distance less than (1-a)d
    //a = 0.25
    //float a = 0.25;
    float a = 0.66f;
    float kd = ((1.0f-a)*r);


    cout << "[Bridson2012PoissonDiskDistribution] We kill particles if they are at a distance less than (1-a)d: "<<kd<<endl;

    //create the grid

    int gridSizeX = (int)ceil(1.0f/cellSize);
    int gridSizeY = (int)ceil(1.0f/cellSize);
    int gridSizeZ = (int)ceil(1.0f/cellSize);

    backgroundGrid = TreeDGrid(gridSizeX,gridSizeY,gridSizeZ, cellSize);

    std::vector<PoissonDisk>::iterator it;
    for(it =  existingPoints.begin(); it != existingPoints.end(); it++)
    {
        this->InsertPoissonDisk(*it,kd, true);
    }
}


PoissonDisk Bridson2012PoissonDiskDistribution::projectPointOnLevelSet(openvdb::Vec3f point, float distance, openvdb::Vec3f grad)
{

    //get the norm of the gradient
    openvdb::Vec3f gradNorm = grad;
    gradNorm.normalize();

    //=================================================================
    //4:      Project p to surface of φ
    //=================================================================

    //projection
    //cout << "old p "<<p<<endl;
    //p = p - dist * (grad/gradNorm);
    point = point - distance * gradNorm;

    PoissonDisk newPoint = PoissonDisk(point);
    UT_Vector3 normal(UT_Vector3(gradNorm.x(),gradNorm.y(),gradNorm.z()));

    newPoint.SetNormal(normal);

    return newPoint;
}

void Bridson2012PoissonDiskDistribution::InsertPoissonDisk(PoissonDisk p, float diskRadius, bool existingPoint)
{

    //get next available id
    long numberOfPoints = this->maxId+1;

    //try to insert the poisson disk on the level set
    backgroundGrid.Insert(p,diskRadius);

    if (p.GetId() > this->maxId) //existing points
    {
        this->maxId = p.GetId();
    }
    if (!existingPoint && p.IsValid()) // if it is a new point and it is valid
    {
        float life = 0.01f;
        //new point !

        p.SetLife(life);
        p.SetSpawn(life);
        p.SetDynamicTau(0.0f);
        p.SetMature(0); //new points are not mature.
        p.SetId(numberOfPoints);
        allpoints.push_back(p);
        this->maxId = numberOfPoints;
    }

    //if it is an existing point, we want to had it anyway because we will delete it temporaly if it is non valid.
    if(existingPoint)
        allpoints.push_back(p);

}

