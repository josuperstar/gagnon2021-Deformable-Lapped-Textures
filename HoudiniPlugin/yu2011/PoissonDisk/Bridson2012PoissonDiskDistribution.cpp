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


//================================================================================================

//                                      POISSON DISK SAMPLING

//================================================================================================


void Bridson2012PoissonDiskDistribution::PoissonDiskSampling(GU_Detail* trackersGdp, GEO_PointTreeGAOffset &tree, GU_Detail *levelSet, float diskRadius, float angleNormalThreshold)
{
    cout << "[Bridson2012PoissonDiskDistribution] on level set using a threshold of "<<angleNormalThreshold<<endl;

    GA_RWHandleV3   attN(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attActive(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));
    GA_RWHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleI    attDensity(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"density", 1));

    // Find first vdb primitive of input 0
    GEO_Primitive* prim;
    GEO_PrimVDB* phi = 0x0;
    GA_FOR_ALL_PRIMITIVES(levelSet, prim)
    {
        if (phi = dynamic_cast<GEO_PrimVDB*>(prim))
            break;
    }

    if (!phi || !phi->hasGrid())
    {
        cout << "[Bridson2012PoissonDiskDistribution] Input geometry 0 has no VDB grid!"<<endl;
        return;
    }

    float a = 0.25;
    this->poissonDiskRadius = diskRadius;
    float killDistance = (1-a)*diskRadius;

    cout << "[Bridson2012PoissonDiskDistribution] We have a valid vdb"<<endl;


    //this->initializeGrid(points, diskRadius);

    GA_RWHandleI    attDeleteFaster(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"deleteFaster", 1));
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        int numberOfClosePoint;

        UT_Vector3 pointPosition = trackersGdp->getPos3(ppt);
        UT_Vector3 pointNormal   = attN.get(ppt);
        if (attId.get(ppt) > this->maxId)
            this->maxId = attId.get(ppt);


        bool meetPoissonDiskCriterion = backgroundGrid.RespectCriterion(trackersGdp,tree, pointPosition, pointNormal, diskRadius, killDistance, cellSize, numberOfClosePoint, angleNormalThreshold, ppt);
        attDensity.set(ppt,numberOfClosePoint);
        int deleteFaster = attDeleteFaster.get(ppt);
        int numberOfNeighbourThreshold = 1;
        //-------------- deleting faster logic ------------------
        if (numberOfClosePoint > numberOfNeighbourThreshold && deleteFaster == 0)
        {
            attDeleteFaster.set(ppt, 1);
        }
        else if(deleteFaster == 1 && numberOfClosePoint <= numberOfNeighbourThreshold)
        {
            attDeleteFaster.set(ppt, 0);
        }
        //-------------------------------------------------------
        if (attActive.get(ppt) == 0)
            continue;

        attActive.set(ppt,meetPoissonDiskCriterion);
        if (!meetPoissonDiskCriterion)
        {
            cout << "We should delete point "<<attId.get(ppt)<<", is in kill distance" <<killDistance<<endl;
        }
        //cout << "existing "<<attId.get(ppt)<<" "<<meetPoissonDiskCriterion<<endl;
    }

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
        return;
    }

    if ((gridSurface->getGridClass() != openvdb::GRID_LEVEL_SET))
    {
        cout<< "[Bridson2012PoissonDiskDistribution] Surface grid is not a Level-set FloatGrid!"<<endl;
        return;
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

    //cout << "[Bridson2012PoissonDiskDistribution]"<<"There is "<< this->backgroundGrid.GetNumberOfPoissonDisk() << " Poisson disk" <<endl;

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
        float boundaryDist = samplerSurface.wsSample(worldCellPos);

        //openvdb::Vec3f p    = it.getCoord();
        //if (boundaryDist <= 0.0)// && grad.length() > 0.0)
        {
            //if it is not close to the surface, continue
            //if (abs(boundaryDist) > poissonDiskRadius/3) // We should use a threshold defined by the user
            //    continue;
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
                //cout << "Generate random point p in C"<<endl;
                int seed = i;
                //we want it to oscillate between -0.5 and 0.5

                srand(seed);
                float rx = (((double) rand()/(RAND_MAX)-offset));
                srand(seed+1);
                float ry = (((double) rand()/(RAND_MAX)-offset));
                srand(seed+2);
                float rz = (((double) rand()/(RAND_MAX)-offset));

                openvdb::Vec3f randomPosition(rx,ry,rz);
                randomPosition *= poissonDiskRadius/10;

                openvdb::Vec3f p = worldCellPos+randomPosition;

                //cout << "P = "<<p<<endl;

                float newPointDistance = samplerSurface.wsSample(p);

                //cout << "P = "<<p<< " distance "<<newPointDistance<< " poissonDiskRadius"<<poissonDiskRadius<<endl;

                if (abs(newPointDistance) > poissonDiskRadius/3)
                {
                    //cout << "random point is outside of range"<<endl;
                    continue;
                }
                //cout << "abs(newPointDistance) > poissonDiskRadius"<<endl;
                //=================================================================
                //4:      Project p to surface of φ
                //=================================================================
                openvdb::Vec3f grad = samplerGradient.wsSample(p);
                if (grad.length() < 0.0001)
                    continue;
                openvdb::Vec3f poissonDisk = projectPointOnLevelSet(p,newPointDistance,grad);
                UT_Vector3 newPointPosition = UT_Vector3(poissonDisk.x(),poissonDisk.y(),poissonDisk.z());
                grad.normalize();
                UT_Vector3 newPointNormal = UT_Vector3(grad.x(),grad.y(),grad.z());


                //=================================================================
                //5:      if p meets the Poisson Disk criterion in S then
                //=================================================================
                int numberOfClosePoint;
                //cout << "Trying to fit "<<newPointPosition<<endl;
                bool meetPoissonDiskCriterion = backgroundGrid.RespectCriterion(trackersGdp,tree, newPointPosition, newPointNormal, poissonDiskRadius,poissonDiskRadius, cellSize, numberOfClosePoint, angleNormalThreshold, -1);
                if (meetPoissonDiskCriterion)
                {
                    //=================================================================
                    //6:          S ← S ∪ {p}
                    //=================================================================
                    bool isValid = this->InsertPoissonDisk(trackersGdp,tree, newPointPosition, newPointNormal, poissonDiskRadius, poissonDiskRadius, false, angleNormalThreshold);
                    if (isValid)
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
    //return allpoints;
    return;
}




//================================================================================================

//                                      INITIALIZE GRID

//================================================================================================

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



//================================================================================================

//                                 PROJECT POINT ON LEVEL SET

//================================================================================================


openvdb::Vec3f Bridson2012PoissonDiskDistribution::projectPointOnLevelSet(openvdb::Vec3f point, float distance, openvdb::Vec3f grad)
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

    return point;
}


//================================================================================================

//                                      INSERT POISSON DISK

//================================================================================================


bool Bridson2012PoissonDiskDistribution::InsertPoissonDisk(GU_Detail *trackersGdp, GEO_PointTreeGAOffset &tree, UT_Vector3 p, UT_Vector3 N, float diskRadius, float killDistance , bool existingPoint, float angleNormalThreshold)
{

    //get next available id
    long numberOfPoints = this->maxId+1;
    int numberOfClosePoint;
    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attActive(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));
    GA_RWHandleI    attDensity(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"density", 1));
    GA_RWHandleI    attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleF    attLife(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleI    attSpawn(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"spawn",1));
    GA_RWHandleI    attIsMature(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"isMature", 1));
    GA_RWHandleF    attMaxDeltaOnD(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"maxDeltaOnD",1));

    //cout << "Insert Poisson Disk "<< p<<endl;

    if (trackersGdp->getNumPoints() > this->maxId) //existing points
    {
        this->maxId = trackersGdp->getNumPoints();
    }
    int id = this->maxId+1;
    this->maxId = id;

    GA_Offset newPoint = trackersGdp->appendPoint();
    trackersGdp->setPos3(newPoint, p);
    attN.set(newPoint,N);
    if (!backgroundGrid.RespectCriterion(trackersGdp, tree, p, N, diskRadius, killDistance, cellSize,numberOfClosePoint, angleNormalThreshold, -1))
    {
        attActive.set(newPoint,false);
    }
    else
    {
        attActive.set(newPoint,true);
    }
    attDensity.set(newPoint,numberOfClosePoint);
    attId.set(newPoint,id);
    attSpawn.set(newPoint,0);
    attLife.set(newPoint,0.001f);
    attIsMature.set(newPoint,0);
    attMaxDeltaOnD.set(newPoint,0);
    tree.build(trackersGdp);

    return true;
}

