#include "ParticleTrackerManager.h"

#include <vector>
#include <algorithm>
#include <SYS/SYS_Math.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>
#include <SOP/SOP_Guide.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <GA/GA_ElementWrangler.h>
#include <algorithm>
#include <ctime>
#include <GEO/GEO_PointTree.h>
#include <GEO/GEO_PrimClassifier.h>
#include <GEO/GEO_PointClassifier.h>
#include <GEO/GEO_PrimConnector.h>
#include <GU/GU_NeighbourList.h>
#include <GU/GU_RayIntersect.h>
#include <GU/GU_Flatten.h>
#include <GEO/GEO_PrimVDB.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/GridOperators.h>

//#include <Strategies/StrategySurfaceTextureSynthesis.h>



ParticleTrackerManager::ParticleTrackerManager(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{
    this->surface = surfaceGdp;
    this->trackersGdp = trackersGdp;
    this->params = params;

    surfaceGroup = (GA_PointGroup *)surfaceGdp->pointGroups().find(this->surfaceGroupName.c_str());
    if (surfaceGroup == 0x0)
    {
        cout << "There is no surface group to synthesis"<<endl;
        return;
    }
    surfaceGrpPrims = (GA_PrimitiveGroup *)surface->primitiveGroups().find(this->surfaceGroupName.c_str());


    markerGrp = (GA_PointGroup *)trackersGdp->pointGroups().find(this->markerGroupName.c_str());
    if (markerGrp == 0x0)
        markerGrp = trackersGdp->newPointGroup(markerGroupName.c_str());

    if (tackerPolygon)
    {
        markerGrpPrims = (GA_PrimitiveGroup *)trackersGdp->primitiveGroups().find(this->markerGroupName.c_str());
        if (markerGrpPrims == 0x0)
            markerGrpPrims = trackersGdp->newPrimitiveGroup(markerGroupName.c_str());
    }

    //this->numberOfPatches = 0;
    this->maxId = 0;
    this->markerAdvectionTime = 0;
    this->numberOfConcealedPatches = 0;
    this->numberOfNewPatches = 0;
    this->numberOfDetachedPatches = 0;
    this->numberOfLonelyTracker = 0;
    this->numberOfNewAndLonelyTracker = 0;
    this->numberOfDistortedPatches = 0;

    this->numberOfInitialPatchFlagToDelete = 0;

    this->attN =  GA_RWHandleV3(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    this->attCenterUV =  GA_RWHandleV3(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"centerUV", 3));
    //GA_RWHandleV3   attV(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    this->attV = GA_RWHandleV3(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));

    this->attId  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    this->attLife  = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    this->attSpawn  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"spawn",1));
    this->attActive  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));
    this->attIsMature  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"isMature", 1));
    this->attDensity =  GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"density", 1));
    this->attBlend  = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    this->attMaxDeltaOnD  = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"maxDeltaOnD",1));
    this->attDeleteFaster =  GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"deleteFaster", 1));
    this->refAttV  = GA_RWHandleV3(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    this->refAttN  = GA_RWHandleV3(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    this->temporalComponentKt = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    this->attFadeIn  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"fadeIn",1));

    this->AttCd = GA_RWHandleV3(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));

    this->attNumberOfPrimitives  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"numberOfPrimitives",1));


    this->attVSurface = GA_RWHandleV3(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    this->attDivergence = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"divergence",1));


    this->numberOfInitialPatches = 0;

    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        int active = attActive.get(ppt);
        float currentLife = attLife.get(ppt);

        //Dead patches are not updated
        if (currentLife <= 0 && active == 0)
        {
            this->numberOfInitialPatchFlagToDelete++;
            continue;
        }
        this->numberOfInitialPatches++;
    }
    cout <<this->approachName<< " Initialization with "<<this->numberOfInitialPatches<<" and "<<this->numberOfInitialPatchFlagToDelete << " flaged to delete"<<endl;
    this->numberOfPatches = this->numberOfInitialPatches;
}

vector<GA_Offset> ParticleTrackerManager::PoissonDiskSamplingDistribution(GU_Detail *levelSet, float diskRadius, float angleNormalThreshold)
{
    cout << "[Bridson2012PoissonDiskDistributionGagnon2020] on level set using a threshold of "<<angleNormalThreshold<<endl;

    GA_RWHandleV3   attN(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attActive(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));
    GA_RWHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleI    attDensity(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"density", 1));
    GA_RWHandleI    isTangeantTracker(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"isTrangeantTracker",1));

    vector<GA_Offset> newPoissonDisk;

    this->numberOfNewPoints = 0;

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
        cout << "[Bridson2012PoissonDiskDistributionGagnon2020] Input geometry 0 has no VDB grid!"<<endl;
        return newPoissonDisk;
    }

//    cout << "Grid name: " << phi->getGridName() << std::endl;
//    cout << "Storage type: " << phi->getStorageType() << ", " << phi->getTupleSize() << std::endl;
//    cout << "JSON: " << phi->getJSON() << std::endl;

    float a = 0.25; //promote this variable to the user interface
    this->poissonDiskRadius = diskRadius;
    float killDistance = (1-a)*diskRadius/2;

    cout << "[Bridson2012PoissonDiskDistributionGagnon2020] We have a valid vdb"<<endl;

    GA_RWHandleI    attDeleteFaster(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"deleteFaster", 1));
    GA_Offset ppt;
    // Only if we want to delete too close patches, which is not the case when we don't use fading in

    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        int numberOfClosePoint;

        if (isTangeantTracker.isValid())
        {
            if (isTangeantTracker.get(ppt) == 1)
            {
                continue;
            }
        }
        UT_Vector3 pointPosition = trackersGdp->getPos3(ppt);
        UT_Vector3 pointNormal   = attN.get(ppt);
        if (attId.get(ppt) > this->maxId)
            this->maxId = attId.get(ppt);


        bool meetPoissonDiskCriterion = this->RespectCriterion(pointPosition, pointNormal, killDistance,  numberOfClosePoint, ppt);
        attDensity.set(ppt,numberOfClosePoint);

        if (attActive.get(ppt) == 0)
            continue;
        //If we have fading in, we are using 2019's approach
        //if (params.fadingIn == 1)
        {
            //attActive.set(ppt,meetPoissonDiskCriterion);
        }
        if (!meetPoissonDiskCriterion)
        {
            //cout << "We should delete point "<<attId.get(ppt)<<", is in kill distance" <<killDistance<<endl;
        }
    }


    t = 30;

//    cout << "Grid name: " << phi->getGridName() << std::endl;
//    cout << "Storage type: " << phi->getStorageType() << ", " << phi->getTupleSize() << std::endl;
//    cout << "JSON: " << phi->getJSON() << std::endl;

    openvdb::GridBase::Ptr ptr = phi->getGridPtr();
    openvdb::FloatGrid::Ptr gridSurface = openvdb::gridPtrCast<openvdb::FloatGrid>(ptr);
    if(!gridSurface)
    {
        cout << "[Bridson2012PoissonDiskDistribution] Surface grid can't be converted in FloatGrid"<<endl;
        return newPoissonDisk;
    }
    if ((gridSurface->getGridClass() != openvdb::GRID_LEVEL_SET))
    {
        cout<< "[Bridson2012PoissonDiskDistribution] Surface grid is not a Level-set FloatGrid!"<<endl;
        return newPoissonDisk;
    }

    //=================================================================
    //                         OPEN VDB ACCESSORS
    //=================================================================

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
    int nbOfCell = 0;

    //cout << "[Bridson2012PoissonDiskDistribution] Step 1: for all grid cells C where φ changes sign do"<<endl;
    for (openvdb::FloatGrid::ValueOnCIter gridCellIt = gridSurface->cbeginValueOn(); gridCellIt; ++gridCellIt)
    {
        float x = gridCellIt.getCoord().x();
        float y = gridCellIt.getCoord().y();
        float z = gridCellIt.getCoord().z();

        float offset = 0.5;

        openvdb::Vec3f cellPosition(x,y,z);
        openvdb::Vec3f worldCellPos = gridSurface->transform().indexToWorld(cellPosition);
        float boundaryDist = samplerSurface.wsSample(worldCellPos);
        //openvdb::Vec3f p    = it.getCoord();
        //if (boundaryDist <= 0.0)// && grad.length() > 0.0)
        {
            //if it is not close to the surface, continue
            if (abs(boundaryDist) > params.CellSize/2.0f) // We should use a threshold defined by the user
                continue;
            //=================================================================
            //2:  for t attempts do
            //=================================================================
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
                randomPosition *= params.CellSize;

                openvdb::Vec3f p = worldCellPos+randomPosition;

                float newPointDistance = samplerSurface.wsSample(p);
                if (abs(newPointDistance) > params.poissondiskradius)
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
                bool meetPoissonDiskCriterion = this->RespectCriterion(newPointPosition, newPointNormal, poissonDiskRadius, numberOfClosePoint, -1);
                if (meetPoissonDiskCriterion)
                {
                    //=================================================================
                    //6:          S ← S ∪ {p}
                    //=================================================================

                    GA_Offset newPoint = this->CreateAParticle(newPointPosition, newPointNormal, numberOfClosePoint);
                    newPoissonDisk.push_back(newPoint);
                    break;
                }
            }
            if (!ableToInsertPoint)
            {
                //cout << "after "<<t<<" attemps, there is no possible insertion."<<endl;
            }
        }
        nbOfCell++;
    }
    //cout << nbOfCell << " cells have been treated."<<endl;
    return newPoissonDisk;
}

GA_Offset ParticleTrackerManager::CreateAParticle(UT_Vector3 p, UT_Vector3 N,  int &numberOfClosePoint)
{

//    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
//    GA_RWHandleI    attActive(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));
//    GA_RWHandleI    attDensity(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"density", 1));
//    GA_RWHandleI    attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
//    GA_RWHandleF    attLife(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
//    GA_RWHandleI    attSpawn(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"spawn",1));
//    GA_RWHandleI    attIsMature(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"isMature", 1));
//    GA_RWHandleF    attMaxDeltaOnD(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"maxDeltaOnD",1));
//    GA_RWHandleF    attExistingLife(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life", 1));
//    GA_RWHandleI    attNumberOfPrimitives(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"numberOfPrimitives", 1));

    int divider = 1;
    if (params.useTangeantTracker == 1)
        divider = 2;
    if (trackersGdp->getNumPoints()/divider > this->maxId) //existing points
    {
        //cout << "New Max Id = "<<trackersGdp->getNumPoints()/divider<<endl;
        this->maxId = trackersGdp->getNumPoints()/divider;
    }
    int id = this->maxId+1;
    this->maxId = id;
    //cout << "New Max Id = "<<this->maxId<<endl;
    GA_Offset newPoint = trackersGdp->appendPoint();
    trackersGdp->setPos3(newPoint, p);
    attN.set(newPoint,N);
    attActive.set(newPoint,true);
    attDensity.set(newPoint,numberOfClosePoint);
    attId.set(newPoint,id);
    attSpawn.set(newPoint,0);
    attLife.set(newPoint,0.001f);
    attNumberOfPrimitives.set(newPoint,0);
    attIsMature.set(newPoint,0);
    attMaxDeltaOnD.set(newPoint,0);

    if(params.startFrame == params.frame)
    {
        attLife.set(newPoint,params.fadingTau);
    }
    this->trackerTree.build(trackersGdp);

    this->numberOfNewPoints++;

    return newPoint;
}


//================================================================================================

//                                 PROJECT POINT ON LEVEL SET

//================================================================================================


openvdb::Vec3f ParticleTrackerManager::projectPointOnLevelSet(openvdb::Vec3f point, float distance, openvdb::Vec3f grad)
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


bool ParticleTrackerManager::RespectCriterion(UT_Vector3 newPointPosition, UT_Vector3 newPointNormal, float killDistance, int &numberOfClosePoint,   GA_Offset exclude )
{
    numberOfClosePoint = 0;

    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attActive(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"active", 1));

    GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;

    this->trackerTree.findAllCloseIdx(newPointPosition,
                         params.poissondiskradius*2,
                         close_particles_indices);

    int l = (int)close_particles_indices.entries();
    GA_Offset neighbor;
    bool tooClose = false;

    float cs    = params.CellSize;
    float r     = params.poissondiskradius;

    newPointNormal.normalize();
    float kd = killDistance;

    UT_Vector3 defaultDirection(1.012f,0.123f,0.002f);
    UT_Vector3 S,T;

    for(int j=0; j<l;j++)
    {
        neighbor = close_particles_indices.array()[j];
        if (attActive.get(neighbor) == 0)
            continue;
        if (neighbor == exclude)
            continue;

        UT_Vector3 pos          = trackersGdp->getPos3(neighbor);

        UT_Vector3 N            = attN.get(neighbor);
        N.normalize();
        S = cross(N,defaultDirection);
        S.normalize();
        T = cross(S,N);
        T.normalize();


        // Transform into local patch space (where STN is aligned with XYZ at the origin)
        const UT_Vector3 relativePosistion = pos - newPointPosition;
        UT_Vector3 poissonDiskSpace;
        poissonDiskSpace.x() = relativePosistion.dot(S);
        poissonDiskSpace.y() = relativePosistion.dot(T);
        poissonDiskSpace.z() = relativePosistion.dot(N);

        float dotN              = dot(N,newPointNormal);
        bool samePlane          = dotN > params.poissonAngleNormalThreshold;

        //(x/a)2 + (y/b)2 + (z/c)2 = 1
        float x = poissonDiskSpace.x();
        float y = poissonDiskSpace.y();
        float z = poissonDiskSpace.z();
        float a = r;
        float b = r;
        float c = cs*2;

        float a2 = kd;
        float b2 = kd;
        float c2 = cs;

        float smallEllipse = (x/a2)*(x/a2) + (y/b2)*(y/b2) + (z/c2)*(z/c2);
        float bigEllipse = (x/a)*(x/a) + (y/b)*(y/b) + (z/c)*(z/c);

        bool outsideOfSmallEllipse = false;
        bool insideBigEllipse = false;

        if (bigEllipse <= 1)
            insideBigEllipse = true;
        if (smallEllipse > 1)
            outsideOfSmallEllipse = true;

        //It is too close to the current point ?
        if(samePlane && !outsideOfSmallEllipse)
        {
            tooClose = true;
        }

        if(insideBigEllipse && samePlane)
            numberOfClosePoint++;
    }
    return !tooClose;
}


int ParticleTrackerManager::NumberOfPatchesToDelete()
{
    int toDelete = 0;

    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        int active = attActive.get(ppt);
        float currentLife = attLife.get(ppt);

        //Dead patches are not updated
        if (currentLife <= 0 && active == 0)
        {
            toDelete++;
            continue;
        }
    }

    return toDelete;
}


UT_Vector3 ParticleTrackerManager::GetParamtrericCoordinate(GEO_Primitive *prim, GA_RWHandleV3 attribute, float u, float v)
{
    GA_Offset vertexOffset0 = prim->getVertexOffset(0);

    GA_Offset pointOffset0  = this->surface->vertexPoint(vertexOffset0);
    UT_Vector3 v0 = attribute.get(pointOffset0);

    GA_Offset vertexOffset1 = prim->getVertexOffset(1);
    GA_Offset pointOffset1  = this->surface->vertexPoint(vertexOffset1);
    UT_Vector3 v1 = attribute.get(pointOffset1);

    GA_Offset vertexOffset2 = prim->getVertexOffset(2);
    GA_Offset pointOffset2  = this->surface->vertexPoint(vertexOffset2);

    UT_Vector3 v2 = attribute.get(pointOffset2);

    UT_Vector3 result = v0+u*(v1-v0)+v*(v2-v0);
    return result;
}


//================================================================================================

//                                      CREATE TRACKER BASED ON POISSON DISK

//================================================================================================


void ParticleTrackerManager::CreateAndUpdateTrackerBasedOnPoissonDisk(GA_Offset ppt)
{

    bool useDynamicTau = params.useDynamicTau;


    UT_Vector3 position;
    UT_Vector3 N;

    float thresholdDistance = params.maximumProjectionDistance;

    GU_MinInfo mininfo;
    GU_RayIntersect ray(surface);
    ray.init();

    int id = 0;

    int deletedTrackers = 0;

    id = attId.get(ppt);
    int active = attActive.get(ppt);
    float currentLife = attLife.get(ppt);
    int currentSpawn = attSpawn.get(ppt);


    UT_Vector3 velocity;

    float dynamicTau = attMaxDeltaOnD.get(ppt);
    UT_Vector3 centerUV = attCenterUV.get(ppt);

    //Dead patches are not updated
    if (currentLife <= 0 && active == 0)
    {
        deletedTrackers++;
        return;
    }

    //============================ PROJECTION ON MESH =======================
    UT_Vector3 p1 = trackersGdp->getPos3(ppt);
    mininfo.init(thresholdDistance,0.0001);
    ray.minimumPoint(p1,mininfo);

    if (!mininfo.prim)
    {
        //cout << "No primitive to project on"<<endl;
        attLife.set(ppt,0);
        attActive.set(ppt,0);
        this->numberOfDetachedPatches++;
        return;
    }

    const GEO_Primitive *geoPrim = mininfo.prim;
    int vertexCount = geoPrim->getVertexCount();
    if (vertexCount != 3)
    {
        //cout << "vertex count "<<vertexCount<<" for primitive "<<geoPrim->getMapOffset()<<endl;
        attLife.set(ppt,0);
        attActive.set(ppt,0);
        this->numberOfDetachedPatches++;
        return;
    }
    //get pos of hit
    UT_Vector4 hitPos;
    mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
    if (distance3d(p1,hitPos) < thresholdDistance)
    {
        p1 = hitPos;
        //------------------------------PARAMETRIC COORDINATE -----------------------------------
        GA_Offset primOffset = mininfo.prim->getMapOffset();
        float u = mininfo.u1;
        float v = mininfo.v1;
        GEO_Primitive *prim = surface->getGEOPrimitive(primOffset);

        //Check if the prim is part of the group 'Surface':
        if (!surfaceGrpPrims->containsOffset(primOffset))
        {
            if (currentSpawn <= 1) // we just had it
            {
                attLife.set(ppt,0);
                attActive.set(ppt,0);
                return;
            }
        }
        N = GetParamtrericCoordinate(prim, refAttN, u, v);
        velocity = GetParamtrericCoordinate(prim, refAttV, u, v);
    }
    else
    {
        //can't project, we delete
        attLife.set(ppt,0);
        attActive.set(ppt,0);
        this->numberOfDetachedPatches++;
        return;
    }

    //========================================================================

    //========================= UPDATE ===============================
    //we want to fade out poisson disk that are flagged a inactive and that are mature (life spawn greater than the fading in time)
    //or that are too close to each other

    int maxNumberOfNeighbour = 5; // TODO promotve that variable
    int density = attDensity.get(ppt);
    //-------------- deleting faster logic ------------------
    //Can we move this to the ParticleTracker update ?
    int deleteFaster = 0;
    if (params.fadingIn == 0)
    {
        int deleteFaster = attDeleteFaster.get(ppt);
        int numberOfNeighbourThreshold = 1; // TODO: promote this variable
        if (density > numberOfNeighbourThreshold && deleteFaster == 0)
        {
            attDeleteFaster.set(ppt, 1);
        }
        else if(deleteFaster == 1 && density <= numberOfNeighbourThreshold)
        {
            attDeleteFaster.set(ppt, 0);
        }
    }
    //-------------------------------------------------------

    int increment = density;
    if (maxNumberOfNeighbour <= density)
        increment = maxNumberOfNeighbour;

    if (!useDynamicTau)
        increment = 0;

    //int deleteFaster = attDeleteFaster.get(ppt);
    bool isMature = (currentSpawn >= params.fadingTau);
    if (params.fadingIn == 0)
        isMature = true;
    if (params.fadingIn == 0 and currentSpawn != 0)
    {
        currentSpawn++;
    }

    if (isMature)
        attIsMature.set(ppt,1);

    if (active == 0 && deleteFaster == 1 && isMature)
    {
        currentLife -= 1.0f+((float)increment);
    }
    else if(active == 0 && deleteFaster == 0 && isMature)
    {
        currentLife -= 1.0f;
    }
    //fade in
    else if (currentSpawn < params.fadingTau)
    {

        currentLife += 1.0f+(float)increment;
        if (currentSpawn == 0)
            currentSpawn+= 1;
        else
            currentSpawn+= 1+increment;

        currentLife = params.fadingTau;

    }
    if (currentLife > (float)params.fadingTau)
        currentLife = (float)params.fadingTau;
    if (currentLife < 0)
        currentLife = 0;

    float deletionLife = params.fadingTau;
    float blending = (float)currentLife/(float(deletionLife));
    attBlend.set(ppt,blending);

    //==============================================

    position = p1;
    trackersGdp->setPos3(ppt,position);
    N.normalize();

    attV.set(ppt,velocity);
    attN.set(ppt,N);
    attCenterUV.set(ppt,centerUV);
    trackersGdp->setPos3(ppt,position);

    float life = currentLife;
    attLife.set(ppt,life);

    float temporalComponetKt = ((float)life)/params.fadingTau;

    attBlend.set(ppt,temporalComponetKt);
    attSpawn.set(ppt,currentSpawn);
    attMaxDeltaOnD.set(ppt,dynamicTau);
}


//================================================================================================

//                                      CREATE TRACKERS BASED ON POISSON DISK

//================================================================================================


void ParticleTrackerManager::CreateAndUpdateTrackersBasedOnPoissonDisk()
{
    bool useDynamicTau = params.useDynamicTau;
    cout <<this->approachName<< " CreateTrackersBasedOnPoissonDisk, with useDynamicTau at "<<useDynamicTau <<endl;

    GA_Offset ppt;
    int deletedTrackers = 0;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        this->CreateAndUpdateTrackerBasedOnPoissonDisk(ppt);
    }

    cout <<this->approachName<< " Deleted trackers: "<<deletedTrackers<<endl;
    cout <<this->approachName<< " New And Lonely Tracker "<<numberOfNewAndLonelyTracker<<endl;
    cout <<this->approachName<< " Total trackers: "<<trackersGdp->getNumPoints() - deletedTrackers - numberOfNewAndLonelyTracker<<endl;
}


//================================================================================================

//                                      ADVECT SINGLE MARKERS

//================================================================================================


void ParticleTrackerManager::AdvectSingleTrackers()
{
    cout <<this->approachName<< " Advect Single Trackers"<<endl;

    std::clock_t startAdvection;
    startAdvection = std::clock();

    //GA_RWHandleV3 refAttN(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));

    maxId = 0;

    if (attV.isInvalid())
    {
        cout << "Trackers have no velocity";
        return;
    }

    if (attDensity.isInvalid())
    {
        cout << "The is no density defined"<<endl;
        return;
    }
    UT_Vector3 N;
    UT_Vector3 v;
    UT_Vector3 p;
    UT_Vector3 p1;
    float dt = 1.0f/24.0f;
    float thresholdDistance = params.maximumProjectionDistance;
    int numberOfPatchBefore = this->numberOfPatches;
    //--------------------------------------------------
    {
        GU_MinInfo mininfo;

        GU_RayIntersect ray(this->surface);
        ray.init();

        GA_Offset ppt;
        int id;
        int density;
        float currentLife = 0;
        cout <<this->approachName<< " Start advection loop"<<endl;
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {
            v = attV.get(ppt);
            N = attN.get(ppt);
            density = attDensity.get(ppt);

            if (N.length() < epsilon)
            {
                continue;
            }
            p = trackersGdp->getPos3(ppt);

            id = attId.get(ppt);
            if(id > maxId)
                maxId = id;

            currentLife = attLife.get(ppt);

            //-----------------------------------------
            //advection
            UT_Vector3 d = v*dt;
            p1 = p+d;
            trackersGdp->setPos3(ppt,p1);
            //-----------------------------------------

            p1 = trackersGdp->getPos3(ppt);

            mininfo.init(thresholdDistance,0.0001);
            ray.minimumPoint(p1,mininfo);

            if (!mininfo.prim)
            {
                cout << "No primitive to project on"<<endl;
                continue;
            }

            const GEO_Primitive *geoPrim = mininfo.prim;
            int vertexCount = geoPrim->getVertexCount();
            if (vertexCount != 3)
            {
                //cout << "vertex count "<<vertexCount<<" for primitive "<<geoPrim->getMapOffset()<<endl;
                continue;
            }
            //get pos of hit
            UT_Vector4 hitPos;
            mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
            if (distance3d(p1,hitPos) < thresholdDistance)
            {
                p1 = hitPos;
                trackersGdp->setPos3(ppt,p1);
                AttCd.set(ppt,UT_Vector3(0,1,0));

                //------------------------------PARAMETRIC COORDINATE -----------------------------------
                GA_Offset primOffset = mininfo.prim->getMapOffset();
                float u = mininfo.u1;
                float v = mininfo.v1;
                GEO_Primitive *prim = this->surface->getGEOPrimitive(primOffset);
                int numberOfVertices = prim->getVertexCount();
                if (numberOfVertices != 3)
                    continue;

                UT_Vector3 N = GetParamtrericCoordinate(prim, refAttN, u, v);
                UT_Vector3 velocity = GetParamtrericCoordinate(prim, refAttV, u, v);
                attV.set(ppt,velocity);
                attN.set(ppt,N);
                //------------------------------------------------------------------------------------
            }
            else
            {
                //delete this point because we can't project it, probably because of a sudden topological change.
                //cout << "delete "<<id<<" because we can't project it, probably because of a sudden topological change."<<endl;
                AttCd.set(ppt,UT_Vector3(1,0,0));

                //detached poisson disks have to be deleted directly, not fading out.
                attLife.set(ppt,0);
                attActive.set(ppt,0);

                numberOfDetachedPatches++;

                trackersGdp->setPos3(ppt,p1);
            }
        }
    }

    //----------------------------------
    cout << this->approachName<< " There are "<<numberOfPatchBefore << " trackers after advection"<<endl;
    cout << this->approachName<< " There are "<<numberOfDetachedPatches<< " detached trackers"<<endl;
    cout << this->approachName<< " There are "<<numberOfPatches << " number of patches"<<endl;

    this->markerAdvectionTime += (std::clock() - startAdvection) / (double) CLOCKS_PER_SEC;

}




