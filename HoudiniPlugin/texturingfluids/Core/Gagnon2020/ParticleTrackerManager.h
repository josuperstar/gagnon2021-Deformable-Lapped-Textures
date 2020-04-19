#ifndef __ParticleTracker_h__
#define __ParticleTracker_h__

#include <Math/Vec3.h>
#include <Strategies/StrategyPatchSurfaceSynthesis.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>
#include <Core/HoudiniUtils.h>
#include <openvdb/tools/Interpolation.h>

namespace TexturingFluids {

#define VERBOSE 0


class ParticleTrackerManager
{
public:

    ParticleTrackerManager(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params);

    vector<GA_Offset> PoissonDiskSamplingDistribution(GU_Detail *levelSet, float diskRadius, float angleNormalThreshold);

    GA_Offset CreateAParticle(UT_Vector3 newPointPosition, UT_Vector3 newPointNormal);

    void ProjectAndUpdateAllTrackers();

    bool ProjectTrackerOnSurface(GA_Offset ppt);
    bool UpdateTracker(GA_Offset ppt);

    void AdvectSingleTrackers();
    void DeleteTracker(int trackerId);

    int GetNumberOfPatches(){return numberOfPatches;}
    int NumberOfPatchesToDelete();

    const string markerGroupName = "markers";
    const string surfaceGroupName = "surface";
    const string approachName   = "[ParticleTrackerGagnon2020]";

    double  markerAdvectionTime;
    double poissondisk;
    int numberOfPatches;
    int numberOfInitialPatchFlagToDelete;
    int numberOfConcealedPatches;
    int numberOfNewPatches;
    int numberOfDetachedPatches;
    int numberOfLonelyTracker;
    int numberOfNewAndLonelyTracker;
    int numberOfInitialPatches;
    int numberOfDistortedPatches;
    int numberOfNewPoints;

protected :

    openvdb::Vec3f projectPointOnLevelSet(openvdb::Vec3f point, float distance, openvdb::Vec3f grad );
    bool RespectCriterion(UT_Vector3 newPointPosition, UT_Vector3 newPointNormal,  float killDistance, int &numberOfClosePoint, GA_Offset exclude);



    UT_Vector3 GetParamtrericCoordinate(GEO_Primitive *prim, GA_RWHandleV3 attribute, float u, float v);

    bool tackerPolygon = false;

    int maxId = 0;
    float epsilon = 0.0001;
    float poissonDiskRadius;    //radius
    int k;      //the limit of samples to choose before rejection in the algorithm, typically k = 30
    int n = 3; // n-dimensional
    int t; // number of attemps

    GA_RWHandleV3   attN;
    GA_RWHandleV3   attV;
    GA_RWHandleV3   attCenterUV;
    GA_RWHandleI    attId;
    GA_RWHandleF    attLife;
    GA_RWHandleI    attSpawn;
    GA_RWHandleI    attActive;
    GA_RWHandleI    attIsMature;
    GA_RWHandleI    attDensity;
    GA_RWHandleF    attBlend;

    GA_RWHandleF    attMaxDeltaOnD;
    GA_RWHandleI    attDeleteFaster;
    GA_RWHandleV3   refAttV;
    GA_RWHandleV3   refAttN;
    GA_RWHandleV3   tangeant;
    GA_RWHandleI    attFadeIn;
    GA_RWHandleF    temporalComponentKt;
    GA_RWHandleV3 AttCd;
    GA_RWHandleI    attNumberOfPrimitives;

    GA_RWHandleV3 attVSurface;
    //GA_RWHandleV3 attNSurface;
    GA_RWHandleF attDivergence;

    GU_Detail *surface;
    GU_Detail *trackersGdp;

    GA_PointGroup *surfaceGroup;
    GA_PrimitiveGroup *surfaceGrpPrims;
    GA_PointGroup *markerGrp;
    GA_PrimitiveGroup *markerGrpPrims;

    GEO_PointTreeGAOffset trackerTree;

    ParametersDeformablePatches params;
};

}

#endif
