#ifndef __ParticleTracker_h__
#define __ParticleTracker_h__

#include <Math/Vec3.h>
#include <Strategies/StrategyPatchSurfaceSynthesis.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

namespace TexturingFluids {

#define VERBOSE 0


class ParticleTrackerManager
{
public:

    ParticleTrackerManager(GU_Detail *surfaceGdp, GU_Detail *trackersGdp);
    void  CreateAndUpdateTrackersBasedOnPoissonDisk(GU_Detail* surface,GU_Detail *trackers, GA_PointGroup *surfaceGroup, ParametersDeformablePatches params);
    void  UpdateTrackersAndTangeant(GU_Detail* surface,GU_Detail *trackers, GA_PointGroup *surfaceGroup, ParametersDeformablePatches params);
    void AdvectSingleTrackers(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params);
    void AdvectTrackersAndTangeants(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params);
    void ComputeDivergence(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree);
    void ComputeDensity(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree);
    void DeleteTracker(GU_Detail* trackers,int trackerId);

    int GetNumberOfPatches(){return numberOfPatches;}

    const string markerGroupName = "markers";
    const string surfaceGroupName = "surface";
    const string approachName   = "[Particle Tracker]";

    double  markerAdvectionTime;


protected :

    bool tackerPolygon = false;
    int numberOfPatches;
    int numberOfConcealedPatches;
    int numberOfNewPatches;
    int numberOfDetachedPatches;
    int maxId = 0;
    const char*    randomThresholdDistortion = "t_rand";
    float epsilon = 0.0001;

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
    GA_RWHandleF    attRandT;
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


};

}

#endif
