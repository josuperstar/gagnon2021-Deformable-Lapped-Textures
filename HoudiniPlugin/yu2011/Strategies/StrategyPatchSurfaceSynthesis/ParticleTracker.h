#ifndef __ParticleTracker_h__
#define __ParticleTracker_h__

#include <Math/Vec3.h>
#include <Strategies/StrategyPatchSurfaceSynthesis.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>
#include <PoissonDisk/PoissonDisk.h>

namespace Mokko {

#define VERBOSE 0


class ParticleTracker : public StrategyPatchSurfaceSynthesis
{
public:

    ParticleTracker();
    bool SynthesisSurface( GU_Detail *trackerGdp, ParametersDeformablePatches params);

    //vector<GA_Offset>  PoissonDiskDistribution(GU_Detail *gdp, ParametersDeformablePatches params);
    vector<GA_Offset>  CreateTrackers(GU_Detail* surface,GU_Detail *trackers, GA_PointGroup *surfaceGroup, ParametersDeformablePatches params, vector<GA_Offset> referencePoints);
    void  CreateAndUpdateTrackersBasedOnPoissonDisk(GU_Detail* surface,GU_Detail *trackers, GA_PointGroup *surfaceGroup, ParametersDeformablePatches params);
    void AdvectMarkers(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree);
    void ComputeDivergence(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree);
    void ComputeDensity(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree);
    void DeleteTracker(GU_Detail* trackers,int trackerId);

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



};

}

#endif
