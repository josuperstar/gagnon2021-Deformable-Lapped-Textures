#include "Yu2011Interface.h"

#include <vector>
#include <algorithm>
#include <SYS/SYS_Math.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_SpareData.h>
#include <SOP/SOP_Guide.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <GA/GA_ElementWrangler.h>
#include <algorithm>
#include <ctime>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_NeighbourList.h>

#include <GU/GU_Flatten.h>
#include <GU/GU_RayIntersect.h>

#include <Strategies/StrategyPatchSurfaceSynthesis/Yu2011.h>

#include <Core/Atlas/HoudiniAtlas.h>
#include <Core/Atlas/TBBAtlas.h>
#include <PoissonDisk/Bridson2012PoissonDiskDistribution.h>

Yu2011Interface::Yu2011Interface()
{
}

Yu2011Interface::~Yu2011Interface()
{
}

void Yu2011Interface::Synthesis(GU_Detail *gdp, GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GU_Detail *levelSet,  ParametersDeformablePatches params)
{
    Yu2011 strategy(surfaceGdp);
    cout << "[Yu2011Interface::Synthesis] "<<params.frame<<endl;
    params.useDynamicTau = false;

    std::clock_t start;
    start = std::clock();

    vector<GA_Offset> newPatchesPoints;
    vector<GA_Offset> trackers;
    cout << "reference gdp created"<<endl;
    const GA_SaveOptions *options;
    UT_StringArray *errors;

    GA_PointGroup *surfaceGroup = (GA_PointGroup *)surfaceGdp->pointGroups().find(strategy.surfaceGroupName.c_str());
    if (surfaceGroup == 0x0)
    {
        cout << "There is no surface group to synthesis"<<endl;
        return;
    }
    //=======================================================
    GA_PointGroup *grp = (GA_PointGroup *)gdp->pointGroups().find(strategy.markerGroupName.c_str());

    GU_RayIntersect ray(gdp);
    ray.init();
    GEO_PointTreeGAOffset surfaceTree;
    surfaceTree.build(surfaceGdp, NULL);

    //=========================== CORE ALGORITHM ============================


    //---- for visualisation purpose

    //string beforeUpdateString = params.trackersFilename + "beforeAdvection.bgeo";
    //const char* filename = beforeUpdateString.c_str();//"dlttest.bgeo";
    //trackersGdp->save(filename,options,errors);
    //----------------------------------


    if(params.startFrame == params.frame)
    {
        vector<PoissonDisk> PPoints = strategy.PoissonDiskSampling(gdp,levelSet,trackersGdp,grp,params);
        strategy.CreateAndUpdateTrackersBasedOnPoissonDisk(surfaceGdp,trackersGdp, surfaceGroup,params,PPoints);
        newPatchesPoints = strategy.AdvectMarkers(surfaceGdp,trackersGdp, params,surfaceTree);
        strategy.CreateGridBasedOnMesh(gdp,surfaceGdp,trackersGdp, params,newPatchesPoints,surfaceTree);
    }
    else
    {
        newPatchesPoints = strategy.AdvectMarkers(surfaceGdp,trackersGdp, params,surfaceTree);
        strategy.AdvectGrids(gdp,trackersGdp,params,surfaceTree,newPatchesPoints,surfaceGdp);
        vector<PoissonDisk> PPoints = strategy.PoissonDiskSampling(gdp,levelSet,trackersGdp,grp,params); //Poisson disk on the level set
        strategy.CreateAndUpdateTrackersBasedOnPoissonDisk(surfaceGdp,trackersGdp, surfaceGroup,params,PPoints);
        strategy.UpdateDistributionUsingBridson2012PoissonDisk(gdp,surfaceGdp, trackersGdp,params,surfaceTree,ray);

    }
    //For the blending computation, we create uv array per vertex that we called patch
    strategy.AddPatchesUsingBarycentricCoordinates(gdp, surfaceGdp,trackersGdp, params,newPatchesPoints,surfaceTree,ray);

    //=======================================================================

    cout << strategy.approachName<<" Done"<<endl;
    cout << "Clear surface tree"<<endl;
    surfaceTree.clear();
    ray.clear();

    cout << strategy.approachName<< " saving grids data"<<endl;
    const char* filenameGrids = params.deformableGridsFilename.c_str();//"dlttest.bgeo";
    gdp->save(filenameGrids,options,errors);

    cout << strategy.approachName<< " saving trackers data"<<endl;
    const char* filenameTrackers = params.trackersFilename.c_str();//"dlttest.bgeo";
    trackersGdp->save(filenameTrackers,options,errors);

    //================================================================
    std::clock_t cleaningStart;
    cleaningStart = std::clock();
    cout<< "Clear, Destroy and merge"<<endl;
    gdp->clearAndDestroy();
    gdp->copy(*surfaceGdp);

    float cleaningSurface = (std::clock() - cleaningStart) / (double) CLOCKS_PER_SEC;
    cout << "--------------------------------------------------------------------------------"<<endl;
    cout << strategy.approachName<<" Poisson Disk Sampling "<<strategy.poissondisk<<endl;
    cout << strategy.approachName<<" Grid mesh on time "<<strategy.gridMeshCreation<<endl;
    cout << strategy.approachName<<" Uv flattening time "<<strategy.uvFlatteningTime<<" for "<<strategy.nbOfFlattenedPatch<<" patches"<<endl;
    cout << strategy.approachName<<" Tracker advection time "<<strategy.markerAdvectionTime<<endl;
    cout << strategy.approachName<<" Grid advection time "<<strategy.gridAdvectionTime<<endl;
    cout << strategy.approachName<<" Patch creation time "<<strategy.patchCreationTime<<endl;
    cout << strategy.approachName<<" Clear and Destroy "<<cleaningSurface<<endl;
    cout << strategy.approachName<<" Update distribution "<<strategy.updatePatchesTime<<endl;

    float total = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    cout << strategy.approachName<< " TOTAL: "<<total<<endl;
    cout << "--------------------------------------------------------------------------------"<<endl;
}

