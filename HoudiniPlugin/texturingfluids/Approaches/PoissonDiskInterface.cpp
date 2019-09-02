#include "PoissonDiskInterface.h"

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
#include <GU/GU_NeighbourList.h>

#include <GU/GU_Flatten.h>
#include <GU/GU_RayIntersect.h>

#include <Core/PatchedSurface.h>
#include <Core/Bridson2012PoissonDiskDistribution.h>

PoissonDiskInterface::PoissonDiskInterface()
{
}

PoissonDiskInterface::~PoissonDiskInterface()
{
}

void PoissonDiskInterface::Synthesis(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GU_Detail *levelSet,  ParametersDeformablePatches params)
{
    PatchedSurface strategy(surfaceGdp, trackersGdp);
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


    GEO_PointTreeGAOffset surfaceTree;
    surfaceTree.build(surfaceGdp, NULL);

    //=========================== CORE ALGORITHM ============================
    strategy.PoissonDiskSampling(levelSet,trackersGdp,params);
    strategy.CreateAndUpdateTrackersBasedOnPoissonDisk(surfaceGdp,trackersGdp, surfaceGroup,params);
    strategy.AdvectSingleTrackers(surfaceGdp,trackersGdp, params);
    //=======================================================================

    cout << strategy.approachName<<" Done"<<endl;
    cout << "Clear surface tree"<<endl;
    surfaceTree.clear();

    cout << strategy.approachName<< " saving trackers data to ";
    const char* filenameTrackers = params.trackersFilename.c_str();//"dlttest.bgeo";
    cout << filenameTrackers<<endl;
    trackersGdp->save(filenameTrackers,options,errors);

    //================================================================
    std::clock_t cleaningStart;
    cleaningStart = std::clock();

    float cleaningSurface = (std::clock() - cleaningStart) / (double) CLOCKS_PER_SEC;
    cout << "--------------------------------------------------------------------------------"<<endl;
    cout << strategy.approachName<<" Poisson Disk Sampling "<<strategy.poissondisk<<endl;

    float total = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    cout << strategy.approachName<< " TOTAL: "<<total<<endl;
    cout << "--------------------------------------------------------------------------------"<<endl;
}

