#include "SinglePatchInterface.h"

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

#include <Core/Gagnon2020/PatchedSurface.h>


SinglePatchInterface::SinglePatchInterface()
{
}

SinglePatchInterface::~SinglePatchInterface()
{
}

void SinglePatchInterface::Synthesis(GU_Detail *gdp, GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GU_Detail *surfaceLowResGdp,  ParametersDeformablePatches params)
{
    PatchedSurfaceGagnon2020 strategy(surfaceGdp, surfaceLowResGdp, trackersGdp,gdp, params);
    cout << "[Yu2011Interface::Synthesis] "<<params.frame<<endl;
    //params.useDynamicTau = false;

    std::clock_t start;
    start = std::clock();

    vector<GA_Offset> newPatchesPoints;

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



    //=========================== CORE ALGORITHM ============================

    bool usingOnlyPoissonDisk = false;

    if(params.startFrame == params.frame)
    {
        cout << "Creating a single poisson disk"<<endl;
        UT_Vector3 position(0,0,0);
        UT_Vector3 normal(0,1,0);
        strategy.CreateAPatch(position, normal);
        strategy.CreateAndUpdateTrackersBasedOnPoissonDisk();
        if (!usingOnlyPoissonDisk)
            strategy.CreateGridsBasedOnMesh(newPatchesPoints);
    }
    else
    {
        bool testAdvection = false;
        //TODO add a paramter to test the advection
        if (testAdvection)
        {
            strategy.AdvectSingleTrackers();
            strategy.AdvectGrids();
        }
        //strategy.PoissonDiskSampling(gdp,levelSet,trackersGdp,grp,params); //Poisson disk on the level set
        strategy.CreateAndUpdateTrackersBasedOnPoissonDisk();
        if (!usingOnlyPoissonDisk)
            strategy.CreateGridsBasedOnMesh(newPatchesPoints);
        strategy.DeleteUnusedPatches();

    }
    if (!usingOnlyPoissonDisk)
    {
        //For the blending computation, we create uv array per vertex that we called patch
        strategy.AddDeformablePatchesUsingBarycentricCoordinates();
    }

    //=======================================================================

    cout << strategy.approachName<<" Done"<<endl;
    cout << "Clear surface tree"<<endl;

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

