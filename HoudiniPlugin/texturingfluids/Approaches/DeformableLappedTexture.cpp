#include "DeformableLappedTexture.h"

#include <fstream>
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

#include <Core/Atlas/AtlasTestingConcealed.h>
#include <Core/Atlas/TBBAtlasTestingConcealed.h>

DeformableLappedTexture::DeformableLappedTexture()
{
}

DeformableLappedTexture::~DeformableLappedTexture()
{
}

void DeformableLappedTexture::Synthesis(GU_Detail *deformableGridGdp, GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GU_Detail *levelSet, GU_Detail *surfaceLowResGdp,  ParametersDeformablePatches params)
{
    PatchedSurfaceGagnon2020 surface(surfaceGdp, surfaceLowResGdp, trackersGdp,deformableGridGdp, params);
    cout << "[DeformableLappedTexture::Synthesis] Version: (put version here) frame: "<<params.frame<<endl;
    //params.useDynamicTau = false;

    std::clock_t start;
    start = std::clock();
    vector<GA_Offset> newPatchesPoints;

    cout << "reference gdp created"<<endl;
    const GA_SaveOptions *options;
    UT_StringArray *errors;


    //=======================================================

    //=========================== CORE ALGORITHM ============================

    //---- for visualisation purpose
    //string beforeUpdateString = params.trackersFilename + "beforeAdvection.bgeo";
    //const char* filename = beforeUpdateString.c_str();//"dlttest.bgeo";
    //trackersGdp->save(filename,options,errors);
    //----------------------------------


    bool usingOnlyPoissonDisk = false;

    if(params.startFrame == params.frame)
    {
        surface.PoissonDiskSamplingDistribution(levelSet,params.poissondiskradius, params.poissonAngleNormalThreshold);
        //----------------- TO get rid of -----------
        surface.CreateAndUpdateTrackersBasedOnPoissonDisk();
        if (!usingOnlyPoissonDisk)
            surface.CreateGridsBasedOnMesh(newPatchesPoints);

        surface.AddDeformablePatchesUsingBarycentricCoordinates();
    }
    else
    {
        cout << "------------------- Advection ---------------------"<<endl;
        surface.AdvectSingleTrackers();
        surface.AdvectGrids();


        if (params.updateDistribution)
        {
            cout << "------------------- Sampling ---------------------"<<endl;
            surface.PoissonDiskSamplingDistribution(levelSet,params.poissondiskradius, params.poissonAngleNormalThreshold); //Poisson disk on the level set
        }
        cout << "------------------- Updating Trackers ---------------------"<<endl;
        surface.CreateAndUpdateTrackersBasedOnPoissonDisk();
        surface.CreateGridsBasedOnMesh(newPatchesPoints);
        cout << "------------------- Delete Dead Patches ---------------------"<<endl;
        surface.DeleteUnusedPatches();
    }
    if (!usingOnlyPoissonDisk)
    {
        //For the blending computation, we create uv array per vertex that we called patch
        cout << "------------------- Patch Creation ---------------------"<<endl;
        surface.AddDeformablePatchesUsingBarycentricCoordinates();
    }

    //-------------------- texture synthesis to test concealed patches --------------------
    AtlasTestingConcealed atlas;
    if (params.outputName == "")
        params.outputName = params.trackersFilename+".png";
    atlas.SetFilename(params.outputName+".png");
    atlas.SetSurface(surfaceGdp);
    atlas.SetLowResDeformableGrids(surfaceLowResGdp);
    atlas.SetLowResSurface(surfaceLowResGdp);
    atlas.SetDeformableGrids(deformableGridGdp);
    atlas.SetTrackers(trackersGdp);
    atlas.SetTextureExemplar1(params.textureExemplar1Name);
    atlas.SetTextureExemplar1Mask(params.textureExemplar1MaskName);
    atlas.SetNumberOfTextureSampleFrame(params.NumberOfTextureSampleFrame);


    GA_RWHandleI attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleF attLife(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleI attActive(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"active",1));
    map<int,UT_Vector3> trackerPositions;
    {
        GA_Offset ppt;
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {
            int id = attId.get(ppt);
            trackerPositions[id] = trackersGdp->getPos3(ppt);
        }
    }
    atlas.SetTrackersPosition(trackerPositions);

    if(params.useDeformableGrids)
    {
        atlas.UseDeformableGrids();
        cout << "[AtlasAnimatedTextureInterface::Synthesis] "<< "Compute pixel using deformable grids parametric coordinates."<<endl;
    }
    else
    {
        cout << "[AtlasAnimatedTextureInterface::Synthesis] "<< "Compute pixel using overlapping uv and alpha."<<endl;
    }
    //atlas.SetPatchedSurface(SetPatchedSurface);
    bool atlasBuilded = atlas.BuildAtlas(params.atlasWidth,params.atlasHeight, params.fadingTau);
    if(!atlasBuilded)
    {
        cout << "[AtlasAnimatedTextureInterface::Synthesis] "<< "Can't build the rasterizer"<<endl;
        return;
    }
    GA_Primitive *prim;

    long nbOfPrimitive = surfaceGdp->getNumPrimitives();
    cout << "[AtlasAnimatedTextureInterface::Synthesis] with tbb "<< "Rasterizing an "<<params.atlasHeight << " x "<<params.atlasWidth<<" image."<<endl;

    int *nullSurface;
    //TestingConcealed_executor exec(atlas,params.atlasWidth,params.atlasHeight,params);
    //tbb::parallel_for(tbb::blocked_range<size_t>(0,nbOfPrimitive),exec);

    //--------------------------------
    long i = 0;
    int lastModulo = 0;

    GA_FOR_ALL_PRIMITIVES(surfaceGdp,prim)
    {
        GA_Offset primOffset = prim->getMapOffset();
        atlas.RasterizePrimitive(surface, primOffset, params.atlasWidth,params.atlasHeight,params);
        i++;

//        float pourcentage = ((float)i/(float)nbOfPrimitive)*100.0f;
//        int p = pourcentage;
//        int modulo = (p % 100);
//        if (modulo != lastModulo)
//        {
//            lastModulo = modulo;
//            cout << "done "<<modulo<<"%"<<endl;
//        }
    }
    //--------------------------------

    map<int, bool> usedPatches = atlas.getUsedPatches();
    map<int, bool>::iterator itUsedPatches;
    int concealedPatches = 0;
    {
        GA_Offset ppt;
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {
            int id = attId.get(ppt);
            int active = attActive.get(ppt);
            if (!usedPatches[id] && active == 1)
            {
                //if (id == 1388)
                //    cout << "Patch is not used !!!"<<endl;
                attLife.set(ppt,0);
                attActive.set(ppt,0);
                concealedPatches++;
            }
        }
    }
    cout <<surface.approachName<< " We have "<< concealedPatches << " flag as concealed patches."<<endl;
    //atlas.~AtlasTestingConcealed();
    atlas.CleanRayMemory(deformableGridGdp);

    //After ading particle and grids where rasterization didn't work, we need to add patches
    //surface.AddDeformablePatchesUsingBarycentricCoordinates(gdp, surfaceGdp,trackersGdp, params,surfaceTree,ray);
    //surface.DeleteUnusedPatches(gdp, trackersGdp,params);

    //-------------------------------------------------------------------------------------

    // Compute concealed patches

    //=======================================================================

    cout << surface.approachName<<" Done"<<endl;
    //cout << "Clear surface tree"<<endl;
//    surfaceTree.clear();

    cout << surface.approachName<< " saving grids data"<<endl;
    const char* filenameGrids = params.deformableGridsFilename.c_str();//"dlttest.bgeo";
    deformableGridGdp->save(filenameGrids,options,errors);

    cout << surface.approachName<< " saving trackers data"<<endl;
    const char* filenameTrackers = params.trackersFilename.c_str();//"dlttest.bgeo";
    trackersGdp->save(filenameTrackers,options,errors);

    //================================================================
    std::clock_t cleaningStart;
    cleaningStart = std::clock();
    //cout<< "Clear, Destroy and merge"<<endl;
    deformableGridGdp->clearAndDestroy();
    deformableGridGdp->copy(*surfaceGdp);

    // ======================================== REPORT ========================================

    int nbPatches = surface.GetNumberOfPatches();

    float cleaningSurface = (std::clock() - cleaningStart) / (double) CLOCKS_PER_SEC;

    surface.numberOfPatches -= concealedPatches;

    int sumOfPatches = surface.numberOfInitialPatches;

    sumOfPatches += surface.numberOfNewPatches;
    sumOfPatches -= surface.numberOfInitialPatchFlagToDelete;
    sumOfPatches -= surface.numberOfDetachedPatches;
    sumOfPatches -= surface.numberOfNewAndLonelyTracker;
    sumOfPatches -= surface.numberOfLonelyTracker;
    sumOfPatches -= concealedPatches;

    cout << "---------------------------------- Patches  ----------------------------------------------"<<endl;
    cout << surface.approachName<<" Initial number of patches   \t"<<surface.numberOfInitialPatches<<endl;
    cout << surface.approachName<<" Initial flaged to delete    \t"<<surface.numberOfInitialPatchFlagToDelete<<endl;
    cout << surface.approachName<<" New And Lonely              \t"<<surface.numberOfNewAndLonelyTracker<<endl;
    cout << surface.approachName<<" New patches                 \t"<<surface.numberOfNewPatches<<endl;
    cout << surface.approachName<<" Detached patches            \t"<<surface.numberOfDetachedPatches<<endl;
    cout << surface.approachName<<" Patch with no primitives    \t"<<surface.numberOfLonelyTracker<<endl;
    cout << surface.approachName<<" Number of degenerated grid  \t"<<surface.numberOfDegeneratedGrid<<endl;
    cout << surface.approachName<<" Concealed patches           \t"<<concealedPatches<<endl;

    cout << surface.approachName<<" Total Number of patches     \t"<<surface.numberOfPatches<<endl;
    cout << " equals to "<<sumOfPatches<<endl;

    cout << "---------------------------------- Computation Time ----------------------------------------------"<<endl;

    cout << surface.approachName<<" Poisson Disk Sampling       "<<surface.poissondisk<<endl;
    cout << surface.approachName<<" Grid mesh on time           "<<surface.gridMeshCreation<<endl;
    cout << surface.approachName<<" Uv flattening time          "<<surface.uvFlatteningTime<<" for "<<surface.nbOfFlattenedPatch<<" patches"<<endl;
    cout << surface.approachName<<" Tracker advection time      "<<surface.markerAdvectionTime<<endl;
    cout << surface.approachName<<" Grid advection time         "<<surface.gridAdvectionTime<<endl;
    cout << surface.approachName<<" Patch creation time         "<<surface.patchCreationTime<<endl;
    cout << surface.approachName<<" Clear and Destroy           "<<cleaningSurface<<endl;
    cout << surface.approachName<<" Update distribution         "<<surface.updatePatchesTime<<endl;

    float total = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    cout << surface.approachName<< " TOTAL: "<<total<<endl;

    std::ofstream outfile;
    outfile.open("core.csv", std::ios_base::app);
    outfile <<surface.poissondisk<<","<< surface.gridMeshCreation << ","<<surface.uvFlatteningTime << ","<<surface.markerAdvectionTime
            <<","<<surface.gridAdvectionTime<<","<<surface.patchCreationTime << ","<<surface.updatePatchesTime<<","<<nbPatches<<endl;

    cout << "--------------------------------------------------------------------------------"<<endl;
}

