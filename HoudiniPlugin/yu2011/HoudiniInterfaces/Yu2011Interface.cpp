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

#include <Strategies/StrategySurfaceTextureSynthesis.h>
#include <Strategies/StrategyPatchSurfaceSynthesis/Yu2011.h>

#include <Core/Atlas/HoudiniAtlas.h>
#include <Core/Atlas/TBBAtlas.h>
#include <PoissonDisk/PoissonDiskDistribution.h>
#include <PoissonDisk/Bridson2012PoissonDiskDistribution.h>

Yu2011Interface::Yu2011Interface()
{

}

Yu2011Interface::~Yu2011Interface()
{

}

void Yu2011Interface::Synthesis(GU_Detail *gdp, GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GU_Detail *levelSet,  ParametersDeformablePatches params)
{
    Yu2011 strategy(gdp,surfaceGdp,trackersGdp);

    cout << "[Yu2011Interface::Synthesis] "<<params.frame<<endl;

    params.useDynamicTau = false;

    std::clock_t start;
    start = std::clock();

    vector<GA_Offset> newPatchesPoints;
    vector<GA_Offset> trackers;
    cout << "reference gdp created"<<endl;


    GA_PointGroup *surfaceGroup = (GA_PointGroup *)surfaceGdp->pointGroups().find(strategy.surfaceGroupName.c_str());
    if (surfaceGroup == 0x0)
    {
        cout << "There is no surface group to synthesis"<<endl;
        return;
    }

    //=======================================================

    bool useDeformableGrids = params.useDeformableGrids;

    GA_PointGroup *grp = (GA_PointGroup *)gdp->pointGroups().find(strategy.markerGroupName.c_str());

    GU_RayIntersect ray(gdp);
    ray.init();
    GEO_PointTreeGAOffset surfaceTree;
    surfaceTree.build(surfaceGdp, NULL);

    //=========================== CORE ALGORITHM ============================

    //section 3.3.1 Particle Distribution
    vector<PoissonDisk> PPoints = strategy.PoissonDiskSampling(gdp,levelSet,trackersGdp,grp,params);
    trackers = strategy.CreateAndUpdateTrackersBasedOnPoissonDisk(surfaceGdp,trackersGdp, surfaceGroup,params,PPoints);

    //section 3.3.2 Particle Delition
    newPatchesPoints = strategy.AdvectMarkers(surfaceGdp,trackersGdp, params,surfaceTree);
    if(params.startFrame == params.frame)
    {
        strategy.CreateGridBasedOnMesh(gdp,surfaceGdp,trackersGdp, params,newPatchesPoints,surfaceTree);
    }
    else
    {
        //section 3.3.2 Grid Advection
        strategy.AdvectGrids(gdp,trackersGdp,params,surfaceTree,newPatchesPoints,surfaceGdp);

        //section 3.3.3 Estimating the Grid Distortion
        strategy.UpdateDistributionUsingBridson2012PoissonDisk(gdp,surfaceGdp, trackersGdp,params,surfaceTree,ray);

    }

    //3.4 Blending and Continuity
    //For the blending computation, we create uv array per vertex that we called patch
    strategy.AddPatchesUsingBarycentricCoordinates(gdp, surfaceGdp,trackersGdp, params,newPatchesPoints,surfaceTree,ray);


    //----------------------------------------------------------------------------------------------------
    //section 3.3.1 Particle Distribution
    //PPoints = strategy.PoissonDiskSampling(gdp,levelSet,trackersGdp,grp,params);
    //strategy.CreateAndUpdateTrackersBasedOnPoissonDisk(surfaceGdp,trackersGdp, surfaceGroup,params,PPoints);
    //----------------------------------------------------------------------------------------------------

    //=======================================================================




    cout << strategy.approachName<<" Done"<<endl;

    cout << "Clear surface tree"<<endl;
    surfaceTree.clear();
    ray.clear();

    if(params.computeAtlas == 0)
        cout << "No atlas to compute" <<endl;
    if (params.computeAtlas == 1)
    {
        //=========================== ATLAS ==============================

        // Section 3.4 Blending and Continuity
        HoudiniAtlas atlas;
        atlas.SetFilename(params.trackersFilename+".png");
        atlas.SetSurface(surfaceGdp);
        if(useDeformableGrids)
            atlas.SetDeformableGrids(gdp);
        atlas.SetTrackers(trackersGdp);
        atlas.SetTextureExemplar1(params.textureExemplar1Name);
        atlas.SetTextureExemplar1Mask(params.textureExemplar1MaskName);
        atlas.SetDisplacementMap1(params.displacementMap1Name);
        if (params.useDeformableGrids)
            atlas.UseDeformableGrids();
        bool atlasBuilded = atlas.BuildAtlas(params.atlasWidth,params.atlasHeight,params.deletionLife);
        if(!atlasBuilded)
        {
            cout << "cant build atlas" <<endl;
            return;
        }
        GA_Primitive *prim;

        bool usingTbb = true;
        cout << "Rasterizing ..."<<endl;
        if(!usingTbb)
        {
            GA_FOR_ALL_PRIMITIVES(surfaceGdp,prim)
            {
                GA_Offset primOffset = prim->getMapOffset();
                atlas.RasterizePrimitive(primOffset, params.atlasWidth,params.atlasHeight, params);
            }
        }
        else
        {
            int nbPrims = surfaceGdp->getNumPrimitives();

            executor exec(atlas,params.atlasWidth,params.atlasHeight, params);
            tbb::parallel_for(tbb::blocked_range<size_t>(0,nbPrims),exec);

        }
        atlas.SaveAtlas();

        ImageCV dilated;
        dilated.OpenImage(params.trackersFilename+".png",-1);
        ImageCV::growRegions(dilated.image,dilated.image,3);
        dilated.SaveImage();
    }

    bool debug = false;
    if(!debug)
    {
        cout << strategy.approachName<< " saving grids data"<<endl;

        const GA_SaveOptions *options;

        UT_StringArray *errors;

        const char* filename = params.deformableGridsFilename.c_str();//"dlttest.bgeo";
        try
        {
            gdp->save(filename,options,errors);
        }
        catch(int e)
        {
            cout << "An exception occurred. Exception Nr. " << e << '\n';
        }

        cout << strategy.approachName<< " saving trackers data"<<endl;
        filename = params.trackersFilename.c_str();//"dlttest.bgeo";
        try
        {
            trackersGdp->save(filename,options,errors);
        }
        catch(int e)
        {
            cout << "An exception occurred. Exception Nr. " << e << '\n';
        }
    }

    //================================================================

    std::clock_t cleaningStart;
    cleaningStart = std::clock();
    cout<< "Clear, Destroy and merge"<<endl;
    gdp->clearAndDestroy();
    //gdp->merge(surfaceGdp);

    gdp->copy(*surfaceGdp);

    float cleaningSurface = (std::clock() - cleaningStart) / (double) CLOCKS_PER_SEC;


    cout << "--------------------------------------------------------------------------------"<<endl;
    cout << strategy.approachName<<" Poisson Disk Sampling "<<strategy.poissondisk<<endl;
    cout << strategy.approachName<<" Grid mesh creation time "<<strategy.gridMeshCreation<<endl;
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

