#include "AtlasDeformablePatchesInterface.h"

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

#include <Core/Gagnon2020/AtlasDeformablePatches.h>
#include <Core/Gagnon2020/TBBAtlasAnimatedTexture.h>
#include <tbb/parallel_for.h>


AtlasDeformablePatchesInterface::AtlasDeformablePatchesInterface()
{
}

AtlasDeformablePatchesInterface::~AtlasDeformablePatchesInterface()
{
}

bool AtlasDeformablePatchesInterface::Synthesis(GU_Detail *gdp,  GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{
    cout << "[AtlasDeformablePatchesInterface::Synthesis] "<<params.frame<<endl;

    std::clock_t start;
    start = std::clock();

    AtlasDeformablePatches atlas;
    if (params.outputName == "")
        params.outputName = params.trackersFilename+".png";
    atlas.SetFilename(params.outputName+".png");
    atlas.SetSurface(surfaceGdp);
    atlas.SetDeformableGrids(gdp);
    atlas.SetTrackers(trackersGdp);
    atlas.SetTextureExemplar1(params.textureExemplar1Name);
    atlas.SetTextureExemplar1Mask(params.textureExemplar1MaskName);
    atlas.SetDisplacementMap1(params.displacementMap1Name);
    atlas.RenderColoredPatches(params.coloredPatches);
    atlas.SetNumberOfTextureSampleFrame(params.NumberOfTextureSampleFrame);


    GA_RWHandleI attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
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
        cout << "[AtlasDeformablePatchesInterface::Synthesis] "<< "Compute pixel using deformable grids parametric coordinates."<<endl;
    }
    else
    {
        cout << "[AtlasDeformablePatchesInterface::Synthesis] "<< "Compute pixel using overlapping uv and alpha."<<endl;
    }
    bool atlasBuilded = atlas.BuildAtlas(params.atlasWidth,params.atlasHeight, params.fadingTau);
    if(!atlasBuilded)
    {
        cout << "[AtlasDeformablePatchesInterface::Synthesis] "<< "Can't build the rasterizer"<<endl;
        return false;
    }
    GA_Primitive *prim;

    bool usingTbb = true;
    long nbOfPrimitive = surfaceGdp->getNumPrimitives();

    if(!usingTbb)
    {
        cout << "[AtlasDeformablePatchesInterface::Synthesis] without tbb"<< "Rasterizing an "<<params.atlasHeight << " x "<<params.atlasWidth<<" image."<<endl;

        long i = 0;
        int lastModulo = 0;

        GA_FOR_ALL_PRIMITIVES(surfaceGdp,prim)
        {
            GA_Offset primOffset = prim->getMapOffset();
            atlas.RasterizePrimitive(primOffset, params.atlasWidth,params.atlasHeight,params);
            i++;

            float pourcentage = ((float)i/(float)nbOfPrimitive)*100.0f;
            int p = pourcentage;
            int modulo = (p % 100);
            if (modulo != lastModulo)
            {
                lastModulo = modulo;
                cout << "done "<<modulo<<"%"<<endl;
            }
        }
    }
    else
    {
        cout << "[AtlasDeformablePatchesInterface::Synthesis] with tbb "<< "Rasterizing an "<<params.atlasHeight << " x "<<params.atlasWidth<<" image."<<endl;
        DeformablePatches_executor exec(atlas,params.atlasWidth,params.atlasHeight,params);
        tbb::parallel_for(tbb::blocked_range<size_t>(0,nbOfPrimitive),exec);
    }
    atlas.SaveAtlas();

    float total = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    cout <<" TOTAL: "<<total<<endl;
    cout << "returing true"<<endl;
    return true;

}

