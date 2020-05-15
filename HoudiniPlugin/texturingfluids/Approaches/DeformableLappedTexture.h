#ifndef __DeformableLappedTeture_h__
#define __DeformableLappedTeture_h__
#include <cassert>
#include <cmath>
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <Math/Vec3.h>
#include "Images/Image.h"
#include "Set/SpatialGrid.h"
#include <GU/GU_Flatten.h>
#include <Strategies/StrategyPatchSurfaceSynthesis.h>

#include <Core/Gagnon2020/PatchedSurface.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/tbb.h>

namespace TexturingFluids {

class DeformableLappedTexture
{

public:

    //=========================== BUILD ======================================
    DeformableLappedTexture();
    ~DeformableLappedTexture();

    //==========================================================================
    void Synthesis(GU_Detail* gdp, GU_Detail* surface, GU_Detail* trackersGdp, GU_Detail* levelSet, GU_Detail *surfaceLowRes, ParametersDeformablePatches params);

private :
    void UpdateByRasterization(PatchedSurfaceGagnon2020 &surface, GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GU_Detail *surfaceLowResGdp, GU_Detail *deformableGridGdp, ParametersDeformablePatches params);

	
};
}

#endif
