#ifndef __PixelColorComputation_h__
#define __PixelColorComputation_h__

#include <string>
#include <vector>

#include "Images/Color.h"
#include "Images/ImageCV.h"

#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>
#include "Core/Deformations/ParametersDeformablePatches.h"


namespace TexturingFluids {

class PixelColorComputation
{

public:

    static Pixel Blend(GU_Detail* deformableGrids, int i, int j, float w, float h,
                       int pixelPositionX, int pixelPositionY,
                       vector<int> &sortedPatches,
                       vector<UT_Vector3> &surfaceUv,
                       vector<UT_Vector3> &surfacePosition,
                       map<int,UT_Vector3> &trackersNormal,
                       map<int,UT_Vector3> &trackersPosition,
                       map<int,UT_Vector3> &trackersUVPosition,
                       map<int, bool> &usePatches,
                       map<string,GU_RayIntersect*> &rays,
                       map<int,Pixel> &patchColors,
                       Pixel RM,           //Mean Value
                       GA_RWHandleV3 &attPointUV,
                       map<int,float> &patchBlend,
                       vector<ImageCV*> textureExemplars,
                       ImageCV *displacementMapImage,
                       bool computeDisplacement,
                       bool renderColoredPatches,
                       Pixel &R1,
                       Pixel &displacementSumEq3,
                       Pixel &displacementSumEq4,
                       ParametersDeformablePatches params);


};

static void Clamp(Pixel &pixel)
{

    if (pixel.A > 1)
        pixel.A = 1;

    if (pixel.R > 1)
        pixel.R = 1;
    if (pixel.G > 1)
        pixel.G = 1;
    if (pixel.B > 1)
        pixel.B = 1;

    if (pixel.A < 0)
        pixel.A = 0;

    if (pixel.R < 0)
        pixel.R = 0;
    if (pixel.G < 0)
        pixel.G = 0;
    if (pixel.B < 0)
        pixel.B = 0;

}

}

#endif
