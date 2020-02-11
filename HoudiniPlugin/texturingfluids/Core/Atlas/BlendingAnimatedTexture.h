#ifndef __BlendingAnimatedTexture_h__
#define __BlendingAnimatedTexture_h__

#include <string>
#include <vector>
#include "Blending.h"


namespace TexturingFluids {

class BlendingAnimatedTexture : public Blending
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
                       map<int, bool> usePatches,
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
}

#endif
