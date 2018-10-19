#ifndef __BlendingYu2011_h__
#define __BlendingYu2011_h__

#include <string>
#include <vector>
#include "Blending.h"
#include "Color.h"

namespace Mokko {

class BlendingYu2011 : public Blending
{

public:

    static Pixel Blend(GU_Detail* deformableGrids, int i, int j, float w, float h,
                int pixelPositionX, int pixelPositionY,
                vector<int> &sortedPatches,
                vector<UT_Vector3> &surfaceUv,
                vector<UT_Vector3> &surfacePosition,
                map<int,UT_Vector3> &trackersPosition,
                map<int,UT_Vector3> &trackersUVPosition,
                map<string,GU_RayIntersect*> &rays,
                map<int,Pixel> &patchColors,
                Pixel RM,           //Mean Value
                GA_RWHandleV3 &attPointUV,
                map<int,float> &patchBlend,
                ImageCV *textureExemplar1Image,
                ImageCV *displacementMapImage,
                bool computeDisplacement,
                bool renderColoredPatches,
                Pixel &R1,
                Pixel &displacementSum,
                ParametersDeformablePatches params);


};
}
#endif
