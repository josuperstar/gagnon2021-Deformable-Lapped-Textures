#ifndef __BlendingGagnon2016_h__
#define __BlendingGagnon2016_h__

#include <string>
#include <vector>


#include "Blending.h"


namespace Mokko {


class BlendingGagnon2016 : public Blending
{

public:

    static Pixel Blend(GU_Detail* trackersGdp,GU_Detail* deformableGrids, int i, int j, float w, float h,
                int pixelPositionX, int pixelPositionY,
                vector<int> &sortedPatches,
                vector<UT_Vector3> &surfaceUv,
                vector<UT_Vector3> &surfacePosition,
                bool useDeformableGrids,
                map<int,UT_Vector3> &trackersUVPosition,
                map<string,GU_RayIntersect*> &rays,
                map<int,Pixel> &patchColors,
                GA_ROHandleF &attAlpha,
                GA_RWHandleV3 &attPointUV,
                map<int,float> &patchBlend,
                ImageCV *textureExemplar1Image,
                vector<ImageCV*> textureExemplarImageMaskVector,
                ImageCV *displacementMapImage,
                bool computeDisplacement,
                bool renderColoredPatches,
                Pixel &R1,
                Pixel &displacementSum,
                ParametersDeformablePatches params);


};
}

#endif
