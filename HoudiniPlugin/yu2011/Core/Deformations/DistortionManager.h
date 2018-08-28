#ifndef __DISTORTIONMANAGER_h__
#define __DISTORTIONMANAGER_h__
#include <cassert>
#include <cmath>
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
//#include <GA_ElementGroup.h>
#include <Math/Vec3.h>
#include "Image.h"
#include "Set/SpatialGrid.h"
//#include "HoudiniInterfaces/TextureSynthesis.h"
#include "Core/GaussianPyramidMesh.h"

#include "Core/BestMatchFunctions.h"
#include <GU/GU_Flatten.h>
#include "../HoudiniUtils.h"
#include "DistortionUtils.h"
#include "ParametersDistortion.h"

namespace Mokko {



class DistortionComputation
{

public:

    //=========================== BUILD ======================================



    virtual void ComputeDistortion(GU_Detail *trackers, GU_Detail *grid, GA_Offset trackerPpt, GA_PointGroup* pointGrp, GA_PrimitiveGroup *primGroup, ParametersDistortion params) = 0;


    static void ComputeDistortionBasedOnEdges(GU_Detail *gdp);
    static void ComputeDistortionBasedOnAreas(GU_Detail *gdp);

    static float TriangleAir(UT_Vector3 a,UT_Vector3 b,UT_Vector3 c)
    {
        UT_Vector3 x =(c-b);
        UT_Vector3 y =(c-a);
        UT_Vector3 z = cross(x,y);

        return 0.5* z.length();
    }

};
}

#endif
