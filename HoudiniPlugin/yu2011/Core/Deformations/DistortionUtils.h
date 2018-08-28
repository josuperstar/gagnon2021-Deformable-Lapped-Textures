#ifndef __DISTORTIONUTILS__
#define __DISTORTIONUTILS__
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <GEO/GEO_PointTree.h>
//#include <GA_ElementGroup.h>
#include <vector>
#include "ParametersDistortion.h"

namespace Mokko {


class DistortionUtils
{

public:
    static void SorkineDistortionEstimation(GU_Detail *gdp, GA_Offset primitive, ParametersDistortion params);
    static void MultipleVariableDistortionEstimation(GU_Detail *gdp, GEO_Primitive *primitive, ParametersDistortion params, int &nbPoints, int &nbDistorted,
                                                     GA_RWHandleF    &attVA, GA_RWHandleF &attW, GA_RWHandleF &attAlpha, GA_RWHandleI &attPrimLife, GA_RWHandleF &attInitArea);
	
};
}

#endif
