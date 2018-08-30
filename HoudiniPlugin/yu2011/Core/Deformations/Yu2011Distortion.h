#ifndef __YU2011DISTORTION_h__
#define __YU2011DISTORTION_h__
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

#include <GU/GU_Flatten.h>
#include "../HoudiniUtils.h"
#include "DistortionManager.h"

namespace Mokko {


class Yu2011Distortion : public DistortionComputation
{

public:

    //=========================== BUILD ======================================
   void ComputeDistortion(GU_Detail *trackers, GU_Detail *gridGdp, GA_Offset trackerPpt, GA_PointGroup* pointGrp, GA_PrimitiveGroup *primGroup, ParametersDistortion params);



};
}

#endif
