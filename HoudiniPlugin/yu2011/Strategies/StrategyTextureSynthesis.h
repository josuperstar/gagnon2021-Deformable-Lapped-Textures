#ifndef __STRATEGYSURFACE_h__
#define __STRATEGYSURFACE_h__

#include <Math/Vec3.h>
#include <Core/GaussianPyramidMesh.h>
#include <GaussianPyramid.h>

#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>

#include <Core/WeyAndLevoyStructure.h>
#include <Core/BestMatchFunctions.h>
#include <Core/BuildingNeighborhoodFunctions.h>

using namespace std;
using namespace Mokko;

namespace Mokko {


class StrategyTextureSynthesis
{
public:
    virtual bool SynthesisTexture( GaussianPyramid &Ga, GaussianPyramid &Go,int numberOfLevel) = 0;

protected :

    WeyAndLevoyStructure core;
    BuildingNeighborhood builder;
    BestMatchFunctions matcher;

};

}

#endif
