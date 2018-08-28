#ifndef __DeformablePatchesStrategy_h__
#define __DeformablePatchesStrategy_h__

#include <Math/Vec3.h>
#include <Core/GaussianPyramidMesh.h>
#include <Strategies/StrategyPatchSurfaceSynthesis.h>
#include "Core/KwatraSurfaceTextureSynthesisParams.h"


namespace Mokko {


class DeformablePatches : public StrategyPatchSurfaceSynthesis
{
public:

    bool SynthesisSurface( GU_Detail *gdp, ParametersDeformablePatches params);

    vector<UT_Vector3>  PoissonDiskDistribution(GU_Detail *gdp, ParametersDeformablePatches params);
    void AddPatches(GU_Detail *gdp, vector<UT_Vector3> points, int startNumber, ParametersDeformablePatches params);
    void ProjectUVSForSelectedGroups(GU_Detail *gdp, ParametersDeformablePatches params);
    void UpdatePatches(GU_Detail *gdp, ParametersDeformablePatches params,int currentFrame);
    void ComputeDistortion(GU_Detail *gdp, ParametersDeformablePatches params);
    const string approachName   = "[Deformable Patches]";

private :

    void ProjectUVForPatch(GU_Detail *gdp, GA_PointGroup *patchGroupToAdd, GA_Attribute *uvArray);


};

}

#endif
