#ifndef __Yu2011_h__
#define __Yu2011_h__

#include <Math/Vec3.h>
#include <Strategies/StrategyPatchSurfaceSynthesis/DeformableGridsManager.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

namespace TexturingFluids {

#define VERBOSE 0


class Yu2011 : public DeformableGridsManager
{
public:

    Yu2011(GU_Detail *surface, GU_Detail *trackersGdp);
    ~Yu2011();

    void PoissonDiskSampling(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params);
    void AddPatchesUsingBarycentricCoordinates(GU_Detail *gdp, GU_Detail* surface, GU_Detail *trackersGdp, ParametersDeformablePatches params,  GEO_PointTreeGAOffset &surfaceTree, GU_RayIntersect &ray);
    void DeleteUnusedPatches(GU_Detail *gdp, GU_Detail *trackersGdp, ParametersDeformablePatches params);

    //for test purpose
    void CreateAPatch(GU_Detail *trackers, ParametersDeformablePatches params);

    double poissondisk;
    double  patchCreationTime;
    double  updatePatchesTime;
    const string approachName   = "[Yu 2011 extention]";

private :

    //-------------- VERTEX ARRAY -----------------
    GA_Attribute        *uvsAtt;
    const GA_AIFNumericArray *uvsArray;

    GA_Attribute        *patchIdsArrayAttrib;
    const GA_AIFNumericArray *patchIdsAtt;

    GA_Attribute        *alphaArrayAtt;
    const GA_AIFNumericArray *alphaAtt;
    //---------------------------------------------

    map<string,GU_RayIntersect*> rays;

};

}

#endif
