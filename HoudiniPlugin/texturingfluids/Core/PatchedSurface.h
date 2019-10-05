#ifndef __PatchedSurface__
#define __PatchedSurface__

#include <Math/Vec3.h>
#include <Core/DeformableGridsManager.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

namespace TexturingFluids {

#define VERBOSE 0


class PatchedSurface : public DeformableGridsManager
{
public:

    PatchedSurface(GU_Detail *surface, GU_Detail *trackersGdp);
    ~PatchedSurface();

    void PoissonDiskSampling(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params);
    void AddDeformablePatchesUsingBarycentricCoordinates(GU_Detail *gdp, GU_Detail* surface, GU_Detail *trackersGdp, ParametersDeformablePatches params,  GEO_PointTreeGAOffset &surfaceTree, GU_RayIntersect &ray);
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