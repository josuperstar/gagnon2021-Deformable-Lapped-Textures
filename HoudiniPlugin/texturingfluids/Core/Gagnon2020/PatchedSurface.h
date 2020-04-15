#ifndef __PatchedSurfaceGagnon2020__
#define __PatchedSurfaceGagnon2020__

#include <Math/Vec3.h>
#include <Core/Gagnon2020/DeformableGridsManager.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

namespace TexturingFluids {

#define VERBOSE 0


class PatchedSurfaceGagnon2020 : public DeformableGridsManager
{
public:

    PatchedSurfaceGagnon2020(GU_Detail *surface, GU_Detail *trackersGdp, ParametersDeformablePatches params);
    ~PatchedSurfaceGagnon2020();

    void PoissonDiskSampling(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params);
    void AddDeformablePatcheUsingBarycentricCoordinates(GU_Detail *deformableGridsGdp,GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GA_Offset ppt, ParametersDeformablePatches params, GEO_PointTreeGAOffset &surfaceTree,  GU_RayIntersect &ray);
    void AddDeformablePatchesUsingBarycentricCoordinates(GU_Detail *gdp, GU_Detail* surface, GU_Detail *trackersGdp, ParametersDeformablePatches params,  GEO_PointTreeGAOffset &surfaceTree, GU_RayIntersect &ray);
    void DeleteUnusedPatches(GU_Detail *gdp, GU_Detail *trackersGdp, ParametersDeformablePatches params);

    //for test purpose
    GA_Offset CreateAPatch(GU_Detail *trackers, UT_Vector3 position, UT_Vector3 normal, ParametersDeformablePatches params);

    double poissondisk;
    double  patchCreationTime;
    double  updatePatchesTime;
    const string approachName   = "[Patched Surface Gagnon 2020]";

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
