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

    PatchedSurfaceGagnon2020(GU_Detail *surface, GU_Detail *surfaceLowResGdp, GU_Detail *trackersGdp, GU_Detail *deformableGridsGdp, ParametersDeformablePatches params);
    ~PatchedSurfaceGagnon2020();

    void PoissonDiskSampling(GU_Detail *levelSet);
    void AddDeformablePatcheUsingBarycentricCoordinates(GA_Offset ppt);
    void AddDeformablePatchesUsingBarycentricCoordinates();
    void DeleteUnusedPatches();

    //for test purpose
    GA_Offset CreateAPatch(UT_Vector3 position, UT_Vector3 normal);

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
