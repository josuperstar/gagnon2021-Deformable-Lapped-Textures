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

    void AddDeformablePatcheUsingBarycentricCoordinates(GA_Offset ppt);
    void AddDeformablePatchesUsingBarycentricCoordinates();
    void DeleteUnusedPatches();

    set<int> GetSortedPatches(GA_Offset primOffset);

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
    map<GA_Offset, set<int> > patchesPerPrimitives;

    GA_RWHandleV3 attUV;

};

}

#endif
