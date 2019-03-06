#ifndef __Yu2011_h__
#define __Yu2011_h__

#include <Math/Vec3.h>
#include <Strategies/StrategyPatchSurfaceSynthesis/DeformableGrids.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

namespace Mokko {

#define VERBOSE 0


class Yu2011 : public DeformableGrids
{
public:

    Yu2011(GU_Detail *surface);
    ~Yu2011();
    bool SynthesisSurface( GU_Detail *gdp, ParametersDeformablePatches params);

    void PoissonDiskSampling(GU_Detail *gdp, GU_Detail *surfaceGdp, GU_Detail *trackers, GA_PointGroup *markerGroup, ParametersDeformablePatches params);
    void AddPatchesUsingBarycentricCoordinates(GU_Detail *gdp, GU_Detail* surface, GU_Detail *trackersGdp, ParametersDeformablePatches params,  GEO_PointTreeGAOffset &surfaceTree, GU_RayIntersect &ray);
    void DeleteUnusedPatches(GU_Detail *gdp, GU_Detail *trackersGdp, ParametersDeformablePatches params);

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
