#ifndef __DeformablePatchesStrategy_h__
#define __DeformablePatchesStrategy_h__

#include <Math/Vec3.h>
#include <Core/GaussianPyramidMesh.h>
#include <Strategies/StrategyPatchSurfaceSynthesis/ParticleTracker.h>
#include "Core/KwatraSurfaceTextureSynthesisParams.h"
#include <GEO/GEO_PointTree.h>

namespace Mokko {


class DynamicLappedTexture : public ParticleTracker
{
public:

    DynamicLappedTexture(GU_Detail *surface, GU_Detail *trackers);
    bool SynthesisSurface( GU_Detail *gdp, ParametersDeformablePatches params);

    void AddPatches(GU_Detail *gdp,GU_Detail *surfaceGdp, GU_Detail *trackersGdp, int startNumber, ParametersDeformablePatches params, vector<GA_Offset> trackers,  GEO_PointTreeGAOffset &tree);
    void ProjectUVSForSelectedGroups(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params, vector<GA_Offset> trackers);
    void UpdatePatches(GU_Detail *gdp,GU_Detail *surface, GU_Detail *trackers,GA_PointGroup *surfaceGroup, ParametersDeformablePatches params,int currentFrame,GEO_PointTreeGAOffset &tree);
    void ComputeDistortion(GU_Detail *gdp, ParametersDeformablePatches params);

    GEO_PointTreeGAOffset* CreateSurfaceTree(GU_Detail *gdp, ParametersDeformablePatches params);

    const string markerGroupName = "markers";
    const string approachName   = "[Dynamic Lapped Texture]";

private :

    void ProjectUVOrthogonalForPatch(GU_Detail *surface, GU_Detail *trackers, GA_PointGroup *patchGroupToAdd,const GA_AIFNumericArray *uvsArray, GA_Attribute *uvAtt,vector<GA_Offset> markers, int index, ParametersDeformablePatches params);

    GA_Offset GetNearestPoint(GEO_PointTreeGAOffset &tree, UT_Vector3 &pos, float maxdist);
    void ConnectivityTest(const GU_Detail *gdp,GA_Offset point, GA_PointGroup *grp, GEO_PointTreeGAOffset::IdxArrayType &pointsAround,set<GA_Offset> &group);


    const string uvArrayName = "uvs";
    const string alphaArrayName = "alphas";
    const string patchIdsName = "patchIds";
    int numberOfPatches;
    int maxId = 0;
    vector<set<map<GA_Offset, UT_Vector3> > > uvs;
    set<int> patchesUsed;

    bool tackerPolygon = false;


    //-------------- VERTEX ARRAY -----------------
    GA_Attribute        *uvsAtt;
    const GA_AIFNumericArray *uvsArray;

    GA_Attribute        *patchIdsArrayAttrib;
    const GA_AIFNumericArray *patchIdsAtt;

    GA_Attribute        *alphaArrayAtt;
    const GA_AIFNumericArray *alphaAtt;
    //---------------------------------------------

};

}

#endif
