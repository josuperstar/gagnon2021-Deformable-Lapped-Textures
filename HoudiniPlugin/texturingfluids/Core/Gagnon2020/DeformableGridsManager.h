#ifndef __DeformableGridGagnon2020_h__
#define __DeformableGridGagnon2020_h__

#include <Math/Vec3.h>
#include <Core/Gagnon2020/ParticleTrackerManager.h>
#include "Core/Deformations/DistortionMetricSorkine2002.h"
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

namespace TexturingFluids {

#define VERBOSE 0


class DeformableGridsManager : public ParticleTrackerManager
{
public:

    DeformableGridsManager(GU_Detail *surfaceGdp, GU_Detail *surfaceLowResGdp, GU_Detail *trackersGdp, GU_Detail *deformableGridsGdp, ParametersDeformablePatches params);
    ~DeformableGridsManager();

    void CreateGridsBasedOnMesh(vector<GA_Offset> trackers);
    void CreateGridBasedOnMesh(GA_Offset ppt);
    void AdvectGrids();

    const string gridGroupName = "grids";

    double  uvFlatteningTime;
    int     nbOfFlattenedPatch;
    double  gridMeshCreation;
    double  gridAdvectionTime;
    const string approachName   = "[Deformable Grids Gagnon 2020]";

    int numberOfDegeneratedGrid;


protected :

    UT_Vector2 RotateUV(UT_Vector2 uv, float rotation, float mid);

    void FlagBoundaries();
    void FlagBoundariesForPatch(GA_Offset ppt);
    bool UVFlattening(GU_Detail &tempGdp,
                      GA_Offset tracker, GA_Offset closestPoint,
                      GA_PointGroup *pointGroup, GA_PointGroup *tempPointGroup,
                      set<GA_Offset> &pointsAround,
                      float scaling);

    ParametersDistortion distortionParams;
    GU_Detail *deformableGridsGdp;
    GU_Detail *surfaceLowResGdp;
    GEO_PointTreeGAOffset surfaceTree;
    GEO_PointTreeGAOffset surfaceLowResTree;

    const string uvArrayName = "uvs";
    const string alphaArrayName = "alphas";
    const string patchIdsName = "patchIds";

    const char*    alpha0Name      = "alpha0";
    const char*    distortionWeightName  = "w";
    const char*    vertexWeightName  = "vw";
    const char*    distortionWeight0Name  = "w0";
    const char*    disrortionMinThreshold = "t_min";
    const char*    distortionMaxThreshold = "t_max";
    const char*    initialVertexAngle = "v_a0";
    const char*    uvName = "uvw";

    const string triangleArrayName = "triangleRef";
    GA_Attribute        *triangleRefAtt;
    const GA_AIFNumericArray *triangleArray;

    vector<set<map<GA_Offset, UT_Vector3> > > uvs;
    map<int,UT_Vector3> gridCenterPosition;
    bool useUvFlattening = true;

    GU_RayIntersect ray;



};

}

#endif
