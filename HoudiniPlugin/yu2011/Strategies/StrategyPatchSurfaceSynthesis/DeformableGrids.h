#ifndef __DeformableGrid_h__
#define __DeformableGrid_h__

#include <Math/Vec3.h>
#include <Core/GaussianPyramidMesh.h>
#include <Strategies/StrategyPatchSurfaceSynthesis/ParticleTracker.h>
#include "Core/KwatraSurfaceTextureSynthesisParams.h"
#include "Core/Deformations/Yu2011Distortion.h"
#include "Core/Deformations/LocalDeformationDistortion.h"
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

namespace Mokko {

#define VERBOSE 0


class DeformableGrids : public ParticleTracker
{
public:

    DeformableGrids(GU_Detail *gdp,GU_Detail *surface, GU_Detail *trackers);
    bool SynthesisSurface( GU_Detail *gdp, ParametersDeformablePatches params);

    void CreateGridBasedOnMesh(GU_Detail *gdp,GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params,vector<GA_Offset> trackers,  GEO_PointTreeGAOffset &tree);
    void AdvectGrids(GU_Detail *gdp, GU_Detail *trackersGdp, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree, vector<GA_Offset> trackers,  GU_Detail *surfaceGdp);

    void ConnectivityTest(const GU_Detail *gdp,GA_Offset point, GA_PointGroup *grp, set<GA_Offset> &pointsAround,set<GA_Offset> &group);


    const string gridGroupName = "grids";

    double  uvFlatteningTime;
    int     nbOfFlattenedPatch;
    double  gridMeshCreation;
    double  gridAdvectionTime;
    const string approachName   = "[Deformable Grids]";


protected :

    //void ConnectivityTest(const GU_Detail *gdp,GA_Offset point, GA_PointGroup *grp, GEO_PointTreeGAOffset::IdxArrayType pointsAround,set<GA_Offset> &group);


    const string uvArrayName = "uvs";
    const string alphaArrayName = "alphas";
    const string patchIdsName = "patchIds";


    const char*    alpha0Name      = "alpha0";
    //const string    alphaLifeNumberName   = "L";
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


    set<int> patchesUsed;


    bool useUvFlattening = true;



};

}

#endif
