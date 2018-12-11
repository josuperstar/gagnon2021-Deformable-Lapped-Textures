#include "TreeDGrid.h"


using namespace Mokko;

bool TreeDGrid::RespectCriterion(GU_Detail* trackersGdp, GEO_PointTreeGAOffset &tree, UT_Vector3 newPointPosition, UT_Vector3 newPointNormal, float killDistance, int &numberOfClosePoint,   GA_Offset exclude, ParametersDeformablePatches params )
{
    numberOfClosePoint = 0;

    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attActive(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"active", 1));

    GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;

    tree.findAllCloseIdx(newPointPosition,
                         params.poissondiskradius*2,
                         close_particles_indices);

    int l = close_particles_indices.entries();
    GA_Offset neighbor;
    bool tooClose = false;

    float cs    = params.CellSize;
    float r     = params.poissondiskradius;

    newPointNormal.normalize();
    float kd = killDistance;

    for(int j=0; j<l;j++)
    {
        neighbor = close_particles_indices.array()[j];
        if (attActive.get(neighbor) == 0)
            continue;
        if (neighbor == exclude)
            continue;

        UT_Vector3 pos          = trackersGdp->getPos3(neighbor);
        UT_Vector3 pNp          = pos - newPointPosition;
        pNp.normalize();
        UT_Vector3 n            = attN.get(neighbor);
        n.normalize();
        float dotP              = dot(pNp, newPointNormal);
        float dotN              = dot(n,newPointNormal);
        bool samePlane          = dotN > params.angleNormalThreshold;
        float d              = distance3d( pos, newPointPosition );
        float dp                = abs(dotP);

        float k        = (1-dp)*r;
        if (k < cs)
            k = cs;

        float k2   = (1-dp)*kd;
        if (k2 < cs)
            k2 = cs;

        //hack to test old approch
        //k = r;
        //k2 = killDistance;

        bool outsideOfSmallEllipse         = d > k2;
        bool insideBigEllipse    = d < k;

        //It is too close to the current point ?
        if(samePlane && !outsideOfSmallEllipse)
        {
            tooClose = true;
        }

        if(insideBigEllipse && samePlane)
            numberOfClosePoint++;
    }
    return !tooClose;
}
