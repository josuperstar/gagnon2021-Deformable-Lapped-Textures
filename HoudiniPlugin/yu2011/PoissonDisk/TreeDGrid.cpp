#include "TreeDGrid.h"

using namespace Mokko;

bool TreeDGrid::RespectCriterion(GU_Detail* trackersGdp, GEO_PointTreeGAOffset &tree, UT_Vector3 newPointPosition, UT_Vector3 newPointNormal, float MinDist, float killDistance, float CellSize, int &numberOfClosePoint, float angleNormalThreshold, GA_Offset exclude )
{

    //brute force
    //std::vector< PoissonDisk >::iterator it;
    numberOfClosePoint = 0;

    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attActive(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"active", 1));

    GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;

    tree.findAllCloseIdx(newPointPosition,
                         MinDist*2,
                         close_particles_indices);

    int l = close_particles_indices.entries();
    GA_Offset neighbor;

    bool tooClose = false;

    for(int j=0; j<l;j++ )
    {

        neighbor = close_particles_indices.array()[j];
        if (attActive.get(neighbor) == 0)
            continue;
        if (neighbor == exclude)
            continue;

        UT_Vector3 pos = trackersGdp->getPos3(neighbor);
        UT_Vector3 n = attN.get(neighbor);
        float dotP = dot(n, newPointNormal);
        bool samePlane = dotP > angleNormalThreshold;
        bool notToClose = distance3d( pos, newPointPosition ) > killDistance ;
        bool inNeighboorhood = distance3d( pos, newPointPosition ) < MinDist;

        //It is too close to the current point ?
        if(samePlane && !notToClose)
        {
            tooClose = true;
            //return false;
        }

        if(inNeighboorhood && samePlane)
            numberOfClosePoint++;
    }

    return !tooClose;
}




