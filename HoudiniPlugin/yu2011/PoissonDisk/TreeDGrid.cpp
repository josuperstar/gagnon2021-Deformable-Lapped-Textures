#include "TreeDGrid.h"

using namespace Mokko;

bool TreeDGrid::RespectCriterion( PoissonDisk Point, float MinDist, float CellSize, std::vector<PoissonDisk> &neighbors, float angleNormalThreshold )
{

    //brute force
    std::vector< PoissonDisk >::iterator it;
    for(it = gridPoints.begin(); it != gridPoints.end(); it++)
    {
        PoissonDisk P = *it;

        //is it on the same plane ?
        float dotP = dot(P.GetNormal(), Point.GetNormal());
        bool samePlane = dotP > angleNormalThreshold;
        bool notToClose = distance3d( P.GetPosition(), Point.GetPosition() ) > MinDist ;

        //It is too close to the current point ?
        if(samePlane && !notToClose)
        {
            return false;
        }
        else // then it is on the same plane, but not too close
        {
            neighbors.push_back(P);
        }
    }
    return true;
}




