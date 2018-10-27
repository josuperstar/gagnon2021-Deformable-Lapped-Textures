#include "TreeDGrid.h"

using namespace Mokko;

bool TreeDGrid::RespectCriterion( PoissonDisk Point, float MinDist, float CellSize, int &numberOfClosePoint, float angleNormalThreshold )
{

    //brute force
    std::vector< PoissonDisk >::iterator it;
    numberOfClosePoint = 0;
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
            numberOfClosePoint++;
            //return false;
        }
        else // then it is on the same plane, but not too close
        {
            //neighbors.push_back(P);
        }
    }
    //Point.SetDensity(numberOfClosePoint);
    if (numberOfClosePoint > 0)
        return false;
    return true;
}




