#include "TreeDGrid.h"

using namespace Mokko;

bool TreeDGrid::IsInNeighbourhood( PoissonDisk Point, float MinDist, float CellSize, std::vector<PoissonDisk> &neighbors )
{

    /*

    UT_Vector3 G =  UT_Vector3(( int )( Point.Data().x() / CellSize ), ( int )( Point.Data().y() / CellSize ),( int )( Point.Data().z() / CellSize )); //ImageToGrid( Point, CellSize );

    // number of adjucent cells to look for neighbour points
    const int D = 5;

    // scan the neighbourhood of the point in the grid
    for ( int i = G.x() - D; i < G.x() + D; i++ )
    {
        for ( int j = G.y() - D; j < G.y() + D; j++ )
        {
            for ( int k = G.z() - D; k < G.z() + D; k++ )
            {
                if ( i >= 0 && i < m_W && j >= 0 && j < m_H && k >= 0 && k < m_D)
                {
                    PoissonDisk P = m_Grid[ i ][ j ][ k ];

                    if ( P.IsValid() && distance3d( P.Data(), Point.Data() ) < MinDist ) { return true; }
                }
            }
        }
    }
    */

    //brute force
    std::vector< PoissonDisk >::iterator it;
    for(it = gridPoints.begin(); it != gridPoints.end(); it++)
    {
        PoissonDisk P = *it;

        //is it on the same plane ?
        float dotP = dot(P.GetNormal(), Point.GetNormal());
        bool samePlane = dotP > 0.5f;
        bool tooClose = (distance3d( P.GetPosition(), Point.GetPosition() ) < MinDist );
        if (samePlane && tooClose)
            return true;
        //neighbors.push_back(P);
    }
    return false;
}




