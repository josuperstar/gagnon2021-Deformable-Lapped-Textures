#include "Bridson2007PoissonDiskDistribution.h"
#include "HoudiniUtils.h"

#include <GU/GU_RandomPoint.h>


using namespace Mokko;
using namespace std;


std::vector<PoissonDisk> Bridson2007PoissonDiskDistribution::PoissonDiskSampling(GU_Detail *gdp, float diskRadius)
{
    cout << "Bridson2007PoissonDiskDistribution"<<endl;
    k = 30;
    r = diskRadius;

    initializeGrid();
    SelectInitialSample();
    EmptyActiveList();


    //converting sample point in UT_Vector3 list

    for(int i=0;i<SamplePoints.size();i++)
        allpoints.push_back(SamplePoints[i]);
    return allpoints;

}


//Step 0

/*
Initialize an n-dimensional background grid for storing
samples and accelerating spatial searches. We pick the cell size to
be bounded by r/√n
n, so that each grid cell will contain at most
one sample, and thus the grid can be implemented as a simple ndimensional
array of integers: the default −1 indicates no sample, a
non-negative integer gives the index of the sample located in a cell.
*/

void Bridson2007PoissonDiskDistribution::initializeGrid()
{
    cout << "Step 0: initialize the grid"<<endl;

    cellSize = r/(sqrt(n));

    //create the grid

    int gridSizeX = (int)ceil(1.0f/cellSize);
    int gridSizeY = (int)ceil(1.0f/cellSize);

    backgrounGrid = sGrid(gridSizeX,gridSizeY,cellSize);

}

//Step 1
/*
Select the initial sample, x0, randomly chosen uniformly
from the domain. Insert it into the background grid, and initialize
the “active list” (an array of sample indices) with this index (zero).
*/
void Bridson2007PoissonDiskDistribution::SelectInitialSample()
{
    cout << "Step 1: Select Initial Sample"<<endl;




    PoissonDisk FirstPoint;
        do {
            FirstPoint = PoissonDisk( Generator.RandomFloat(), Generator.RandomFloat(),0 );
        } while (!(Circle ? FirstPoint.IsInCircle() : FirstPoint.IsInRectangle()));

        // update containers
        ProcessList.push_back( FirstPoint );
        SamplePoints.push_back( FirstPoint );
        backgrounGrid.Insert( FirstPoint );
    cout << "first point: "<<FirstPoint.GetPosition().x() << " "<<FirstPoint.GetPosition().y()<<endl;
}

//Step 2
/*
While the active list is not empty, choose a random index
from it (say i). Generate up to k points chosen uniformly from the
spherical annulus between radius r and 2r around xi. For each
point in turn, check if it is within distance r of existing samples
(using the background grid to only test nearby samples). If a point
is adequately far from existing samples, emit it as the next sample
and add it to the active list. If after k attempts no such point is
found, instead remove i from the active list.
*/

void Bridson2007PoissonDiskDistribution::EmptyActiveList()
{
    cout << "Step 2: Empty Active List"<<endl;

    //int numPoints = 10;

    // generate new points for each point in the queue
        while ( !ProcessList.empty() && SamplePoints.size() < numberOfPoints )
        {
            PoissonDisk Point = PopRandom<DefaultPRNG>( ProcessList, Generator );

            for ( int i = 0; i < k; i++ )
            {
                PoissonDisk NewPoint = GenerateRandomPointAround( Point, r, Generator );

                bool Fits = Circle ? NewPoint.IsInCircle() : NewPoint.IsInRectangle();

                if ( Fits && !backgrounGrid.IsInNeighbourhood( NewPoint, r, cellSize ) )
                {
                    ProcessList.push_back( NewPoint );
                    SamplePoints.push_back( NewPoint );
                    backgrounGrid.Insert( NewPoint );
                    continue;
                }
            }
        }


}
