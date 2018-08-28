#ifndef __Bridson2007PoissonDiskDistribution__
#define __Bridson2007PoissonDiskDistribution__
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <GEO/GEO_PointTree.h>
//#include <GA_ElementGroup.h>
#include <vector>
#include "PoissonDisk/PoissonDiskDistribution.h"



using namespace std;
namespace Mokko {


//Based on paper from Bridson "Fast Poisson Disk Sampling in Arbritrary Dimensions"
// http://people.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf

//The code has been insprired by https://github.com/thinks/poisson-disk-sampling/blob/master/include/thinks/poissonDiskSampling.hpp


class DefaultPRNG
{
public:
    DefaultPRNG()
    : m_Gen( std::random_device()() )
    , m_Dis( 0.0f, 1.0f )
    {
        // prepare PRNG
        m_Gen.seed( time( nullptr ) );
    }

    explicit DefaultPRNG( uint32_t seed )
    : m_Gen( seed )
    , m_Dis( 0.0f, 1.0f )
    {
    }

    float RandomFloat()
    {
        return static_cast<float>( m_Dis( m_Gen ) );
    }

    int RandomInt( int Max )
    {
        std::uniform_int_distribution<> DisInt( 0, Max );
        return DisInt( m_Gen );
    }

private:
    std::mt19937 m_Gen;
    std::uniform_real_distribution<float> m_Dis;
};

/*
class PoissonDisk
{
public:
    PoissonDisk(){}
    PoissonDisk(UT_Vector3 d) {data = d;}
    PoissonDisk(float x, float y){data = UT_Vector3(x,y,0);}
    bool IsValid(){return valid;}
    UT_Vector3 Data(){return data;}

    bool IsInRectangle() const
    {
        return data.x() >= 0 && data.y() >= 0 && data.x() <= 1 && data.y() <= 1;
    }
    //
    bool IsInCircle() const
    {
        float fx = data.x() - 0.5f;
        float fy = data.y() - 0.5f;
        return ( fx*fx + fy*fy ) <= 0.25f;
    }

private:
    UT_Vector3 data;
    bool valid;
};
*/

class sGrid
{

public:
    sGrid(){}

    sGrid( int W, int H, float CellSize )
        : m_W( W )
        , m_H( H )
        , m_CellSize( CellSize )
    {
        m_Grid.resize( m_H );

        for ( auto i = m_Grid.begin(); i != m_Grid.end(); i++ ) { i->resize( m_W ); }
    }
    void Insert( PoissonDisk& P )
    {
        float x = P.GetPosition().x() / m_CellSize ;
        float y = P.GetPosition().y() / m_CellSize ;
        UT_Vector3 G = UT_Vector3(x,y,0);
        m_Grid[ G.x() ][ G.y() ] = P;
    }
    bool IsInNeighbourhood( PoissonDisk Point, float MinDist, float CellSize )
    {

        UT_Vector3 G =  UT_Vector3(( int )( Point.GetPosition().x() / CellSize ), ( int )( Point.GetPosition().y() / CellSize ),0); //ImageToGrid( Point, CellSize );

        // number of adjucent cells to look for neighbour points
        const int D = 5;

        // scan the neighbourhood of the point in the grid
        for ( int i = G.x() - D; i < G.x() + D; i++ )
        {
            for ( int j = G.y() - D; j < G.y() + D; j++ )
            {
                if ( i >= 0 && i < m_W && j >= 0 && j < m_H )
                {
                    PoissonDisk P = m_Grid[ i ][ j ];

                    if ( P.IsValid() && distance3d( P.GetPosition(), Point.GetPosition() ) < MinDist )
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }

private:
    int m_W;
    int m_H;
    float m_CellSize;

    std::vector< std::vector<PoissonDisk> > m_Grid;
};



/*
The algorithm takes as input the extent of the sample domain in
Rn, the minimum distance r between samples, and a constant k
as the limit of samples to choose before rejection in the algorithm
(typically k = 30).
*/

class Bridson2007PoissonDiskDistribution : public PoissonDiskDistribution
{

public:

    //Bridson2007PoissonDiskDistribution(GU_Detail *gdp, float diskRadius, int k);

    std::vector<PoissonDisk> PoissonDiskSampling(GU_Detail *gdp, float diskRadius);
    void SetNumberOfPoint(int data){this->numberOfPoints = data;}

private:

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

    void initializeGrid();

    //Step 1
    /*
    Select the initial sample, x0, randomly chosen uniformly
    from the domain. Insert it into the background grid, and initialize
    the “active list” (an array of sample indices) with this index (zero).
    */
    void SelectInitialSample();

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

    void EmptyActiveList();



    template <typename PRNG>
    PoissonDisk PopRandom( std::vector<PoissonDisk>& Points, PRNG& Generator )
    {
        const int Idx = Generator.RandomInt( Points.size()-1 );
        const PoissonDisk P = Points[ Idx ];
        Points.erase( Points.begin() + Idx );
        return P;
    }

    template <typename PRNG>
    PoissonDisk GenerateRandomPointAround( PoissonDisk& P, float MinDist, PRNG& Generator )
    {
        // start with non-uniform distribution
        float R1 = Generator.RandomFloat();
        float R2 = Generator.RandomFloat();

        // radius should be between MinDist and 2 * MinDist
        float Radius = MinDist * ( R1 + 1.0f );

        // random angle
        float Angle = 2 * 3.141592653589f * R2;

        // the new point is generated around the point (x, y)
        UT_Vector3 p = P.GetPosition();

        float X = p.x() + Radius * cos( Angle );
        float Y = p.y() + Radius * sin( Angle );

        return PoissonDisk( X, Y, 0 );
    }


    float r;    //radius
    int k;      //the limit of samples to choose before rejection in the algorithm, typically k = 30
    int numberOfPoints;
    sGrid backgrounGrid;
    vector<PoissonDisk> ProcessList;
    vector<PoissonDisk> SamplePoints;
    int n = 3; // n-dimensional
    float cellSize;
    DefaultPRNG Generator;
    bool Circle = true;


};
}

#endif
