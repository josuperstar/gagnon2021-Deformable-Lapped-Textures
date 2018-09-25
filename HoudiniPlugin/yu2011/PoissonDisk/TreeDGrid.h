#ifndef __TreeDGrid_h_
#define __TreeDGrid_h_
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <GEO/GEO_PointTree.h>
//#include <GA_ElementGroup.h>
#include <vector>
#include "PoissonDisk/PoissonDisk.h"



using namespace std;
namespace Mokko {


//Based on paper from Bridson "Fast Poisson Disk Sampling in Arbritrary Dimensions"
// http://people.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf



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



class TreeDGrid
{

public:
    TreeDGrid(){}

    void Clear()
    {
        m_Grid.clear();
        gridPoints.clear();
    }

    ~TreeDGrid()
    {
        m_Grid.clear();
        gridPoints.clear();
    }

    TreeDGrid( int W, int H, int D, float CellSize )
        : m_W( W )
        , m_H( H )
        , m_D( D )
        , m_CellSize( CellSize )
    {

        /*
        std::vector<  std::vector< std::vector< PoissonDisk > > >::iterator itx;
        std::vector<  std::vector< PoissonDisk > >::iterator ity;
        std::vector< PoissonDisk >::iterator itz;

        m_Grid.resize( m_W );
        for (itx = m_Grid.begin(); itx != m_Grid.end(); itx++)
        {
            (*itx).resize(m_H);
            for (ity = (*itx).begin(); ity != (*itx).end(); ity++)
            {
                (*ity).resize(m_D);
            }
        }
        */


        /*
        m_Grid.resize( m_W );

        for (int i = 0; i < m_W; i++)
        {
            m_Grid[i].resize(m_H);
        }

        for (int i = 0; i < m_W; i++)
        {
            //m_Grid[i].resize(m_H);
            for (int j = 0; j < m_H; j++)
            {
               m_Grid[i][j].resize(m_D);
            }
        }
        */
    }
    void Insert( PoissonDisk& P, float MinDist, float angleNormalThreshold)
    {
        int x = P.GetPosition().x() / m_CellSize ;
        int y = P.GetPosition().y() / m_CellSize ;
        int z = P.GetPosition().z() / m_CellSize ;
        UT_Vector3 G = UT_Vector3(x,y,z);
        //m_Grid[ G.x() ][ G.y() ][G.z() ] = P;
        //m_Grid[G.x()][G.y()] = P;

        std::vector<PoissonDisk> neighbors;

        if (!RespectCriterion(P,MinDist,m_CellSize,neighbors, angleNormalThreshold))
            P.SetValid(false);
        else
        {
            P.SetValid(true);
            gridPoints.push_back(P);
        }
    }

    bool RespectCriterion( PoissonDisk Point, float MinDist, float CellSize, std::vector<PoissonDisk> &neighbors ,float angleNormalThreshold);
    int GetNumberOfPoissonDisk(){return this->gridPoints.size();}

private:
    int m_W;
    int m_H;
    int m_D;
    float m_CellSize;

    std::vector<  std::vector< std::vector< PoissonDisk > > > m_Grid;
    std::vector< PoissonDisk > gridPoints;

};


}

#endif
