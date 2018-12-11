#ifndef __TreeDGrid_h_
#define __TreeDGrid_h_
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <GEO/GEO_PointTree.h>
//#include <GA_ElementGroup.h>
#include <vector>
#include "PoissonDisk/PoissonDisk.h"
#include <Core/Deformations/ParametersDeformablePatches.h>


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

    bool RespectCriterion(GU_Detail* trackers, GEO_PointTreeGAOffset &tree, UT_Vector3 newPointPosition, UT_Vector3 newPointNormal,  float killDistance, int &numberOfClosePoint, GA_Offset exclude , ParametersDeformablePatches params);

};


}

#endif
