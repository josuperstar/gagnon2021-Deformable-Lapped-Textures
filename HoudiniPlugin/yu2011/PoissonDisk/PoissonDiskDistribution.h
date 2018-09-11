#ifndef __PoissonDiskDistribution__
#define __PoissonDiskDistribution__
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <GEO/GEO_PointTree.h>
//#include <GA_ElementGroup.h>
#include <vector>
#include <openvdb/openvdb.h>
#include "PoissonDisk.h"

namespace Mokko {

class PoissonDiskDistribution
{

public:

    PoissonDiskDistribution();
    ~PoissonDiskDistribution();


    //PoissonDiskSampling creates a poisson disk distribution on a level set
    //It will take into account the existing Poisson disks.
    //If an existing point is too close to a neighbor, it will be flagged as invalid.
    std::vector<PoissonDisk> PoissonDiskSampling(GU_Detail *gdp, float diskRadius);

    static std::vector<GA_Offset> PoissonDiskUpdate(GU_Detail *trackers,GU_Detail *surface, GEO_PointTreeGAOffset &tree,float diskRadius, GA_PointGroup *surfaceGroup);

    void SetMaxId(long data){maxId = data;}

protected:
    std::vector<PoissonDisk> allpoints;
    long maxId = 0;

};
}

#endif
