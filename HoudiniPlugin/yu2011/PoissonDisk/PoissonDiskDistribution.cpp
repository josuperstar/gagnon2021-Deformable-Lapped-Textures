#include "PoissonDiskDistribution.h"
#include "Core/HoudiniUtils.h"

#include <GU/GU_RandomPoint.h>


using namespace Mokko;
using namespace std;




PoissonDiskDistribution::PoissonDiskDistribution()
{
    this->allpoints.clear();
}

PoissonDiskDistribution::~PoissonDiskDistribution()
{
    this->allpoints.clear();
}


std::vector<PoissonDisk> PoissonDiskDistribution::PoissonDiskSampling(GU_Detail *gdp, float diskRadius)
{
    cout << "PoissonDiskDistribution::PoissonDiskSampling"<<endl;
}




//================================================================================================

//                                      UPDATE POISSON DISK DISTRIBUTION

//================================================================================================

vector<GA_Offset> PoissonDiskDistribution::PoissonDiskUpdate(GU_Detail *trackers,GU_Detail *surface, GEO_PointTreeGAOffset &tree,float diskRadius, GA_PointGroup *surfaceGroup )
{

    cout << "[PoissonDiskDistribution] PoissonDiskUpdate"<<endl;

    vector<GA_Offset> newPoissonDisk;


    GEO_PointTreeGAOffset trackersTree;
    trackersTree.build(trackers,NULL);

    GA_RWHandleI attActive(trackers->addIntTuple(GA_ATTRIB_POINT,"active", 1));
    GA_RWHandleI attPoissonDisk(trackers->addIntTuple(GA_ATTRIB_POINT,"poissondisk", 1));
    GA_RWHandleI attPoissonDiskSurface(surface->addIntTuple(GA_ATTRIB_POINT,"poissondisk", 1));
    {

        GA_Offset ppt;

        float radius = (diskRadius/2)+(diskRadius/20);

        //transfering poisson disk attribute to the surface
        //the points that have no attribute after the transfer will be in a pool for new poisson disk points
        HoudiniUtils::AttributeTransfert(trackers,surface,tree,radius,attPoissonDisk,attPoissonDiskSurface);


        //mark poisson disk to delete
        //count the number of point inside the radius/2. If greater than 0, we have to delete it
        {
            float innerRadius = radius/2;
            UT_Vector3 p;
            UT_Vector3 pN;
            GA_Offset ppt;
            {
                // Transfert velocity from
                bool hasPointInside = false;
                GA_FOR_ALL_PTOFF(trackers,ppt)
                {

                    if (attActive.get(ppt) == 0)
                        continue;

                    p = trackers->getPos3(ppt);

                    GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;
                    trackersTree.findAllCloseIdx(p,
                                         innerRadius,
                                         close_particles_indices);

                    unsigned close_particles_count = close_particles_indices.entries();
                    if (close_particles_count > 0)
                    {
                        //compute number of active poisson disk
                        for(int j=0; j<close_particles_count; j++)
                        {

                            int index = close_particles_indices.array()[j];
                            if (ppt == index )
                                continue;
                            pN = trackers->getPos3(index);
                            float d = distance3d(p,pN);
                            if (d < innerRadius)
                            {
                                if (attActive.get(index) == 1)
                                {
                                    //mark as inactive to delete the point

                                    hasPointInside = true;
                                }
                            }
                        }
                    }
                    if (hasPointInside)
                    {
                        attActive.set(ppt,0);
                        hasPointInside = false;
                    }

                }
            }
        }

        cout << "[PoissonDiskDistribution] Creating new points"<<endl;
        {
            GA_FOR_ALL_PTOFF(surface,ppt)
            {
                if (!surfaceGroup->containsOffset(ppt))
                    continue;
                int poisson = attPoissonDiskSurface.get(ppt);
                if (poisson == 0)
                {
                    //adding new point
                    newPoissonDisk.push_back(ppt);
                    //UT_Vector3 position = surface->getPos3(ppt);
                    //GA_Offset newPoint = trackers->appendPointOffset();
                    //trackers->setPos3(newPoint,position);
                    //cout << "[PoissonDiskDistribution] Adding new point"<<endl;
                }
                else
                {
                    //cout << "We have a transfert of poisson disk"<<endl;
                }
            }
        }
        cout << "[PoissonDiskDistribution] There is "<<newPoissonDisk.size()<<" new points"<<endl;

    }
    return newPoissonDisk;
}

