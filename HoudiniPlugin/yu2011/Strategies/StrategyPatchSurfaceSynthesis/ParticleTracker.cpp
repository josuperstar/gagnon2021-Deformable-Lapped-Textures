#include "ParticleTracker.h"

#include <vector>
#include <algorithm>
#include <SYS/SYS_Math.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>
#include <SOP/SOP_Guide.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <GA/GA_ElementWrangler.h>
#include <algorithm>
#include <ctime>
#include <GEO/GEO_PointTree.h>
#include <GEO/GEO_PrimClassifier.h>
#include <GEO/GEO_PointClassifier.h>
#include <GEO/GEO_PrimConnector.h>
#include <GU/GU_NeighbourList.h>
#include <GU/GU_RayIntersect.h>
#include <GU/GU_Flatten.h>

//#include <Strategies/StrategySurfaceTextureSynthesis.h>
#include <Core/HoudiniUtils.h>


ParticleTracker::ParticleTracker()
{
    this->numberOfPatches = 0;
    this->maxId = 0;
    this->markerAdvectionTime = 0;
    this->numberOfConcealedPatches = 0;
    this->numberOfNewPatches = 0;
    this->numberOfDetachedPatches = 0;

}

bool ParticleTracker::SynthesisSurface(GU_Detail *trackerGdp, ParametersDeformablePatches params)
{
    return true;

}



//================================================================================================

//                                      CREATE TRACKERS BASED ON POISSON DISK

//================================================================================================


void ParticleTracker::CreateAndUpdateTrackersBasedOnPoissonDisk(GU_Detail *surface, GU_Detail *trackersGdp, GA_PointGroup *surfaceGroup,  ParametersDeformablePatches params)
{

    bool useDynamicTau = params.useDynamicTau;
    cout << "[ParticleTracker] CreateTrackersBasedOnPoissonDisk, with useDynamicTau at "<<useDynamicTau<<endl;

    if (surfaceGroup == 0x0)
        return;

    GA_PointGroup *markerGrp = (GA_PointGroup *)trackersGdp->pointGroups().find(this->markerGroupName.c_str());
    if (markerGrp == 0x0)
        markerGrp = trackersGdp->newPointGroup(markerGroupName.c_str());


    GA_PrimitiveGroup *markerGrpPrims;
    if (tackerPolygon)
    {
        markerGrpPrims = (GA_PrimitiveGroup *)trackersGdp->primitiveGroups().find(this->markerGroupName.c_str());
        if (markerGrpPrims == 0x0)
            markerGrpPrims = trackersGdp->newPrimitiveGroup(markerGroupName.c_str());
    }

    GA_RWHandleV3 attVSurface(surface->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3 attNSurface(surface->findFloatTuple(GA_ATTRIB_POINT,"N", 3));

    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleV3   attCenterUV(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"centerUV", 3));
    GA_RWHandleV3   attV(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   attCd(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));
    GA_RWHandleI    attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleF    attLife(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleI    attSpawn(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"spawn",1));
    GA_RWHandleI    attFadeIn(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"fadeIn",1));
    GA_RWHandleI    attActive(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));
    GA_RWHandleI    attIsMature(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"isMature", 1));
    GA_RWHandleI    attDensity(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"density", 1));
    GA_RWHandleI    attPoissonDisk(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"poissondisk", 1));
    GA_RWHandleF    attBlend(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    GA_RWHandleF    attRandT(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,randomThresholdDistortion,1));
    GA_RWHandleF    attMaxDeltaOnD(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"maxDeltaOnD",1));
    GA_RWHandleI    attDeleteFaster(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"deleteFaster", 1));
    GA_RWHandleV3 refAttV(surface->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3 refAttN(surface->addFloatTuple(GA_ATTRIB_POINT,"N", 3));

    UT_Vector3 position;
    UT_Vector3 N;
    UT_Vector3 defaultDirection(1,0,0);

    float thresholdDistance = params.maximumProjectionDistance;

    GU_MinInfo mininfo;
    GU_RayIntersect ray(surface);
    ray.init();

    int id = 0;
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        id = attId.get(ppt);
        int active = attActive.get(ppt);
        float currentLife = attLife.get(ppt);
        int currentSpawn = attSpawn.get(ppt);


        UT_Vector3 velocity;

        float dynamicTau = attMaxDeltaOnD.get(ppt);
        UT_Vector3 centerUV = attCenterUV.get(ppt);

        //Dead patches are not updated
        if (currentLife <= 0 && active == 0)
        {
            continue;
        }

        //============================ PROJECION ON MESH =======================
        UT_Vector3 p1 = trackersGdp->getPos3(ppt);
        mininfo.init(thresholdDistance,0.0001);
        ray.minimumPoint(p1,mininfo);

        if (!mininfo.prim)
        {
            //cout << "No primitive to project on"<<endl;
            continue;
        }

        const GEO_Primitive *geoPrim = mininfo.prim;
        int vertexCount = geoPrim->getVertexCount();
        if (vertexCount != 3)
        {
            //cout << "vertex count "<<vertexCount<<" for primitive "<<geoPrim->getMapOffset()<<endl;
            continue;
        }
        //get pos of hit
        UT_Vector4 hitPos;
        mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
        if (distance3d(p1,hitPos) < thresholdDistance)
        {
            p1 = hitPos;

            //------------------------------PARAMETRIC COORDINATE -----------------------------------
            GA_Offset primOffset = mininfo.prim->getMapOffset();
            float u = mininfo.u1;
            float v = mininfo.v1;
            GEO_Primitive *prim = surface->getGEOPrimitive(primOffset);

            GA_Offset vertexOffset0 = prim->getVertexOffset(0);

            GA_Offset pointOffset0  = surface->vertexPoint(vertexOffset0);
            UT_Vector3 n0 = refAttN.get(pointOffset0);
            UT_Vector3 v0 = refAttV.get(pointOffset0);

            GA_Offset vertexOffset1 = prim->getVertexOffset(1);
            GA_Offset pointOffset1  = surface->vertexPoint(vertexOffset1);
            UT_Vector3 n1 = refAttN.get(pointOffset1);
            UT_Vector3 v1 = refAttV.get(pointOffset1);

            GA_Offset vertexOffset2 = prim->getVertexOffset(2);
            GA_Offset pointOffset2  = surface->vertexPoint(vertexOffset2);
            UT_Vector3 n2 = refAttN.get(pointOffset2);
            UT_Vector3 v2 = refAttV.get(pointOffset2);

            N                   = n0+u*(n1-n0)+v*(n2-n0);
            velocity = v0+u*(v1-v0)+v*(v2-v0);
        }
        else
        {
            //can't project, we delete
            attLife.set(ppt,0);
            attActive.set(ppt,0);
        }


        //========================================================================



        //========================= UPDATE ===============================
        //we want to fade out poisson disk that are flagged a inactive and that are mature (life spawn greater than the fading in time)
        //or that are too close to each other

        int maxNumberOfNeighbour = 5; // TODO promotve that variable
        int density = attDensity.get(ppt);
        //-------------- deleting faster logic ------------------
        //Can we move this to the ParticleTracker update ?
        int deleteFaster = attDeleteFaster.get(ppt);
        int numberOfNeighbourThreshold = 1; // TODO: promote this variable
        if (density > numberOfNeighbourThreshold && deleteFaster == 0)
        {
            attDeleteFaster.set(ppt, 1);
        }
        else if(deleteFaster == 1 && density <= numberOfNeighbourThreshold)
        {
            attDeleteFaster.set(ppt, 0);
        }
        //-------------------------------------------------------

        int increment = density;
        if (maxNumberOfNeighbour <= density)
            increment = maxNumberOfNeighbour;

        if (!useDynamicTau)
            increment = 0;

        //int deleteFaster = attDeleteFaster.get(ppt);
        bool isMature = (currentSpawn >= params.fadingTau);
        if (isMature)
            attIsMature.set(ppt,1);
        if (active == 0 && deleteFaster == 1 && isMature)
        {
            currentLife -= 1.0f+((float)increment);
        }
        else if(active == 0 && deleteFaster == 0 && isMature)
        {
            currentLife -= 1.0f;
        }
        //fade in
        else if (currentSpawn < params.fadingTau)
        {
            //currentLife++;
            //currentSpawn++;

            currentLife += 1.0f+(float)increment;
            if (currentSpawn == 0)
                currentSpawn+= 1;
            else
                currentSpawn+= 1+increment;

        }
        if (currentLife > (float)params.fadingTau)
            currentLife = (float)params.fadingTau;
        if (currentLife < 0)
            currentLife = 0;

        float deletionLife = params.fadingTau;
        float blending = (float)currentLife/(float(deletionLife));
        attBlend.set(ppt,blending);

        //==============================================

        position = p1;
        trackersGdp->setPos3(ppt,position);
        N.normalize();

        attV.set(ppt,velocity);
        attN.set(ppt,N);
        attCenterUV.set(ppt,centerUV);
        trackersGdp->setPos3(ppt,position);

        float life = currentLife;
        attLife.set(ppt,life);

        float temporalComponetKt = ((float)life)/params.fadingTau;

        attBlend.set(ppt,temporalComponetKt);
        attSpawn.set(ppt,currentSpawn);
        attMaxDeltaOnD.set(ppt,dynamicTau);
        float randt = (((double) rand() / (RAND_MAX)));
        attRandT.set(ppt,randt);

        //numberOfPatches++;
    }
}




//================================================================================================

//                                      ADVECT MARKERS

//================================================================================================


void ParticleTracker::AdvectMarkers(GU_Detail *surfaceGdp,GU_Detail *trackersGdp, ParametersDeformablePatches params,GEO_PointTreeGAOffset &tree)
{
    cout <<this->approachName<< " Advect Markers"<<endl;

    std::clock_t startAdvection;
    startAdvection = std::clock();

    numberOfPatches = 0;
    maxId = 0;

    GA_RWHandleV3   attV(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));

    GA_ROHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleI    attFadeIn(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"fadeIn",1));
    GA_RWHandleF    temporalComponentKt = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    GA_RWHandleF    attLife(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_ROHandleI    attDensity(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"density",1));
    GA_RWHandleI    attActive(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"active", 1));

    float deletionLife = params.fadingTau;

    if (attV.isInvalid())
    {
        cout << "Markers have no velocity";
        return;
    }

    if (attDensity.isInvalid())
    {
        cout << "The is no density defined"<<endl;
        return;
    }

    UT_Vector3 v;
    UT_Vector3 p;
    UT_Vector3 p1;
    float dt = 1.0f/24.0f;
    float thresholdDistance = params.maximumProjectionDistance;

    //============================================================

    GA_RWHandleV3 refAttV(surfaceGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3 refAttN(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleV3 AttCd(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));

    //===============================================================================
    {
        GU_MinInfo mininfo;

        GU_RayIntersect ray(surfaceGdp);
        ray.init();

        UT_Vector3 refDir(1,0,0);
        UT_Vector3 N;
        GA_Offset ppt;
        int id;
        int density;
        float currentLife = 0;
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {

            v = attV.get(ppt);
            N = attN.get(ppt);
            density = attDensity.get(ppt);
            //cout << "advecting point "<<ppt<<endl;
            if (N.length() < epsilon)
            {
                //cout << "N lenght is too small"<<endl;
                continue;
            }
            p = trackersGdp->getPos3(ppt);

            id = attId.get(ppt);
            if(id > maxId)
                maxId = id;

            currentLife = attLife.get(ppt);

            //update fadein
            int fadeIn = attFadeIn.get(ppt);
            if (fadeIn < deletionLife)
            {
                fadeIn = fadeIn +1;
                attFadeIn.set(ppt,fadeIn);
            }


            //-----------------------------------------
            //advect
            UT_Vector3 d = v*dt;
            p1 = p+d;
            trackersGdp->setPos3(ppt,p1);
            //-----------------------------------------

            p1 = trackersGdp->getPos3(ppt);
            mininfo.init(thresholdDistance,0.0001);
            ray.minimumPoint(p1,mininfo);

            if (!mininfo.prim)
            {
                cout << "No primitive to project on"<<endl;
                continue;
            }

            const GEO_Primitive *geoPrim = mininfo.prim;
            int vertexCount = geoPrim->getVertexCount();
            if (vertexCount != 3)
            {
                //cout << "vertex count "<<vertexCount<<" for primitive "<<geoPrim->getMapOffset()<<endl;
                continue;
            }
            //get pos of hit
            UT_Vector4 hitPos;
            mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
            if (distance3d(p1,hitPos) < thresholdDistance)
            {
                p1 = hitPos;
                trackersGdp->setPos3(ppt,p1);
                AttCd.set(ppt,UT_Vector3(0,1,0));

                //------------------------------PARAMETRIC COORDINATE -----------------------------------
                GA_Offset primOffset = mininfo.prim->getMapOffset();
                float u = mininfo.u1;
                float v = mininfo.v1;
                GEO_Primitive *prim = surfaceGdp->getGEOPrimitive(primOffset);

                GA_Offset vertexOffset0 = prim->getVertexOffset(0);

                GA_Offset pointOffset0  = surfaceGdp->vertexPoint(vertexOffset0);
                UT_Vector3 n0 = refAttN.get(pointOffset0);
                UT_Vector3 v0 = refAttV.get(pointOffset0);

                GA_Offset vertexOffset1 = prim->getVertexOffset(1);
                GA_Offset pointOffset1  = surfaceGdp->vertexPoint(vertexOffset1);
                UT_Vector3 n1 = refAttN.get(pointOffset1);
                UT_Vector3 v1 = refAttV.get(pointOffset1);

                GA_Offset vertexOffset2 = prim->getVertexOffset(2);
                GA_Offset pointOffset2  = surfaceGdp->vertexPoint(vertexOffset2);
                UT_Vector3 n2 = refAttN.get(pointOffset2);
                UT_Vector3 v2 = refAttV.get(pointOffset2);

                N                   = n0+u*(n1-n0)+v*(n2-n0);
                UT_Vector3 velocity = v0+u*(v1-v0)+v*(v2-v0);
                attV.set(ppt,velocity);

                attN.set(ppt,N);
                //------------------------------------------------------------------------------------
                numberOfPatches++;
            }
            else
            {
                //delete this point because we can't project it, probably because of a sudden topological change.
                //cout << "delete "<<id<<" because we can't project it, probably because of a sudden topological change."<<endl;
                AttCd.set(ppt,UT_Vector3(1,0,0));

                //detached poisson disks have to be deleted directly, not fading out.
                attLife.set(ppt,0);
                attActive.set(ppt,0);

                numberOfPatches--;
                numberOfDetachedPatches++;

                trackersGdp->setPos3(ppt,p1);
                //cout << "new new position "<<p1<<endl;
            }
        }
    }


    //---- for visualisation purpose
    const GA_SaveOptions *options;
    UT_StringArray *errors;
    string beforeUpdateString = params.trackersFilename + "advectedTrackers.bgeo";
    const char* filename = beforeUpdateString.c_str();//"dlttest.bgeo";
    //trackersGdp->save(filename,options,errors);
    //----------------------------------


    cout << this->approachName<< " There are "<<trackersGdp->getNumPoints() << " trackers after advection"<<endl;
    cout << this->approachName<< " There are "<<numberOfDetachedPatches<< " detached trackers"<<endl;
    cout << this->approachName<< " There are "<<numberOfPatches << " number of patches"<<endl;

    this->markerAdvectionTime += (std::clock() - startAdvection) / (double) CLOCKS_PER_SEC;

    //return trackers;
}

//================================================================================================

//                                      COMPUTE DENSITY

//================================================================================================

void ParticleTracker::ComputeDensity(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree)
{

    cout <<this->approachName<< " Compute Density"<<endl;

    UT_Vector3 v,vn,p;

    float epsilon = 0.001;

    GA_RWHandleI attDensity(trackers->addIntTuple(GA_ATTRIB_POINT,"density",1));

    //GA_RWHandleV3 attV(trackers->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    float patchRadius = params.poissondiskradius;
    GA_Offset ppt;

    GA_FOR_ALL_PTOFF(trackers,ppt)
    {

        p = trackers->getPos3(ppt);

        GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;
        tree.findAllCloseIdx(p,
                             patchRadius,
                             close_particles_indices);

        int l = close_particles_indices.entries();

        attDensity.set(ppt,l);

    }
}



//================================================================================================

//                                      COMPUTE DIVERGENCE

//================================================================================================

void ParticleTracker::ComputeDivergence(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree)
{

    cout <<this->approachName<< " Compute Divergence"<<endl;

    UT_Vector3 v,vn,p;
    UT_Vector3 N,Nn;
    float epsilon = 0.001;
    GA_RWHandleV3 attVSurface(surfaceGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleF attDivergence(trackers->addFloatTuple(GA_ATTRIB_POINT,"divergence",1));

    GA_RWHandleV3 attV(trackers->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    float patchRadius = params.poissondiskradius*3;
    GA_Offset ppt;

    GA_FOR_ALL_PTOFF(trackers,ppt)
    {
        v = attV.get(ppt);
        v.normalize();

        float sumDotWeighted = 0;
        float w_k = 0;

        p = trackers->getPos3(ppt);

        GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;
        tree.findAllCloseIdx(p,
                             patchRadius,
                             close_particles_indices);

        unsigned l = close_particles_indices.entries();
        if (l > 0)
        {
            GA_Offset neighbor;
            for(int j=0; j<l;j++ )
            {
                neighbor = close_particles_indices.array()[j];
                vn = attVSurface.get(neighbor);
                float ln = vn.length();
                if (ln < epsilon)
                    continue;
                vn.normalize();
                //float d = dot(vn,v);
                float dw = dot(vn,v) * ln;
                w_k += ln;
                sumDotWeighted += dw;
            }
        }

        float divergence = (1+sumDotWeighted/w_k)/2;
        attDivergence.set(ppt,divergence);

    }
}

