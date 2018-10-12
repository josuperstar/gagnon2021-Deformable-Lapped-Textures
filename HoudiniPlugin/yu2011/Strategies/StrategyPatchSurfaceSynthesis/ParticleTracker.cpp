#include "ParticleTracker.h"

#include <vector>
#include <algorithm>
#include <SYS/SYS_Math.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
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

void ParticleTracker::DeleteTracker(GU_Detail* trackers,int trackerId)
{
    GA_RWHandleI attId(trackers->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_PointGroup *grpToDestroy = (GA_PointGroup *)trackers->newPointGroup("ToDelete");

    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackers,ppt)
    {
        int id = attId.get(ppt);
        if (id == trackerId)
        {
            grpToDestroy->addOffset(ppt);
        }
    }

    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;
    trackers->deletePoints(*grpToDestroy,mode);
    trackers->destroyPointGroup(grpToDestroy);
}


//================================================================================================

//                                      CREATE TRACKERS BASED ON POISSON DISK

//================================================================================================


void ParticleTracker::CreateAndUpdateTrackersBasedOnPoissonDisk(GU_Detail *surface, GU_Detail *trackersGdp, GA_PointGroup *surfaceGroup,  ParametersDeformablePatches params, vector<PoissonDisk> &existingPoissonDisks)
{

    bool useDynamicTau = params.useDynamicTau;
    cout << "[ParticleTracker] CreateTrackersBasedOnPoissonDisk based on "<<existingPoissonDisks.size()<<" poisson disk, with useDynamicTau at "<<useDynamicTau<<endl;

    //float maxDelta = 5.0f;

    float tau = (float)params.fadingTau;
    float maxDelta = tau;
    float dynamicLifespaw = 2.0f;

    cout << "max delta = "<<maxDelta<<endl;
    cout << "tau = "<<tau<<endl;


    if (surfaceGroup == 0x0)
        return;

    trackersGdp->clearAndDestroy();

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
    GA_RWHandleV3   attV(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   attCd(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));
    GA_RWHandleI    attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleF    attLife(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleI    attSpawn(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"spawn",1));
    GA_RWHandleI    attFadeIn(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"fadeIn",1));
    GA_RWHandleI    attActive(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));
    GA_RWHandleI    attIsMature(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"isMature", 1));
    GA_RWHandleI    attPoissonDisk(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"poissondisk", 1));
    GA_RWHandleF    attBlend(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    GA_RWHandleF    attRandT(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,randomThresholdDistortion,1));
    GA_RWHandleF    attMaxDeltaOnD(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"maxDeltaOnD",1));

    GA_RWHandleV3 refAttV(surface->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3 refAttN(surface->addFloatTuple(GA_ATTRIB_POINT,"N", 3));

    //GA_Offset ppt;
    GA_Offset newPoint;

    UT_Vector3 position;
    UT_Vector3 vRef;
    UT_Vector3 N;

    UT_Vector3 defaultDirection(1,0,0);
    UT_Vector3 S,T;
    float thresholdDistance = params.maximumProjectionDistance;

    vector<PoissonDisk>::iterator it;
    GU_MinInfo mininfo;
    GU_RayIntersect ray(surface);
    ray.init();

    int id = 0;
    for(it = existingPoissonDisks.begin(); it != existingPoissonDisks.end(); ++it)
    {
        PoissonDisk currentPoissonDisk = (*it);
        id = currentPoissonDisk.GetId();
        float currentLife = currentPoissonDisk.GetLife();
        int currentSpawn = currentPoissonDisk.GetSpawn();
        currentSpawn++;

        UT_Vector3 velocity;

        float dynamicTau = currentPoissonDisk.GetDynamicTau();

        //=================== UPDATE ===================
        if (currentLife <= 0 && params.frame > params.startFrame)
        {
            continue;
        }

        //============================ PROJECION ON MESH =======================
        UT_Vector3 p1 = currentPoissonDisk.GetPosition();
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

        //========================================================================

        int active = currentPoissonDisk.IsValid();
        int isMature = currentPoissonDisk.IsMature();

        if (active == 0 && currentSpawn >= params.fadingTau)
        {
            currentLife--;
        }
        //fade in
        else if (currentSpawn < params.fadingTau)
        {
            currentLife++;
        }
        if (currentLife > (float)params.fadingTau)
            currentLife = (float)params.fadingTau;
        if (currentLife < 0)
            currentLife = 0;

        //==============================================
        //if (!surfaceGroup->containsOffset(ppt))
        //    continue;
        //position = currentPoissonDisk.GetPosition();
        position = p1;
        //vRef = attVSurface.get(ppt);
        //vRef = currentPoissonDisk.GetVelocity();
        //N = currentPoissonDisk.GetNormal();
        N.normalize();

        newPoint = trackersGdp->appendPoint();
        attV.set(newPoint,velocity);
        attN.set(newPoint,N);

        trackersGdp->setPos3(newPoint,position);
        markerGrp->addOffset(newPoint);
        attCd.set(newPoint,UT_Vector3(0,1,1));

        //cout << "[ParticleTracker] creating tracker "<<currentPoissonDisk.GetId()<<endl;

        attId.add(newPoint,currentPoissonDisk.GetId());
        float life = currentLife;
        attLife.set(newPoint,life);

        float temporalComponetKt = ((float)life-1)/params.fadingTau;

        attBlend.set(newPoint,temporalComponetKt);
        attSpawn.set(newPoint,currentSpawn);
        attMaxDeltaOnD.set(newPoint,dynamicTau);
        //cout << "Point "<<id<< " active == "<<active<<endl;
        attActive.set(newPoint,active);
        attIsMature.set(newPoint,isMature);
        attPoissonDisk.set(newPoint,1);
        /*
        GA_RWHandleF blend = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));

        if (params.startFrame == params.frame)
        {
            attFadeIn.set(newPoint,params.fadingTau);
            blend.set(newPoint,1.0f);
        }
        else
        {
            attFadeIn.set(newPoint,0);
            blend.set(newPoint,0.0f);
        }
        */
        float randt = (((double) rand() / (RAND_MAX)));
        attRandT.set(newPoint,randt);

        numberOfPatches++;
    }
}




//================================================================================================

//                                      ADVECT MARKERS

//================================================================================================


vector<GA_Offset> ParticleTracker::AdvectMarkers(GU_Detail *surfaceGdp,GU_Detail *trackersGdp, ParametersDeformablePatches params,GEO_PointTreeGAOffset &tree)
{
    cout <<this->approachName<< " Advect Markers"<<endl;

    std::clock_t startAdvection;
    startAdvection = std::clock();

    numberOfPatches = 0;
    maxId = 0;

    vector<GA_Offset> trackers;

    GA_RWHandleV3   attV(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_ROHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleI    attFadeIn(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"fadeIn",1));
    GA_RWHandleF    temporalComponentKt = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    GA_RWHandleF    attLife(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    float deletionLife = params.fadingTau;

    if (attV.isInvalid())
    {
        cout << "Markers have no velocity";
        return trackers;
    }

    UT_Vector3 v;
    UT_Vector3 p;
    UT_Vector3 p1;
    float dt = 1.0f/24.0f;
    float thresholdDistance = params.maximumProjectionDistance;

    GA_PointGroup * grpToDestroy;
    //============================================================

    GA_RWHandleV3 refAttV(surfaceGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3 refAttN(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleV3 AttCd(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));

    GA_PointGroup *markerGrp = (GA_PointGroup *)trackersGdp->pointGroups().find(this->markerGroupName.c_str());

    //===============================================================================
    {
        grpToDestroy = (GA_PointGroup *)trackersGdp->newPointGroup("ToDelete");
        GU_MinInfo mininfo;

        GU_RayIntersect ray(surfaceGdp);
        ray.init();

        UT_Vector3 refDir(1,0,0);
        UT_Vector3 N;

        bool toAdd = false;
        GA_Offset ppt;
        int id;
        float currentLife = 0;
        GA_FOR_ALL_GROUP_PTOFF(trackersGdp,markerGrp,ppt)
        {
            toAdd = true;
            v = attV.get(ppt);
            N = attN.get(ppt);
            //cout << "advecting point "<<ppt<<endl;
            if (N.length() < epsilon)
            {
                //cout << "N lenght is too small"<<endl;
                toAdd = false;
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
            float blending = (float)currentLife/(float(deletionLife));
            temporalComponentKt.set(ppt,blending);

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
                //delete this point
                //cout << "Delete this point because it can't be projected" <<endl;
                AttCd.set(ppt,UT_Vector3(1,0,0));
                grpToDestroy->addOffset(ppt);

                numberOfPatches--;
                numberOfDetachedPatches++;
                toAdd = false;

                trackersGdp->setPos3(ppt,p1);
                //cout << "new new position "<<p1<<endl;
            }

            //----------------------------------------------------
            //To add or not to add, this is the question.
            if(toAdd)
                trackers.push_back(ppt);
            else
                grpToDestroy->addOffset(ppt);
            //----------------------------------------------------
        }
    }

    //---- for visualisation purpose
    const GA_SaveOptions *options;
    UT_StringArray *errors;
    string beforeUpdateString = params.trackersFilename + "advectedTrackers.bgeo";
    const char* filename = beforeUpdateString.c_str();//"dlttest.bgeo";
    trackersGdp->save(filename,options,errors);
    //----------------------------------


    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;
    trackersGdp->deletePoints(*grpToDestroy,mode);
    trackersGdp->destroyPointGroup(grpToDestroy);

    cout << this->approachName<< " There are "<<trackers.size() << " trackers after advection"<<endl;
    cout << this->approachName<< " There are "<<numberOfDetachedPatches<< " detached trackers"<<endl;
    cout << this->approachName<< " There are "<<numberOfPatches << " number of patches"<<endl;

    this->markerAdvectionTime += (std::clock() - startAdvection) / (double) CLOCKS_PER_SEC;

    return trackers;
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

