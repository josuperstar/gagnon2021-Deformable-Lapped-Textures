#include "Yu2011.h"
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
#include <PoissonDisk/Bridson2012PoissonDiskDistribution.h>
#include <Core/HoudiniUtils.h>


Yu2011::Yu2011(GU_Detail* surface) : DeformableGrids()
{
    this->numberOfPatches = 0;
    this->maxId = 0;
    this->gridCenterPosition.clear();

    /*
    uvsArray->clear(this->uvsAtt);
    patchIdsAtt->clear(patchIdsArrayAttrib);
    alphaAtt->clear(alphaArrayAtt);
    */
    //=========================== PATCH ID ARRAY ATTRIB ==========================
    UT_String patchName(this->patchIdsName);
    patchIdsArrayAttrib = surface->findIntArray(GA_ATTRIB_POINT,
                                            patchName,
                                            // Allow any tuple size to match
                                            -1, -1);
    if (!patchIdsArrayAttrib)
    {
        patchIdsArrayAttrib = surface->addIntArray(GA_ATTRIB_POINT,
                                    patchName,
                    // Tuple size one means we group the array into
                    // logical groups of 1.  It does *NOT* affect
                    // the length of the arrays, which are always
                    // measured in ints.
                                    1);
    }
    // Make sure we are an array.  Note tuples do not match to this,
    // nor do GA_*Handle* match!
    // We will match both int and float here, however.
    // (For string, getAIFSharedStringArray)
    patchIdsAtt = patchIdsArrayAttrib->getAIFNumericArray();
    if (!patchIdsAtt)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) patchName);
        //addError(SOP_MESSAGE, buf.buffer());
        //return error();
    }
    patchIdsArrayAttrib->clearDataId();

    //=========================== ALPHA ARRAY ATTRIB ==========================
    UT_String uvname(this->uvArrayName);
    uvsAtt = surface->findFloatArray(GA_ATTRIB_POINT,
                                            uvname,
                                            // Allow any tuple size to match
                                            -1, -1);

    if (!uvsAtt)
    {
        uvsAtt = surface->addFloatArray(GA_ATTRIB_POINT,
                                    uvname,
                    // Tuple size one means we group the array into
                    // logical groups of 1.  It does *NOT* affect
                    // the length of the arrays, which are always
                    // measured in ints.
                                    1);
    }

    uvsArray = uvsAtt->getAIFNumericArray();
    if (!uvsArray)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) uvname);
        //addError(SOP_MESSAGE, buf.buffer());
        //return error();
    }
    uvsAtt->clearDataId();
    //=========================== ALPHA ARRAY ATTRIB ==========================

    UT_String aname(this->alphaArrayName);
    alphaArrayAtt = surface->findFloatArray(GA_ATTRIB_POINT,
                                            aname,
                                            // Allow any tuple size to match
                                            -1, -1);
    if (!alphaArrayAtt)
    {
        alphaArrayAtt = surface->addFloatArray(GA_ATTRIB_POINT,
                                    aname,
                    // Tuple size one means we group the array into
                    // logical groups of 1.  It does *NOT* affect
                    // the length of the arrays, which are always
                    // measured in ints.
                                    1);
    }
    // Make sure we are an array.  Note tuples do not match to this,
    // nor do GA_*Handle* match!
    // We will match both int and float here, however.
    // (For string, getAIFSharedStringArray)
    alphaAtt = alphaArrayAtt->getAIFNumericArray();
    if (!alphaAtt)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) aname);
        //addError(SOP_MESSAGE, buf.buffer());
        //return error();
    }
    alphaArrayAtt->clearDataId();

    this->uvFlatteningTime = 0;
    this->gridMeshCreation = 0;
    this->gridAdvectionTime = 0;
    this->markerAdvectionTime = 0;
    this->patchCreationTime = 0;
    this->nbOfFlattenedPatch = 0;
    this->updatePatchesTime = 0;
    this->numberOfConcealedPatches = 0;
    this->numberOfNewPatches = 0;
    this->numberOfDetachedPatches = 0;
}

Yu2011::~Yu2011()
{
    this->rays.clear();
}

bool Yu2011::SynthesisSurface(GU_Detail *gdp, ParametersDeformablePatches params)
{
    return true;
}

//================================================================================================

//                                     POISSON DISK SAMPLING

//================================================================================================


vector<PoissonDisk> Yu2011::PoissonDiskSampling(GU_Detail *gdp, GU_Detail *levelSet, GU_Detail *trackersGdp, GA_PointGroup *markerGroup ,ParametersDeformablePatches params)
{

    //This is a function that does a Poisson Disk Sampling using the approach of Bridson 2012 paper
    //This function is a wrapper to the Bridson2012PoissonDiskDistribution class.
    //It basically take the points from Houdini and fill it to the approach.

    std::clock_t addPoissonDisk;
    addPoissonDisk = std::clock();

    cout << "[Yu2011:PoissonDiskSampling]"<<endl;
    GA_RWHandleV3   attV(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleF    attExistingLife(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"life", 1));
    GA_RWHandleI    attExistingSpawn(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"spawn", 1));
    GA_RWHandleI    attExistingActive(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"active", 1));
    GA_RWHandleI    attExistingMature(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"isMature", 1));
    GA_RWHandleF    attMaxDeltaOnD(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"maxDeltaOnD",1));
    GA_Offset ppt;
    vector<PoissonDisk> oldPoints;
    vector<GA_Offset> newPatchesPoints;

    if(params.startFrame == params.frame)
    {
        GA_Offset ppt;
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {

            PoissonDisk p(trackersGdp->getPos3(ppt));
            p.SetId(attId.get(ppt));
            p.SetLife(attExistingLife.get(ppt));
            p.SetSpawn(attExistingSpawn.get(ppt));
            p.SetDynamicTau(0.0f);
            p.SetNormal(attN.get(ppt));

            if (params.startFrame == params.frame)
            {
                p.SetValid(1);
                p.SetId(ppt+1);
                p.SetLife(params.fadingTau);
                p.SetSpawn(params.fadingTau);
                p.SetDynamicTau(0.0f);
            }
            else
                p.SetValid(attExistingActive.get(ppt));
                p.SetMature(attExistingMature.get(ppt));
            oldPoints.push_back(p);
        }
    }
    else
    {
        //------------------ existing points ----------------------
        //we should move this in a function
        GA_FOR_ALL_GROUP_PTOFF(trackersGdp,markerGroup,ppt)
        {

            PoissonDisk p(trackersGdp->getPos3(ppt));

            int id = attId.get(ppt);
            //cout << "Existing Poisson "<<id<<endl;
            p.SetId(id);
            p.SetLife(attExistingLife.get(ppt));
            p.SetSpawn(attExistingSpawn.get(ppt));
            p.SetDynamicTau(attMaxDeltaOnD.get(ppt));
            p.SetNormal(attN.get(ppt));
            //cout << "existing point "<<id<<" valid "<<attExistingActive.get(ppt)<<endl;
            p.SetValid(attExistingActive.get(ppt));
            p.SetMature(attExistingMature.get(ppt));
            p.SetVelocity((attV.get(ppt)));
            oldPoints.push_back(p);
            newPatchesPoints.push_back(ppt);

        }
    }
    //-----------------------------------------------------------

    cout << "[Yu2011] deleting other groups for surface"<<endl;
    int numberOfPoints = newPatchesPoints.size();

    cout << "[Yu2011] we have "<<numberOfPoints << " existing point(s) in trackersGdp"<<endl;
    Bridson2012PoissonDiskDistribution poissonDiskDistribution;
    poissonDiskDistribution.SetNumberOfPoint(numberOfPoints);
    poissonDiskDistribution.initializeGrid(oldPoints,params.poissondiskradius, params.poissonAngleNormalThreshold);

    vector<PoissonDisk> PPoints = poissonDiskDistribution.PoissonDiskSampling(levelSet,params.poissondiskradius, params.poissonAngleNormalThreshold);

    cout << "[Yu2011] poisson disk sample "<<PPoints.size()<< " point(s)"<<endl;

    if(params.startFrame == params.frame)
    {
        //all PPpoints have to have a full life
        vector<PoissonDisk>::iterator itPoisson;
        for(itPoisson = PPoints.begin(); itPoisson != PPoints.end(); ++itPoisson)
        {
            (*itPoisson).SetLife(params.fadingTau);
            (*itPoisson).SetSpawn(params.fadingTau);
        }
    }

    this->poissondisk += (std::clock() - addPoissonDisk) / (double) CLOCKS_PER_SEC;
    return PPoints;
}


//================================================================================================

//                                       ADD PATCHES USING BARYCENTRIC COORDONATE

//================================================================================================


void Yu2011::AddPatchesUsingBarycentricCoordinates(GU_Detail *deformableGridsGdp,GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params,  vector<GA_Offset> trackers, GEO_PointTreeGAOffset &surfaceTree,  GU_RayIntersect &ray)
{

    std::clock_t addPatchesStart;
    addPatchesStart = std::clock();


    float beta = 0.6f;
    float d = params.poissondiskradius;
    float gridwidth = (2+beta)*d; //same formula used in DeformableGrids.cpp
    fpreal patchRadius = (fpreal)gridwidth;

    //================================ CREATE PATCH GROUPS ==============================
    GA_PointGroup *grpMarker = (GA_PointGroup *)trackersGdp->pointGroups().find(this->markerGroupName.c_str());
    if (grpMarker->entries() == 0)
    {
        cout << this->approachName << " tracker group is empty"<<endl;
        return;
    }

    GA_GroupType primitiveGroupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *primitiveGTable = deformableGridsGdp->getGroupTable(primitiveGroupType);

    GA_RWHandleV3 attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleV3 attUV(deformableGridsGdp->findFloatTuple(GA_ATTRIB_POINT,uvName, 3));
    GA_RWHandleF attAlpha(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"Alpha",1));
    GA_RWHandleV3 attNSurface(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));

    float thresholdDistance = params.maximumProjectionDistance;

    GA_Offset ppt;
    UT_Vector3 N;
    UT_Vector3 NN;
    UT_Vector3 position;
    UT_Vector3 patchP;

    GA_Offset surfacePointOffset;
    int patchNumber = 0;
    vector<GA_Offset>::iterator it;
    for(it = trackers.begin(); it != trackers.end(); ++it)
    {
        ppt = *it;
        patchNumber = attId.get(ppt);
        if (params.testPatch == 1 && params.patchNumber != patchNumber)
            continue;
        N = attN.get(ppt);
        position = trackersGdp->getPos3(ppt);

        UT_IntArray         patchArrayData;
        UT_FloatArray         alphaArrayData;
        UT_FloatArray         uvArrayData;
        //getting neigborhood
        // Close particles indices
        GEO_PointTreeGAOffset::IdxArrayType surfaceNeighborhoodVertices;
        surfaceTree.findAllCloseIdx(position,
                             patchRadius,
                             surfaceNeighborhoodVertices);

        unsigned close_particles_count = surfaceNeighborhoodVertices.entries();

        string str = std::to_string(patchNumber);
        UT_String patchGroupName("patch"+str);
        GA_PointGroup* patchGroup = surfaceGdp->newPointGroup(patchGroupName, 0);

        UT_String gridGroupName("grid"+str);
        GA_PrimitiveGroup*  gridPrimitiveGroup  = (GA_PrimitiveGroup*)primitiveGTable->find(gridGroupName);
        if (gridPrimitiveGroup == 0x0)
            continue;

        ray.init(deformableGridsGdp,gridPrimitiveGroup);

        set<GA_Offset> neighborhood;
        for(int j=0; j<close_particles_count;j++ )
        {
            surfacePointOffset = surfaceNeighborhoodVertices.array()[j];
            neighborhood.insert(surfacePointOffset);
        }

        set<GA_Offset>::iterator itG;
        for(itG = neighborhood.begin(); itG != neighborhood.end(); ++itG)
        {
            surfacePointOffset = *itG;
            NN = attNSurface.get(surfacePointOffset);
            float dotP = dot(N,NN); //exlude points that are not in the same plane.
            if (dotP < params.angleNormalThreshold)
                continue;
            patchP = surfaceGdp->getPos3(surfacePointOffset);
            //------------------------------------ RAY -----------------------------------------
            //project patchP on trackers set
            GU_MinInfo mininfo;
            mininfo.init(thresholdDistance,0.0001);
            ray.minimumPoint(patchP,mininfo);
            if (mininfo.prim == 0x0)
            {
                //we can't project the point on the surface
                continue;
            }
            //get pos of hit
            UT_Vector4 hitPos;
            mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
            if (distance3d(hitPos,patchP) > thresholdDistance)
                continue;
            float u = mininfo.u1;
            float v = mininfo.v1;

            //------------------------------PARAMETRIC COORDINATE -----------------------------------
            GA_Offset primOffset = mininfo.prim->getMapOffset();
            GEO_Primitive *prim = deformableGridsGdp->getGEOPrimitive(primOffset);
            if (prim->getVertexCount() < 3)
                continue;

            GA_Offset vertexOffset0 = prim->getVertexOffset(0);
            GA_Offset vertexOffset1 = prim->getVertexOffset(1);
            GA_Offset vertexOffset2 = prim->getVertexOffset(2);

            GA_Offset pointOffset0  = deformableGridsGdp->vertexPoint(vertexOffset0);
            GA_Offset pointOffset1  = deformableGridsGdp->vertexPoint(vertexOffset1);
            GA_Offset pointOffset2  = deformableGridsGdp->vertexPoint(vertexOffset2);

            UT_Vector3 v0   = attUV.get(pointOffset0);
            UT_Vector3 v1   = attUV.get(pointOffset1);
            UT_Vector3 v2   = attUV.get(pointOffset2);

            float a0        = attAlpha.get(pointOffset0);
            float a1        = attAlpha.get(pointOffset1);
            float a2        = attAlpha.get(pointOffset2);

            UT_Vector3 uvPatch = v0+u*(v1-v0)+v*(v2-v0);
            float   alphaPatch = a0+u*(a1-a0)+v*(a2-a0);

            if (alphaPatch > 1)
                alphaPatch = 1;
            else if (alphaPatch < 0)
                alphaPatch = 0; //this is weird, should not happen

            //------------------------------- SAVE TO VERTEX ----------------------------------------
            patchGroup->addOffset(surfacePointOffset);
            // Fetch array value
            patchIdsAtt->get(patchIdsArrayAttrib,surfacePointOffset, patchArrayData);
            patchArrayData.append(patchNumber);
            // Write back
            patchIdsAtt->set(patchIdsArrayAttrib,surfacePointOffset, patchArrayData);

            alphaAtt->get(alphaArrayAtt, surfacePointOffset, alphaArrayData);
            alphaArrayData.append(alphaPatch);
            // Write back
            alphaAtt->set(alphaArrayAtt, surfacePointOffset, alphaArrayData);

            uvsArray->get(uvsAtt, surfacePointOffset, uvArrayData);
            uvArrayData.append(uvPatch.x());
            uvArrayData.append(uvPatch.y());
            uvArrayData.append(uvPatch.z());
            // Write back
            uvsArray->set(uvsAtt, surfacePointOffset, uvArrayData);
            //---------------------------------------------------------------------------------
        }
        neighborhood.clear();
    }

    for(it = trackers.begin(); it != trackers.end(); ++it)
    {
        ppt = *it;
        patchNumber = attId.get(ppt);

        //for test purposes
        if (params.testPatch == 1 && params.patchNumber != patchNumber)
            continue;

        string str = std::to_string(patchNumber);
        UT_String pointGroupName("patch"+str);
        GA_PrimitiveGroup* primGrp = surfaceGdp->newPrimitiveGroup(pointGroupName);
        GA_GroupType groupType = GA_GROUP_POINT;
        const GA_GroupTable *gtable = surfaceGdp->getGroupTable(groupType);
        GA_OffsetArray primitives;
        GA_PointGroup* pointGrp = (GA_PointGroup*)gtable->find(primGrp->getName());
        GA_FOR_ALL_GROUP_PTOFF(surfaceGdp,pointGrp,ppt)
        {
            surfaceGdp->getPrimitivesReferencingPoint(primitives,ppt);
            for(GA_OffsetArray::const_iterator prims_it = primitives.begin(); !prims_it.atEnd(); ++prims_it)
            {
                primGrp->addOffset(*prims_it);
            }
        }
    }
    this->patchCreationTime += (std::clock() - addPatchesStart) / (double) CLOCKS_PER_SEC;
}



//======================================================================================================================================
//                                                  UpdateUsingBridson2012PoissonDisk
//======================================================================================================================================

void Yu2011::UpdateDistributionUsingBridson2012PoissonDisk(GU_Detail *gdp,GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params,  GEO_PointTreeGAOffset &surfaceTree, GU_RayIntersect &ray)
{
    cout << this->approachName<<" Update Using Bridson 2012 Poisson Disk with "<<numberOfPatches<<" existing trackers"<<endl;
    std::clock_t startUpdatePatches;
    startUpdatePatches = std::clock();

    //--------------------------------------------------------------------------
    GA_RWHandleF attSumAlpha(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"SumAlpha", 1));
    GA_RWHandleI attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleF attLife(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleI attSpawn(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"spawn",1));
    GA_RWHandleI attActive(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"active", 1));
    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;
    GA_PointGroup *markerGroup = (GA_PointGroup *)trackersGdp->pointGroups().find(this->markerGroupName.c_str());

    GA_Offset ppt;
    int beforeAddingNumber = numberOfPatches;
    //--------------------------- ADDING A NEW PATCH ON NEWLY ADDED POISSON DISK --------------------------------------------
    {
        GA_FOR_ALL_GROUP_PTOFF(trackersGdp,markerGroup,ppt)
        {   
            int spawn = attSpawn.get(ppt);
            int active = attActive.get(ppt);
            if (active == 1 && spawn == 1)
            {
                vector<GA_Offset> newPatchPoints;
                vector<GA_Offset> newTrackers;
                newTrackers.push_back(ppt);
                newPatchPoints.push_back(ppt);
                CreateGridBasedOnMesh(gdp,surfaceGdp,trackersGdp, params,newTrackers,surfaceTree);
                AddPatchesUsingBarycentricCoordinates(gdp,surfaceGdp,trackersGdp,params,newTrackers, surfaceTree, ray);
                numberOfPatches++;
            }
            //we compute update excluding the time for adding a new patc since it is already computed inside these functions
            startUpdatePatches = std::clock();
            this->updatePatchesTime += (std::clock() - startUpdatePatches) / (double) CLOCKS_PER_SEC;
        }
    }

    GA_PointGroup *grpToDestroy = (GA_PointGroup *)trackersGdp->newPointGroup("ToDelete");
    set<int> toDelete;
    //--------------------------- DELETE DEAD PATCH --------------------------------------------
    {
        GA_FOR_ALL_GROUP_PTOFF(trackersGdp,markerGroup,ppt)
        {
            int id = attId.get(ppt);
            float life = attLife.get(ppt);
            int active = attActive.get(ppt);
            if (active == 0 && life <= 0.0f && params.frame != params.startFrame)
            {
                toDelete.insert(id);
                numberOfPatches--;
                numberOfConcealedPatches++;
            }
        }
    }
    {
        GA_FOR_ALL_GROUP_PTOFF(trackersGdp,markerGroup,ppt)
        {
            int id = attId.get(ppt);
            if (toDelete.count(id) > 0)
            {
                grpToDestroy->addOffset(ppt);
            }
        }
    }
    //destroying trackers
    trackersGdp->deletePoints(*grpToDestroy,mode);
    trackersGdp->destroyPointGroup(grpToDestroy);

    cout <<this->approachName<< " Added "<<(numberOfPatches-beforeAddingNumber) <<" new patches"<<endl;
    cout <<this->approachName<< " Removed "<<(numberOfConcealedPatches)<<" patches "<<endl;
    this->updatePatchesTime += (std::clock() - startUpdatePatches) / (double) CLOCKS_PER_SEC;
    cout << this->approachName<<" TOTAL "<<numberOfPatches<< " patches"<<endl;
}
