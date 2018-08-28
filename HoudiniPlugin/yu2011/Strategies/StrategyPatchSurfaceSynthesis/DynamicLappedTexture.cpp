#include "DynamicLappedTexture.h"

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

#include <Strategies/StrategySurfaceTextureSynthesis.h>
#include <Core/HoudiniUtils.h>


DynamicLappedTexture::DynamicLappedTexture(GU_Detail *surface, GU_Detail *trackers)
{
    this->numberOfPatches = 0;
    this->maxId = 0;

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

}

bool DynamicLappedTexture::SynthesisSurface(GU_Detail *gdp, ParametersDeformablePatches params)
{
    return true;

}





//================================================================================================

//                                       ADD DEFORMABLE PATCHES

//================================================================================================

void DynamicLappedTexture::AddPatches(GU_Detail *gdp, GU_Detail *surfaceGdp, GU_Detail *trackersGdp, int startNumber, ParametersDeformablePatches params ,  vector<GA_Offset> trackers, GEO_PointTreeGAOffset &tree)
{

    //cout << this->approachName<<" Add Patches"<<endl;

    float radius = params.poissondiskradius;
    float patchRadius = radius*1.5;

    //vector<UT_Vector3>::iterator it;
    int i = numberOfPatches;
    int patchNumber = 0;

    UT_Vector3 position;
    GA_PointGroup *grp = (GA_PointGroup *)gdp->pointGroups().find(this->markerGroupName.c_str());
    GA_RWHandleV3 attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI attM(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"M",1));
    GA_RWHandleI attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));

    GA_RWHandleV3 attNSurface(surfaceGdp->findFloatTuple(GA_ATTRIB_POINT,"N", 3));

    GA_Offset ppt;
    UT_Vector3 N;
    int id;
    UT_Vector3 NN;


    vector<GA_Offset>::iterator it;

    GA_Offset neighbor;

    for(it = trackers.begin(); it != trackers.end(); ++it)
    //GA_FOR_ALL_GROUP_PTOFF(gdp,grp,ppt)
    {
        ppt = *it;
        //don't take tangent marker into account
        if(attM.get(ppt)==0)
            continue;

        //cout << "Add patch for tracker "<<ppt<<endl;

        position = trackersGdp->getPos3(ppt);
        N = attN.get(ppt);
        id = attId.get(ppt);
        //patchNumber = i+startNumber;
        patchNumber = id;

        UT_IntArray         patchArrayData;
        UT_FloatArray         alphaArrayData;
        UT_FloatArray         uvArrayData;

        //getting neigborhood
        // Close particles indices
        GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;
        tree.findAllCloseIdx(position,
                             patchRadius,
                             close_particles_indices);


        GA_Offset closestPoint = tree.findNearestIdx(position);

        unsigned close_particles_count = close_particles_indices.entries();
        if (close_particles_count > 0)
        {
            //cout << "found "<<close_particles_count<< " particles"<<endl;
            //string str = std::to_string(patchNumber);
            string str = std::to_string(id);

            UT_String pointGroupName("patch"+str);
            GA_PointGroup* patchGroup = surfaceGdp->newPointGroup(pointGroupName, 0);
            //cout << "Create group "<<pointGroupName<<endl;
            set<GA_Offset> group;

            if (params.connectivityTest)
                ConnectivityTest(gdp,closestPoint,grp,close_particles_indices,group);
            else
            {
                for(int j=0; j<close_particles_count;j++ )
                {
                    neighbor = close_particles_indices.array()[j];
                    group.insert(neighbor);
                }
            }

            set<GA_Offset>::iterator itG;
            for(itG = group.begin(); itG != group.end(); ++itG)
            //for(int j=0; j<close_particles_count;j++ )
            {
                //cout << "add offset "<<close_particles_indices[j]<< " to group "<<pointGroupName<<endl;

                //neighbor = close_particles_indices[j];
                neighbor = *itG;
                //if (grp->containsOffset(neighbor)) // do this test in connectivity test
                //    continue;

                NN = attNSurface.get(neighbor);
                float dotP = dot(N,NN);
                if (dotP < 0.01)
                    continue;

                float minT = 0.5;
                float dotProd = dot(N,NN);

                float alpha = (dotProd/minT)-0.5;
                if (alpha > 1)
                    alpha = 1;

                patchGroup->addOffset(neighbor);
                // Fetch array value
                patchIdsAtt->get(patchIdsArrayAttrib,neighbor, patchArrayData);
                patchArrayData.append(patchNumber);
                // Write back
                patchIdsAtt->set(patchIdsArrayAttrib,neighbor, patchArrayData);

                alphaAtt->get(alphaArrayAtt, neighbor, alphaArrayData);
                alphaArrayData.append(alpha);
                // Write back
                alphaAtt->set(alphaArrayAtt, neighbor, alphaArrayData);

                uvsArray->get(uvsAtt, neighbor, uvArrayData);

                uvArrayData.append(0.0f);
                uvArrayData.append(0.0f);
                uvArrayData.append(0.0f);

                // Write back
                uvsArray->set(uvsAtt, neighbor, uvArrayData);

                //attId.set(neighbor,id);

            }

            group.clear();

        }
        i++;
    }

    //----------------------------------------------------------------------
    //write in de the detail view the number of pathes.
    GA_RWHandleI numberOfPatchesAtt(gdp->addIntTuple(GA_ATTRIB_DETAIL,"numberOfPatches", 1));
    numberOfPatchesAtt.set(GA_Offset(0),startNumber+i);
    //----------------------------------------------------------------------

    // Mark as modified.
    alphaArrayAtt->bumpDataId();

    i = numberOfPatches;
    for(it = trackers.begin(); it != trackers.end(); ++it)
    {

        if (attM.get(*it) == 0)
            continue;
        //string groupName = "patch"+*it;
        //patchNumber = i+startNumber;
        patchNumber = attId.get(*it);
        string str = std::to_string(patchNumber);
        UT_String pointGroupName("patch"+str);
        //cout << "creating primitive group "<<pointGroupName<<endl;
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
        i++;
    }

    numberOfPatches += trackers.size()/2;

    //referenceGdp->clear();

}

//================================================================================================

//                                       PROJECT UVS

//================================================================================================

void DynamicLappedTexture::ProjectUVSForSelectedGroups(GU_Detail *surface, GU_Detail *trackersGdp, ParametersDeformablePatches params, vector<GA_Offset> trackers)
{

    //cout << this->approachName<<" Project UV For Selected Groups"<<endl;
    GA_PointGroup *grp;

    /*
    //=========================== UV ARRAY ATTRIB ==========================
    UT_String aname(this->uvArrayName);
    GA_Attribute        *uvsAtt = gdp->findFloatArray(GA_ATTRIB_POINT,
                                            aname,
                                            // Allow any tuple size to match
                                            -1, -1);

    if (!uvsAtt)
    {
        uvsAtt = gdp->addFloatArray(GA_ATTRIB_POINT,
                                    aname,
                    // Tuple size one means we group the array into
                    // logical groups of 1.  It does *NOT* affect
                    // the length of the arrays, which are always
                    // measured in ints.
                                    1);
    }

    const GA_AIFNumericArray *uvsArray = uvsAtt->getAIFNumericArray();
    if (!uvsArray)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) aname);
        //addError(SOP_MESSAGE, buf.buffer());
        //return error();
    }
    */
    /*



    vector<GA_Offset> markers;
    //GA_FOR_ALL_POINTGROUPS(gdp,grp)
    {
        GA_Offset ppt;
        GA_FOR_ALL_GROUP_PTOFF(gdp,grpMarker,ppt)
        {
            markers.push_back(ppt);
        }
    }
    */

    GA_RWHandleI attM(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"M",1));
    GA_RWHandleI attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));

    GA_GroupType groupType = GA_GROUP_POINT;
    const GA_GroupTable *gtable = surface->getGroupTable(groupType);

    int i = this->numberOfPatches-trackers.size()/2; //trackers are along side with tangent trackers

    //cout << "Projeting uv for "<<i<<" patches"<<endl;

    int id;
    GA_Offset ppt;
    vector<GA_Offset>::iterator it;
    for(it = trackers.begin(); it != trackers.end(); ++it)
    //GA_FOR_ALL_POINTGROUPS(gdp,grp)
    {
        ppt = *it;
        if (attM.get(ppt) == 0)
            continue;

        vector<GA_Offset> localFrame;
        localFrame.push_back(ppt);
        localFrame.push_back(ppt+1);

        id = attId.get(ppt);

        string str = std::to_string(id);
        UT_String pointGroupName("patch"+str);
        grp = (GA_PointGroup*)gtable->find(pointGroupName);
        //ProjectUVFlattenForPatch(gdp,grp,patchIdsArrayAttrib);
        ProjectUVOrthogonalForPatch(surface,trackersGdp,grp,uvsArray, uvsAtt, localFrame,i, params);
        i++;
    }
}

//================================================================================================

//                                       UPDATE PATCHES

//================================================================================================

void DynamicLappedTexture::UpdatePatches(GU_Detail *gdp,GU_Detail *surface, GU_Detail *trackers,GA_PointGroup *surfaceGroup, ParametersDeformablePatches params, int currentFrame, GEO_PointTreeGAOffset &tree)
{

    cout << this->approachName<<" Updates patches"<<endl;


    //--------------------------------------------------------------------------
    if (params.innerCircleRadius < 0.1)
        params.innerCircleRadius = 1;
    UT_Vector3 centerUvPosition = UT_Vector3(0.5,0.5,0);


    GA_RWHandleF attSumAlpha(surface->addFloatTuple(GA_ATTRIB_POINT,"Alpha", 1));
    GA_RWHandleV3 attCd(surface->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));

    GA_RWHandleI attM(trackers->findIntTuple(GA_ATTRIB_POINT,"M",1));
    GA_RWHandleI attId(trackers->addIntTuple(GA_ATTRIB_POINT,"id",1));

    GA_Offset ppt;
    UT_Vector3 uv;
    int uvIndex = 0 ;

    int beforeAddingNumber = numberOfPatches;

    patchesUsed.clear();
    //set<int> patchesUsed;
    {
        GA_FOR_ALL_PTOFF(surface,ppt)
        {
            UT_IntArray         patchesData;
            patchIdsAtt->get(patchIdsArrayAttrib, ppt, patchesData);

            UT_FloatArray         alphasData;
            alphaAtt->get(alphaArrayAtt, ppt, alphasData);

            UT_FloatArray         uvsData;
            uvsArray->get(uvsAtt, ppt, uvsData);

            int nb = patchesData.size();

            float sumAlpha = 0;
            int nbAlpha = 0;

            //put patches in a ordered list
            set<int> orderedList;
            vector<float> alphas;
            vector<UT_Vector3> uvs;
            for (int i = 0; i< nb; i++)
            {
                int patchId = patchesData.array()[i];

                //if (patchId == 1531)
                //    cout << "w00t"<<endl;
                orderedList.insert(patchId);

                float alpha = alphasData.array()[i];
                alphas.push_back(alpha);

                uvIndex = i*3;
                uv = UT_Vector3(uvsData.array()[uvIndex],uvsData.array()[uvIndex+1],uvsData.array()[uvIndex+2]);
                uvs.push_back(uv);
            }

            set<int>::iterator it;
            int i = 0;
            for (it = orderedList.begin(); it != orderedList.end(); ++it)
            {
                int patchId = *it;
                float alpha = alphas[i];

                uv = uvs[i];
                uv.z() = 0; //don't know why, but something it's not equal to zero and it's influencing the distance
                float dist = distance3d(uv,centerUvPosition);
                if ( dist > params.innerCircleRadius )
                {
                    alpha = 0;
                }
                else if (alpha > 0)
                {
                    patchesUsed.insert(patchId);
                }
                sumAlpha += alpha;
                nbAlpha++;

                if (sumAlpha >= 1 )
                    break;
                i++;

            }
            attSumAlpha.set(ppt,sumAlpha);
            attCd.set(ppt,UT_Vector3(0,0,1));
            float threshold = params.alphaThreshold;

            if (!params.updateDistribution)
                continue;
            //--------------------------- ADDING A NEW PATCH ! --------------------------------------------
            if(sumAlpha < threshold)
            {
                //cout <<"new patch: There was "<<nb<< " patch on vertex "<<ppt << " with sum alpha "<<sumAlpha<<endl;


                vector<GA_Offset> newPatchPoints;
                vector<GA_Offset> newTrackers;
                newPatchPoints.push_back(ppt);
                //cout << "Adding new Patch "<<numberOfPatches<< " on point "<<ppt;
                newTrackers = CreateTrackers(surface,trackers,surfaceGroup,params,newPatchPoints);
                AddPatches(gdp,surface, trackers, params.startNumber,params,newTrackers, tree);
                ProjectUVSForSelectedGroups(surface,trackers,params,newTrackers);
                patchesUsed.insert(maxId);

                attCd.set(ppt,UT_Vector3(1,0,0));
            }
        }
    }

    cout << "Added "<<(numberOfPatches-beforeAddingNumber) <<" new patches !"<<endl;

    //-------------------------------- REMOVE CONCEALED PATCHES ---------------------------
    if (params.deleteConcealedPatches)
    {
        cout << "Delete concealed patches"<<endl;
        int beforeConcealedNumber = numberOfPatches;

        GA_PointGroup *grpToDestroy = (GA_PointGroup *)trackers->newPointGroup("ToDelete");

        set<int> toDelete;

        //delete consealed patches

        {
            GA_FOR_ALL_PTOFF(trackers,ppt)
            {
                if (attM.get(ppt) == 1)
                {
                    int id = attId.get(ppt);
                    if (patchesUsed.count(id) == 0)
                    {
                        //cout << "Patch "<<patchIndex/2<< " is concealed"<<endl;
                        toDelete.insert(id);

                        numberOfPatches--;
                        //toAdd = false;
                    }
                    //else
                        //toAdd = true;
                    //patchIndex++;
                }
                //if(!toAdd)
               //    grpToDestroy->addOffset(ppt);
            }
        }
        {
            GA_FOR_ALL_PTOFF(trackers,ppt)
            {
                int id = attId.get(ppt);
                if (toDelete.count(id) > 0)
                {
                    grpToDestroy->addOffset(ppt);
                }
            }
        }

        //destroying trackers
        GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;
        trackers->deletePoints(*grpToDestroy,mode);
        trackers->destroyPointGroup(grpToDestroy);

        /*
        //destroying the rest
        set<int>::iterator it;
        for(it = toDelete.begin(); it != toDelete.end(); ++it)
        {
            string str = std::to_string(*it);
            UT_String pointGroupName("patch"+str);
            grpToDestroy = (GA_PointGroup*)gtable->find(pointGroupName);

            gdp->deletePoints(*grpToDestroy,mode);
            gdp->destroyPointGroup(grpToDestroy);
            numberOfPatches--;
        }

        */

        cout << "Removing "<<(toDelete.size())<<" concealed "<<endl;
    }

    //--------------------------------------------------------------------------------------

    cout << "TOTAL "<<numberOfPatches<< " patches"<<endl;
}

//================================================================================================

//                                       COMPUTE DISTORTION

//================================================================================================

void DynamicLappedTexture::ComputeDistortion(GU_Detail *gdp, ParametersDeformablePatches params)
{

    //DistortionManager::ComputeDistortionBasedOnEdges(gdp);
    //DistortionManager::ComputeDistortionBasedOnAreas(gdp);

}


//================================================================================================

//                                       Project UV Orthogonal For Patches

//================================================================================================

void DynamicLappedTexture::ProjectUVOrthogonalForPatch(GU_Detail *surface, GU_Detail *trackers, GA_PointGroup *patchGroupToAdd, const GA_AIFNumericArray *uvsArray, GA_Attribute *uvAtt, vector<GA_Offset> markers, int index,ParametersDeformablePatches params)
{

    //cout << "Project UV for patches"<<endl;
    GA_GroupType groupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *gtable = surface->getGroupTable(groupType);

    if (patchGroupToAdd == 0x0)
        return;

    float scaling = (params.poissondiskradius*2);
    GA_RWHandleV3 attN(trackers->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    //cout << "group "<<grp->getName()<<endl;
    string groupString = patchGroupToAdd->getName().toStdString();
    if(groupString == "markers")
        return;
    groupString.replace(0,5,"");

    int patchId = std::stoi( groupString );

    //get the marker for this patch number

    GA_Offset tracker = markers[0]; // In memory, we have the tracker and the tangent tracker side by side
    GA_Offset tangentTracker = markers[1];
    UT_Vector3 pt = trackers->getPos3(tangentTracker);

    UT_Vector3 p = trackers->getPos3(tracker);
    UT_Vector3 N = attN.get(tracker);
    N.normalize();
    UT_Vector3 T = pt-p;
    T.normalize();
    UT_Vector3 S = cross(N,T);
    S.normalize();

    GEO_Primitive *prim;

    //----------------------------------------------------------------------
    //put back the uvs to the main gdp

    int i = 0;
    int primCounter = 0;
    GA_Offset vertexOffset;
    GA_PrimitiveGroup* selectedPrimGroup = (GA_PrimitiveGroup*)gtable->find(patchGroupToAdd->getName());

    GA_Offset pointOffset;
    GA_FOR_ALL_GROUP_PRIMITIVES(surface, selectedPrimGroup, prim)
    {
        //cout << "Prim "<<prim->getNum();
        int nbrVertex = prim->getVertexCount();
        //if (primCounter >= primGrpCopy->entries())
        //    break;
        for(int j= 0; j< nbrVertex; j++)
        {

            UT_FloatArray         fdata;
            UT_IntArray patchArrayData;

            vertexOffset = prim->getVertexOffset(j);
            pointOffset = surface->vertexPoint(vertexOffset);

            //----------------------- UV PROJECTION --------------
            UT_Vector3 val = surface->getPos3(pointOffset);

            // Transform into local patch space (where STN is aligned with XYZ at the origin)
            const UT_Vector3 relativePosistion = val-p;
            UT_Vector3 triangleSpacePos;
            triangleSpacePos.x() = relativePosistion.dot(S);
            triangleSpacePos.y() = relativePosistion.dot(T);
            triangleSpacePos.z() = relativePosistion.dot(N);

            // Fetch array value
            patchIdsAtt->get(patchIdsArrayAttrib, pointOffset, patchArrayData);
            int nb = patchArrayData.size();
            int index = -1;
            for (int i = 0; i< nb; i++)
            {
                if (patchArrayData.array()[i] == patchId)
                    index = i;
            }

            UT_Vector3 uv;
            uv.x() = triangleSpacePos.x();
            uv.y() = triangleSpacePos.y();
            uv.z() = 0;

            float mid = 0.5;
            uv /= scaling;
            uv += mid;
            //uv.z should be zero
            //----------------------------------------------------

            i++;


            // Fetch array value
            uvsArray->get(uvAtt, pointOffset, fdata);

            if (index == -1)
                continue;

            fdata.insertAt(uv.x(), (index*3)+0);
            fdata.insertAt(uv.y(), (index*3)+1);
            fdata.insertAt(uv.z(), (index*3)+2);

            // Write back
            uvsArray->set(uvAtt, pointOffset, fdata);

            //--------------------------------------------------------------------------------
        }
        primCounter++;
        //cout << endl;
    }
    //----------------------------------------------------------------------
}

GA_Offset DynamicLappedTexture::GetNearestPoint(GEO_PointTreeGAOffset &tree, UT_Vector3 &pos, float maxdist)
{

    // Get the offset of the nearest point, within maxdist.
    return tree.findNearestIdx(pos, maxdist);

}


void DynamicLappedTexture::ConnectivityTest(const GU_Detail *gdp,GA_Offset point,GA_PointGroup *grp, GEO_PointTreeGAOffset::IdxArrayType &pointsAround,set<GA_Offset> &group)
{

    GA_OffsetArray connectedTriangle;
    gdp->getPrimitivesReferencingPoint(connectedTriangle,point);
    GA_OffsetArray::const_iterator it;

    for(it = connectedTriangle.begin(); it != connectedTriangle.end(); ++it)
    {
        GA_Offset index = (*it);
        const GA_Primitive *prim = gdp->getPrimitive(index);
        //triangleGroup.insert(index);
        GA_Size vertexCount = prim->getVertexCount();

        for(int i = 0; i < vertexCount; i++)
        {
            GA_Offset vertexPoint = prim->getVertexOffset(i);
            GA_Offset pointOffset = gdp->vertexPoint(vertexPoint);

            if (grp->containsOffset(pointOffset))
                continue;

            int found = pointsAround.find(pointOffset);
            if(found != -1 && group.count(pointOffset) == 0)
            {
                group.insert(pointOffset);
                ConnectivityTest(gdp,pointOffset,grp, pointsAround,group);
            }
        }
    }
}


GEO_PointTreeGAOffset* DynamicLappedTexture::CreateSurfaceTree(GU_Detail *gdp, ParametersDeformablePatches params)
{
    GU_Detail *referenceGdp = new GU_Detail();
    referenceGdp->clear();
    referenceGdp->copy(*gdp);
    GA_PointGroup *grpToDestroy = (GA_PointGroup *)referenceGdp->pointGroups().find(this->markerGroupName.c_str());
    GA_PrimitiveGroup *primGrpToDestroy = (GA_PrimitiveGroup *)referenceGdp->primitiveGroups().find(this->markerGroupName.c_str());

    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;
    referenceGdp->deletePoints(*grpToDestroy,mode);
    referenceGdp->destroyPointGroup(grpToDestroy);
    referenceGdp->destroyPrimitiveGroup(primGrpToDestroy);

    //============================================================

    //GA_RWHandleV3 refAttV(referenceGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    //GA_RWHandleV3 refAttN(referenceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    //GA_RWHandleV3 AttCd(gdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));

    float Tlenght = params.tangentTrackerLenght;

    GEO_PointTreeGAOffset tree;
    tree.build(referenceGdp, NULL);
    return &tree;
}
