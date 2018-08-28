#include "DeformablePatches.h"
#include "DeformablePatches.h"

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
#include <GU/GU_NeighbourList.h>
#include "Core/Deformations/ParametersDeformablePatches.h"
#include <GU/GU_Flatten.h>

#include <Strategies/StrategySurfaceTextureSynthesis.h>
#include "Core/Deformations/DistortionManager.h"


/*
DeformablePatches::DeformablePatches()
{

}

DeformablePatches::~DeformablePatches()
{

}
*/

bool DeformablePatches::SynthesisSurface(GU_Detail *gdp, ParametersDeformablePatches params)
{
    return true;

}



//================================================================================================

//                                      POISSON DISK DISTRIBUTION

//================================================================================================

vector<UT_Vector3>  DeformablePatches::PoissonDiskDistribution(GU_Detail *gdp,ParametersDeformablePatches params )
{

    float radius = params.poissondiskradius;

    cout << "Scatter points and fuse with radius "<<radius<<endl;

    GU_Detail *referenceGdp = new GU_Detail();
    referenceGdp->clear();
    referenceGdp->copy(*gdp);

    referenceGdp->consolidatePoints(radius);

    vector<UT_Vector3> points;

    GEO_Primitive *prim;
    GA_FOR_ALL_PRIMITIVES(referenceGdp, prim)
    {
        referenceGdp->deletePrimitive(*prim);
    }

    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(referenceGdp,ppt)
    {
        points.push_back(referenceGdp->getPos3(ppt));
    }

    return points;
}

//================================================================================================

//                                       ADD DEFORMABLE PATCHES

//================================================================================================

void DeformablePatches::AddPatches(GU_Detail *gdp, vector<UT_Vector3> points, int startNumber, ParametersDeformablePatches params )
{

    float radius = params.poissondiskradius;

    //=========================== PATCH ID ARRAY ATTRIB ==========================
    UT_String aname("patchIds");
    GA_Attribute        *patchIdsArrayAttrib = gdp->findIntArray(GA_ATTRIB_POINT,
                                            aname,
                                            // Allow any tuple size to match
                                            -1, -1);
    if (!patchIdsArrayAttrib)
    {
        patchIdsArrayAttrib = gdp->addIntArray(GA_ATTRIB_POINT,
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
    const GA_AIFNumericArray *aif = patchIdsArrayAttrib->getAIFNumericArray();
    if (!aif)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) aname);
        //addError(SOP_MESSAGE, buf.buffer());
        //return error();
    }

    //================================ CREATE PATCH GROUPS ==============================

    float patchRadius = radius*2;

    GEO_PointTreeGAOffset tree;
    tree.build(gdp, NULL);
    vector<UT_Vector3>::iterator it;
    int i = 0;
    int patchNumber = 0;
    for(it = points.begin(); it != points.end();++it)
    {

        patchNumber = i+startNumber;

        UT_IntArray         data;

        //getting neigborhood
        // Close particles indices
        GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;
        tree.findAllCloseIdx(*it,
                             patchRadius,
                             close_particles_indices);

        unsigned close_particles_count = close_particles_indices.entries();
        if (close_particles_count > 0)
        {
            //cout << "found "<<close_particles_count<< " particles"<<endl;
            string str = std::to_string(patchNumber);
            UT_String pointGroupName("patch"+str);
            GA_PointGroup* patchGroup = gdp->newPointGroup(pointGroupName, 0);

            for(int j=0; j<close_particles_count;j++ )
            {
                //cout << "add offset "<<close_particles_indices[j]<< " to group "<<pointGroupName<<endl;
                patchGroup->addOffset(close_particles_indices.array()[j]);
                // Fetch array value
                aif->get(patchIdsArrayAttrib, close_particles_indices.array()[j], data);
                data.append(patchNumber);
                // Write back
                aif->set(patchIdsArrayAttrib, close_particles_indices.array()[j], data);

                string alphaname = "Alpha"+std::to_string(patchNumber);
                //WARNING: I think this part is not working properly
                GA_RWHandleF attAlpha(gdp->addFloatTuple(GA_ATTRIB_POINT,alphaname.c_str(), 1));
                //if (attAlpha.isValid())
                attAlpha.set(close_particles_indices.array()[j],1.0f);

            }
        }

        i++;
    }

    //----------------------------------------------------------------------
    //write in de the detail view the number of pathes.
    GA_RWHandleI numberOfPatchesAtt(gdp->addIntTuple(GA_ATTRIB_DETAIL,"numberOfPatches", 1));
    numberOfPatchesAtt.set(GA_Offset(0),startNumber+i);
    //----------------------------------------------------------------------

    // Mark as modified.
    patchIdsArrayAttrib->bumpDataId();

    //create primitives group
    GA_PointGroup *grp;
    GA_FOR_ALL_POINTGROUPS(gdp,grp)
    {
        GA_PrimitiveGroup* primGrp = gdp->newPrimitiveGroup(grp->getName());
        GA_Offset ppt;

        //cout << "group "<<grp->getName()<<endl;
        GA_OffsetArray primitives;

        GA_FOR_ALL_GROUP_PTOFF(gdp,grp,ppt)
        {
            gdp->getPrimitivesReferencingPoint(primitives,ppt);
            for(GA_OffsetArray::const_iterator prims_it = primitives.begin(); !prims_it.atEnd(); ++prims_it)
            {
                primGrp->addOffset(*prims_it);
            }
        }
    }

}

//================================================================================================

//                                       PROJECT UVS

//================================================================================================

void DeformablePatches::ProjectUVSForSelectedGroups(GU_Detail *gdp,ParametersDeformablePatches params )
{

    GA_PointGroup *grp;

    //=========================== UV ARRAY ATTRIB ==========================
    UT_String aname("uvs");
    GA_Attribute        *patchIdsArrayAttrib = gdp->findFloatArray(GA_ATTRIB_VERTEX,
                                            aname,
                                            // Allow any tuple size to match
                                            -1, -1);
    if (!patchIdsArrayAttrib)
    {
        patchIdsArrayAttrib = gdp->addFloatArray(GA_ATTRIB_VERTEX,
                                    aname,
                    // Tuple size one means we group the array into
                    // logical groups of 1.  It does *NOT* affect
                    // the length of the arrays, which are always
                    // measured in ints.
                                    1);
    }


    GA_FOR_ALL_POINTGROUPS(gdp,grp)
    {
        ProjectUVForPatch(gdp,grp,patchIdsArrayAttrib);
    }
}

//================================================================================================

//                                       UPDATE PATCHES

//================================================================================================

void DeformablePatches::UpdatePatches(GU_Detail *gdp, ParametersDeformablePatches params, int currentFrame)
{

    cout <<this->approachName<< " Updates patches"<<endl;
    UT_String aname("patchIds");
    GA_Attribute        *patchIds = gdp->findFloatArray(GA_ATTRIB_POINT,
                                            aname,
                                            // Allow any tuple size to match
                                            -1, -1);

    if (!patchIds)
    {
        patchIds = gdp->addFloatArray(GA_ATTRIB_POINT,
                                    aname,
                    // Tuple size one means we group the array into
                    // logical groups of 1.  It does *NOT* affect
                    // the length of the arrays, which are always
                    // measured in ints.
                                    1);
    }

    const GA_AIFNumericArray *aif = patchIds->getAIFNumericArray();
    if (!aif)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) aname);
        //addError(SOP_MESSAGE, buf.buffer());
        //return error();
    }

    int numberOfPatches = 0;


    // check the number of patches
    //TODO : we should store that on a detail attribute
    GA_Offset ppt;
    GA_Offset	lcl_start1, lcl_end1;	\
    for (GA_Iterator lcl_it1((gdp)->getPointRange()); lcl_it1.blockAdvance(lcl_start1, lcl_end1); )	\
    for (ppt = lcl_start1; ppt < lcl_end1; ++ppt)
    {
        UT_FloatArray         data;
        aif->get(patchIds, ppt, data);
        int nb = data.size();
        for (int i = 0; i< nb; i++)
        {
            int patchId = data.array()[i];
            if (numberOfPatches <= patchId)
                numberOfPatches = patchId+1;
        }
    }

    GA_RWHandleF attSumAlpha(gdp->addFloatTuple(GA_ATTRIB_POINT,"Alpha", 1));

    GA_FOR_ALL_PTOFF(gdp,ppt)
    {

        //There are a lot of NAN values in the uv attribute. We need to clean that.
        //For this point, take the patch id list and for each one check if there is a nan value
        UT_FloatArray         data;
        aif->get(patchIds, ppt, data);
        int nb = data.size();
        float sumAlpha = 0;
        int nbAlpha = 0;
        for (int i = 0; i< nb; i++)
        {
            int patchId = data.array()[i];
            //if (numberOfPatches< patchId)
            //   numberOfPatches = patchId+1;
            string uvname = "uv"+std::to_string(patchId);
            GA_RWHandleV3 attUVcopy(gdp->findFloatTuple(GA_ATTRIB_POINT,uvname.c_str(), 3));
            if (attUVcopy.isValid())
            {
                UT_Vector3 uv = attUVcopy.get(ppt);
                //------------------------------------ NAN CHECK -------------------------------------
                if (uv.x() != uv.x() || uv.y() != uv.y() || uv.z() != uv.z())
                {
                    uv = UT_Vector3(0,0,0);
                    attUVcopy.set(ppt,uv);
                }
            }
            //-------------------------------------------------------------------------------------

            //-------------------------------------ALPHA SUM---------------------------------------
            string alphaname = "Alpha"+std::to_string(patchId);
            GA_RWHandleF attAlpha(gdp->findFloatTuple(GA_ATTRIB_POINT,alphaname.c_str(), 1));
            if (attAlpha.isValid())
            {
                sumAlpha += attAlpha.get(ppt);
                nbAlpha++;
            }
            //-------------------------------------------------------------------------------------

        }

        attSumAlpha.set(ppt,sumAlpha);

        if (nbAlpha ==0)
            continue;

        sumAlpha;// /= nb;
        float threshold = 1;

        //--------------------------- ADDING A NEW PATCH ! --------------------------------------------
        if(sumAlpha < threshold)
        {
            vector<UT_Vector3> newPatchPoints;
            newPatchPoints.push_back(gdp->getPos3(ppt));
            //cout << "Adding new Patch "<<numberOfPatches<< " on point "<<ppt;
            AddPatches(gdp,newPatchPoints,numberOfPatches,params);
            //cout << " done"<<endl;


            numberOfPatches++;
        }

    }
    cout << "projecting uvs"<<endl;
    ProjectUVSForSelectedGroups(gdp,params);
    cout << "Updates done"<<endl;



}

//================================================================================================

//                                       COMPUTE DISTORTION

//================================================================================================

void DeformablePatches::ComputeDistortion(GU_Detail *gdp, ParametersDeformablePatches params)
{

       //DistortionManager::ComputeDistortionBasedOnEdges(gdp);
    DistortionComputation::ComputeDistortionBasedOnAreas(gdp);

}

//================================================================================================

//                                       Project UV For Patches

//================================================================================================

void DeformablePatches::ProjectUVForPatch(GU_Detail *gdp, GA_PointGroup *patchGroupToAdd, GA_Attribute *uvArray)
{

    //cout << "Project UV for patches"<<endl;

    GU_Detail *referenceGdp = new GU_Detail();
    GA_GroupType groupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *gtable = gdp->getGroupTable(groupType);

    GA_PrimitiveGroup* primGrp = (GA_PrimitiveGroup*)gtable->find(patchGroupToAdd->getName());
    //cout << "group "<<grp->getName()<<endl;
    string groupString = patchGroupToAdd->getName().toStdString();
    groupString.replace(0,5,"");

    referenceGdp->clear();
    referenceGdp->copy(*gdp);

    map<GA_Offset, vector<GA_Offset> > vertexList;
    vector<GA_Offset> vertexVector;

    // Make sure we are an array.  Note tuples do not match to this,
    // nor do GA_*Handle* match!
    // We will match both int and float here, however.
    // (For string, getAIFSharedStringArray)
    UT_String aname("uvs");
    const GA_AIFNumericArray *aif = uvArray->getAIFNumericArray();
    if (!aif)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) aname);
        //addError(SOP_MESSAGE, buf.buffer());
        return;
    }

    //====================== DESTROY OTHER GROUPS ====================
    GEO_Primitive *prim;
    GA_PrimitiveGroup* primGroupToDelete;
    GA_FOR_ALL_PRIMGROUPS(referenceGdp, primGroupToDelete)
    {
        //cout << "is "<< __iter2.group()->getName() <<" != "<< grp->getName()<<endl;
        if(primGroupToDelete->getName() != patchGroupToAdd->getName())
        {
            //cout << " dretroying group "<<primGroupToDelete->getName()<<endl;
            //delete primitives not shared in the selected group
            GA_FOR_ALL_GROUP_PRIMITIVES(referenceGdp, primGroupToDelete, prim)
            {
                if (!primGrp->containsOffset(prim->getMapOffset()))
                {
                    referenceGdp->destroyPrimitiveOffset(prim->getMapOffset());
                }
            }
            //GA_Range primRange = referenceGdp->getPrimitiveRange(primGroupToDelete);
            //referenceGdp->destroyPrimitiveOffsets(primRange,false);
            referenceGdp->destroyPrimitiveGroup(primGroupToDelete->getName());
        }
    }

    //================================================================

    vertexList.clear();
    vertexVector.clear();

    GA_Offset vertexOffset;
    const GA_GroupTable *gReferenceTable = referenceGdp->getGroupTable(groupType);
    GA_PrimitiveGroup* primGrpCopy = (GA_PrimitiveGroup*)gReferenceTable->find(patchGroupToAdd->getName());
    if (primGrpCopy == 0x0)
        return;
    GA_FOR_ALL_GROUP_PRIMITIVES(referenceGdp, primGrpCopy, prim)
    {
        int nbrVertex = (prim)->getVertexCount();
        for(int j= 0; j< nbrVertex; j++)
        {
            vertexOffset = (prim)->getVertexOffset(j);

            //cout << "["<<prim->getMapOffset()<<"] = "<<vertexOffset<<endl;
            vertexList[prim->getMapOffset()].push_back(vertexOffset);
            vertexVector.push_back(vertexOffset);
        }
    }
    if(vertexList.size() == 0)
        return;

    //========================= FLATTEN ====================================
    GU_Flatten flattener(referenceGdp,primGrp,NULL,NULL);

    //cout << "Flattened with "<<flattener.getNumIslands()<<endl;


    bool interupted = flattener.flattenAndPack();
    if (interupted)
        cout << "has been interupted"<<endl;

    //cout << "warning "<<flattener.getWarningMessage()<<endl;
    //======================================================================

    //----------------------------------------------------------------------
    //put back the uvs to the main gdp

    int i = 0;
    string uvname = "uv"+groupString;

    string alphaname = "Alpha"+groupString;

    GA_RWHandleV3 attUVcopy(referenceGdp->findFloatTuple(GA_ATTRIB_VERTEX,"uv", 3));
    GA_RWHandleV3 attUV(gdp->addFloatTuple(GA_ATTRIB_POINT,uvname.c_str() , 3));

    GA_RWHandleF attAlpha(gdp->addFloatTuple(GA_ATTRIB_POINT,alphaname.c_str() , 1));

    int primCounter = 0;
    //GA_Offset vertexOffset;
    GA_PrimitiveGroup* selectedPrimGroup = (GA_PrimitiveGroup*)gtable->find(patchGroupToAdd->getName());

    //cout << "There is "<<selectedPrimGroup->entries() << "primtivies in gdp"<<endl;
    //cout << "There is "<<primGrpCopy->entries() << "primtivies in copy gdp"<<endl;

    GA_Offset pointOffset;

    GA_FOR_ALL_GROUP_PRIMITIVES(gdp, selectedPrimGroup, prim)
    {
        //cout << "Prim "<<prim->getNum();
        int nbrVertex = prim->getVertexCount();
        if (primCounter >= primGrpCopy->entries())
            break;
        for(int j= 0; j< nbrVertex; j++)
        {

            UT_FloatArray         data;
            vertexOffset = prim->getVertexOffset(j);
            pointOffset = gdp->vertexPoint(vertexOffset);

            GA_Offset index = vertexVector[i];
            //GA_Index vertexIndex = referenceGdp->vertexIndex(index);
            UT_Vector3 val = attUVcopy.get(index);
            //cout << " UV "<<val;
            i++;
            attUV.set(pointOffset,val);

            //Shall we do this here ?????????
            attAlpha.set(pointOffset,1.0f);

            //------------ DOES NOT WORK PROPERLY WHEN TRANSFERED TO POINTS ------------------
            // Fetch array value
            aif->get(uvArray, vertexOffset, data);
            data.append(val.x());
            data.append(val.y());
            data.append(val.z());
            // Write back
            aif->set(uvArray, vertexOffset, data);
            //--------------------------------------------------------------------------------
        }
        primCounter++;
        //cout << endl;
    }
    //----------------------------------------------------------------------
    referenceGdp->clearAndDestroy();
}


