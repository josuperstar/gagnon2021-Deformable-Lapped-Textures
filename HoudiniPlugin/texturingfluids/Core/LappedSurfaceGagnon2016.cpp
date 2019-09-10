#include "LappedSurfaceGagnon2016.h"
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
#include <Core/Bridson2012PoissonDiskDistribution.h>
#include <Core/HoudiniUtils.h>


LappedSurfaceGagnon2016::LappedSurfaceGagnon2016(GU_Detail *surface, GU_Detail *trackersGdp) : ParticleAndTrackerManagerGagnon2016(surface, trackersGdp)
{
    this->numberOfPatches = 0;
    this->maxId = 0;


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

    this->markerAdvectionTime = 0;
    this->patchCreationTime = 0;
    this->updatePatchesTime = 0;
    this->numberOfConcealedPatches = 0;
    this->numberOfNewPatches = 0;
    this->numberOfDetachedPatches = 0;
}

LappedSurfaceGagnon2016::~LappedSurfaceGagnon2016()
{
    this->rays.clear();
}


//================================================================================================

//                                     POISSON DISK SAMPLING

//================================================================================================


void LappedSurfaceGagnon2016::PoissonDiskSampling(GU_Detail *levelSet, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{

    //This is a function that does a Poisson Disk Sampling using the approach of Bridson 2012 paper
    //This function is a wrapper to the Bridson2012PoissonDiskDistribution class.
    //It basically take the points from Houdini and fill it to the approach.

    std::clock_t addPoissonDisk;
    addPoissonDisk = std::clock();

    cout << "[Yu2011:PoissonDiskSampling]"<<endl;

    GEO_PointTreeGAOffset trackerTree;
    trackerTree.build(trackersGdp, NULL);

    //cout << "[Yu2011] we have "<<numberOfPoints << " existing point(s) in trackersGdp"<<endl;
    Bridson2012PoissonDiskDistribution poissonDiskDistribution;
    poissonDiskDistribution.PoissonDiskSampling(trackersGdp, trackerTree, levelSet,params.poissondiskradius, params.poissonAngleNormalThreshold, params);

    cout << "[Yu2011] poisson disk sample "<<trackersGdp->getNumPoints()<< " point(s)"<<endl;
    this->poissondisk += (std::clock() - addPoissonDisk) / (double) CLOCKS_PER_SEC;
}

void LappedSurfaceGagnon2016::CreateAPatch(GU_Detail *trackersGdp,  ParametersDeformablePatches params)
{
    //This is a function that does a Poisson Disk Sampling using the approach of Bridson 2012 paper
    //This function is a wrapper to the Bridson2012PoissonDiskDistribution class.
    //It basically take the points from Houdini and fill it to the approach.

    std::clock_t addPoissonDisk;
    addPoissonDisk = std::clock();


    GEO_PointTreeGAOffset trackerTree;
    trackerTree.build(trackersGdp, NULL);

    //cout << "[Yu2011] we have "<<numberOfPoints << " existing point(s) in trackersGdp"<<endl;
    Bridson2012PoissonDiskDistribution poissonDiskDistribution;
    int numberOfClosePoint = 0;
    poissonDiskDistribution.CreateAParticle(trackersGdp, trackerTree, UT_Vector3(0,0,0),UT_Vector3(0,1,0),1,numberOfClosePoint,params);

    cout << "[Yu2011] poisson disk sample "<<trackersGdp->getNumPoints()<< " point(s)"<<endl;

    this->poissondisk += (std::clock() - addPoissonDisk) / (double) CLOCKS_PER_SEC;

}



//================================================================================================

//                                       ADD PATCHES USING BARYCENTRIC COORDONATE

//================================================================================================


void LappedSurfaceGagnon2016::AddSolidPatchesUsingBarycentricCoordinates(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params, GEO_PointTreeGAOffset &surfaceTree)
{

    //This function is used to transfer the uv list from the deformable patches to the surface where the texture will be synthesis.
    cout << "[AddSolidPatchesUsingBarycentricCoordinates]" << endl;

    std::clock_t addPatchesStart;
    addPatchesStart = std::clock();
    this->patchCreationTime = 0;

    fpreal patchRadius = 2*params.poissondiskradius;
    float cs = params.CellSize;
    //================================ CREATE PATCH GROUPS ==============================
    GA_RWHandleV3 attNSurface(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI attNumberOfPatch(surfaceGdp->addIntTuple(GA_ATTRIB_POINT,"numberOfPatch",1));

    if (attLife.isInvalid())
    {
        cout << "particles have no life"<<endl;
        return;
    }

    set<int> patchTreated;
    float r = params.poissondiskradius;
    GA_Offset ppt;
    UT_Vector3 N;
    UT_Vector3 NN;
    UT_Vector3 position;
    UT_Vector3 patchP;
    int isTangeant = 0;
    GA_Offset surfacePointOffset;
    int patchNumber = 0;
    {
        //================== CREATE PATCHES =================
        // create a group of point based on trackers position
        GA_FOR_ALL_PTOFF(trackersGdp, ppt)
        {
            isTangeant = isTangeantTracker.get(ppt);
            if (isTangeant == 1)
                continue;

            patchNumber = attId.get(ppt);
            int active = attActive.get(ppt);
            float life = attLife.get(ppt);
            if (active == 0 && life <= 0 )
                continue;
            if (params.testPatch == 1 && params.patchNumber != patchNumber)
                continue;
            N = attN.get(ppt);
            position = trackersGdp->getPos3(ppt);

            UT_IntArray         patchArrayData;
            //getting neigborhood
            // Close particles indices
            GEO_PointTreeGAOffset::IdxArrayType surfaceNeighborhoodVertices;
            surfaceTree.findAllCloseIdx(position,
                                 patchRadius,
                                 surfaceNeighborhoodVertices);

            unsigned close_particles_count = surfaceNeighborhoodVertices.entries();

            string str = std::to_string(patchNumber);
            UT_String patchGroupName("patch"+str);
            //cout << "Create patch "<<patchGroupName<<endl;
            GA_PointGroup* patchGroup = surfaceGdp->newPointGroup(patchGroupName, 0);


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
                //if (dotP < params.angleNormalThreshold)
                //    continue;

                patchP = surfaceGdp->getPos3(surfacePointOffset);
                //respect poisson disk criterion
                //UT_Vector3 pos          = trackersGdp->getPos3(neighbor);
                UT_Vector3 pos          = patchP;
                //=====================================================
                UT_Vector3 pNp          = position - pos;
                pNp.normalize();
                dotP              = dot(pNp, N);

                //float d              = distance3d( pos, position );
                float dp                = abs(dotP);

                float k        = (1-dp)*r*3;
                if (k < cs*2)
                    k = cs*2;
                //=====================================================
                patchGroup->addOffset(surfacePointOffset);
                patchIdsAtt->get(patchIdsArrayAttrib,surfacePointOffset, patchArrayData);
                int exist = patchArrayData.find(patchNumber);
                if (exist == -1)
                {
                    patchArrayData.append(patchNumber);
                    int numberOfPatch = attNumberOfPatch.get(surfacePointOffset);
                    numberOfPatch++;
                    attNumberOfPatch.set(surfacePointOffset,numberOfPatch);
                }
                patchIdsAtt->set(patchIdsArrayAttrib,surfacePointOffset, patchArrayData);
                numberOfPatcheCreated++;
            }
            neighborhood.clear();
        }
    }
    {
        GA_FOR_ALL_PTOFF(trackersGdp, ppt)
        {
            patchNumber = attId.get(ppt);
            if (patchTreated.count(patchNumber) > 0)
            {
                cout << "We already treated patch "<< patchNumber << endl;
                return;
            }
            patchTreated.insert(patchNumber);

            int active = attActive.get(ppt);
            float life = attLife.get(ppt);

            if (active == 0 && life <= 0 )
                continue;

            //for test purposes
            if (params.testPatch == 1 && params.patchNumber != patchNumber)
                continue;

            string str = std::to_string(patchNumber);
            UT_String pointGroupName("patch"+str);
            //cout << "Create primitive group" << str <<endl;
            GA_PrimitiveGroup* primGrp = surfaceGdp->newPrimitiveGroup(pointGroupName);
            GA_GroupType groupType = GA_GROUP_POINT;
            const GA_GroupTable *gtable = surfaceGdp->getGroupTable(groupType);
            GA_OffsetArray primitives;
            GA_PointGroup* pointGrp = (GA_PointGroup*)gtable->find(primGrp->getName());
            if (pointGrp == 0x0)
                continue;

            GA_FOR_ALL_GROUP_PTOFF(surfaceGdp,pointGrp,ppt)
            {
                surfaceGdp->getPrimitivesReferencingPoint(primitives,ppt);
                for(GA_OffsetArray::const_iterator prims_it = primitives.begin(); !prims_it.atEnd(); ++prims_it)
                {
                    primGrp->addOffset(*prims_it);
                }
            }
        }
     }
    this->patchCreationTime += (std::clock() - addPatchesStart) / (double) CLOCKS_PER_SEC;
}

void LappedSurfaceGagnon2016::OrthogonalUVProjection(GU_Detail* surface, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{
    cout << "[LappedSurfaceGagnon2016] OrthogonalUVProjection" << endl;
    this->orthogonalUVProjectionTime = 0;
    std::clock_t projectionStart;
    projectionStart = std::clock();
    GA_GroupType groupType = GA_GROUP_POINT;
    const GA_GroupTable *gtable = surface->getGroupTable(groupType);
    int patchNumber=0;
    GA_Offset ppt;
    UT_Vector3 N;
    UT_Vector3 S,T;
    int isTangeant = 0;
    UT_Vector3 trackerPosition;
    int i = 0;
    GA_Offset numPoint = trackersGdp->getNumPointOffsets();

//    UT_String s = UT_String("uv");
//    GA_AttributeOwner attribute_type = GA_ATTRIB_VERTEX;
//    surface->destroyAttribute(attribute_type,s);


    GA_RWHandleV3 attUV(surface->addFloatTuple(GA_ATTRIB_POINT,"uv", 3));

    GA_RWHandleV3 attUVTracker(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"uv", 3));

    GA_FOR_ALL_PTOFF(trackersGdp, ppt)
    {
        patchNumber = attId.get(ppt);
        if (params.testPatch == 1 && params.patchNumber != patchNumber)
            continue;

        //replace the tangeant tracker
        isTangeant = isTangeantTracker.get(ppt);
        if (isTangeant == 1)
            continue;
        N = attN.get(ppt);
        trackerPosition = trackersGdp->getPos3(ppt);
        GA_Offset tracker_offset = ppt+numPoint/2;
        UT_Vector3 currentDirection = trackersGdp->getPos3(tracker_offset)-trackerPosition;
        currentDirection.normalize();
        N.normalize();
        S = cross(N,currentDirection);
        S.normalize();
        T = cross(S,N);
        T.normalize();

        // Transform into local patch space (where STN is aligned with XYZ at the origin)

        UT_Vector3 trackerTrianglePos;
        trackerTrianglePos.x() = trackerPosition.dot(S);
        trackerTrianglePos.y() = trackerPosition.dot(T);
        trackerTrianglePos.z() = trackerPosition.dot(N);
        UT_Vector3 centerUv;
        centerUv.x() = trackerTrianglePos.x();
        centerUv.y() = trackerTrianglePos.y();
        centerUv.z() = trackerTrianglePos.z();
        attUVTracker.set(ppt,centerUv);
        cout << "center uv "<<centerUv<<endl;
        //cout << "Projection for patch "<<patchNumber<<endl;
        string patchGroupName= "patch"+std::to_string(patchNumber);
        cout << "Getting point group "<<patchGroupName<<endl;
        //GA_PointGroup* pointGrp = (GA_PointGroup*)gtable->find(patchGroupName);

        GA_PointGroup*pointGrp = (GA_PointGroup*)gtable->find(patchGroupName.c_str());

        //GA_PrimitiveGroup* selectedPrimGroup = (GA_PrimitiveGroup*)gtable->find(patchGroupName);
        //cout << "UV projection on group patch"<<patchNumber<<endl;

        if (pointGrp)
        {
            GA_Offset pointOffset;
            int nbTreated = 0;
            GA_FOR_ALL_GROUP_PTOFF(surface, pointGrp, pointOffset)
            {
                //cout << "Projection for patch "<<pointGrp->getName()<<endl;
                //----------------------- UV PROJECTION --------------
                UT_Vector3 currentPosition = surface->getPos3(pointOffset);

                // Transform into local patch space (where STN is aligned with XYZ at the origin)
                const UT_Vector3 relativePosistion = currentPosition-trackerPosition;
                UT_Vector3 triangleSpacePos;
                triangleSpacePos.x() = relativePosistion.dot(S);
                triangleSpacePos.y() = relativePosistion.dot(T);
                triangleSpacePos.z() = relativePosistion.dot(N);

                UT_Vector3 uv;
                uv.x() = triangleSpacePos.x();
                uv.y() = triangleSpacePos.y();
                uv.z() = triangleSpacePos.z();

                float mid = 0.5;
                //uv /= params.UVScaling;
                uv += mid;
                //uv.z should be zero
                //uv -= centerUv;

                UT_FloatArray         fdata;
                UT_IntArray patchArrayData;
                // Fetch array value
                patchIdsAtt->get(patchIdsArrayAttrib, pointOffset, patchArrayData);
                int nb = patchArrayData.size();
                int index = -1;
                for (int i = 0; i< nb; i++)
                {
                    if (patchArrayData.array()[i] == patchNumber)
                        index = i;
                }

                // Fetch array value
                uvsArray->get(uvsAtt, pointOffset, fdata);

                if (index == -1)
                    continue;

                fdata.insertAt(uv.x(), (index*3)+0);
                fdata.insertAt(uv.y(), (index*3)+1);
                fdata.insertAt(uv.z(), (index*3)+2);

                // Write back
                uvsArray->set(uvsAtt, pointOffset, fdata);

                //cout << "fdata:"<<fdata<<endl;
                attUV.set(pointOffset,uv);
                nbTreated++;
            }
            cout << "number of point with uv: "<<nbTreated<<endl;
        }
    }

    this->orthogonalUVProjectionTime += (std::clock() - projectionStart) / (double) CLOCKS_PER_SEC;
}

//======================================================================================================================================
//                                                  UpdateUsingBridson2012PoissonDisk
//======================================================================================================================================

void LappedSurfaceGagnon2016::DeleteUnusedPatches(GU_Detail *gdp, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{
    cout << this->approachName<<" Update Using Bridson 2012 Poisson Disk with "<<numberOfPatches<<" existing trackers"<<endl;
    std::clock_t startUpdatePatches;
    startUpdatePatches = std::clock();

    //--------------------------------------------------------------------------

    /*
    GA_RWHandleI attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleF attLife(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleI attSpawn(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"spawn",1));
    GA_RWHandleI attActive(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));

    GA_RWHandleI attIsMature(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"isMature", 1));
    */
    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;

    GA_Offset ppt;
    int beforeAddingNumber = numberOfPatches;

    GA_PointGroup *grpToDestroy = (GA_PointGroup *)trackersGdp->newPointGroup("ToDelete");
    GA_GroupType groupType = GA_GROUP_POINT;
    const GA_GroupTable *gtable = gdp->getGroupTable(groupType);
    GA_GroupType primGroupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *gPrimTable = gdp->getGroupTable(primGroupType);
    set<int> toDelete;
    //--------------------------- DELETE DEAD PATCH --------------------------------------------
    {
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {
            int id = attId.get(ppt);
            float life = attLife.get(ppt);
            int active = attActive.get(ppt);
            if (active == 0 && life <= 0.0f && params.frame != params.startFrame)
            {
                //cout << "Deleting deformable grid "<<id<<" mature "<<attIsMature.get(ppt)<<endl;
                toDelete.insert(id);
                numberOfPatches--;
                numberOfConcealedPatches++;

                string str = std::to_string(id);
                string groupName = "grid"+str;
                GA_PrimitiveGroup *primGroup = (GA_PrimitiveGroup*)gPrimTable->find(groupName.c_str());
                GA_PointGroup* pointGrp = (GA_PointGroup*)gtable->find(groupName.c_str());

                if (pointGrp != 0x0)
                {
                    //cout << "delete points "<<endl;
                    gdp->deletePoints(*pointGrp,mode);
                    //cout << "delete point group "<<groupName<<endl;
                    gdp->destroyPointGroup(pointGrp);
                }
                if (primGroup != 0x0)
                {
                    //cout << "delete point group "<<groupName<<endl;
                    gdp->destroyPrimitiveGroup(primGroup);
                }
            }
        }
    }
    {
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {
            int id = attId.get(ppt);
            if (toDelete.count(id) > 0)
            {
                //cout << "Delete poisson disk point "<<id<<endl;
                grpToDestroy->addOffset(ppt);
            }
        }
    }
    //destroying trackers
    trackersGdp->deletePoints(*grpToDestroy,mode);
    trackersGdp->destroyPointGroup(grpToDestroy);

    cout <<this->approachName<< " Added "<<(numberOfPatches-beforeAddingNumber) <<" new patches"<<endl;
    cout <<this->approachName<< " Removed "<<(numberOfConcealedPatches)<<" patches "<<endl;
    cout <<this->approachName<< "uv projection time "<<orthogonalUVProjectionTime<< endl;
    this->updatePatchesTime += (std::clock() - startUpdatePatches) / (double) CLOCKS_PER_SEC;
    cout << this->approachName<<" TOTAL "<<numberOfPatches<< " patches"<<endl;
}

