#include "DeformableGridsManager.h"

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

#include <Core/HoudiniUtils.h>


DeformableGridsManager::DeformableGridsManager(GU_Detail *surfaceGdp, GU_Detail *surfaceLowResGdp, GU_Detail *trackersGdp, GU_Detail *deformableGridsGdp, ParametersDeformablePatches params)  : ParticleTrackerManager(surfaceGdp, trackersGdp, params)
{
    this->deformableGridsGdp = deformableGridsGdp;
    this->surfaceLowResGdp = surfaceLowResGdp;

    this->maxId = 0;
    this->gridCenterPosition.clear();

    this->uvFlatteningTime = 0;
    this->gridMeshCreation = 0;
    this->gridAdvectionTime = 0;
    this->markerAdvectionTime = 0;

    this->nbOfFlattenedPatch = 0;
    this->numberOfDegeneratedGrid = 0;

    distortionParams.deletionLife               = params.fadingTau;
    distortionParams.distortionRatioThreshold   = params.distortionRatioThreshold ;
    distortionParams.Yu2011DMax                 = params.Yu2011DMax ;
    distortionParams.QvMin                      = params.QvMin;

    distortionParams.alphaName = "Alpha";
    distortionParams.temporalRemoveName = "temporalRemove";
    distortionParams.initialVertexAngle = initialVertexAngle;
    distortionParams.distortionWeightName = distortionWeightName;
    distortionParams.primLifeName = "life";

    //should move this in the global approach
    distortionParams.flagDistortedParticles = true;

    //GEO_PointTreeGAOffset surfaceTree;
    surfaceTree.build(surfaceGdp, NULL);
    surfaceLowResTree.build(surfaceLowResGdp, NULL);

}

DeformableGridsManager::~DeformableGridsManager()
{
    this->surfaceTree.clear();
}

//================================================================================================

//                                       ADD DEFORMABLE GRID

//================================================================================================

void DeformableGridsManager::CreateGridBasedOnMesh(GA_Offset ppt)
{
    //cout << "[DeformableGridsManager] CreateGridBasedOnMesh with beta "<<params.Yu2011Beta<<endl;

    //Yu2011 Section 3.3.1
    //For each particle, we create a regular grid (see Fig. 2, left), centered on it, of width larger than 2d. Combined
    //with the properties of the boundary sampling algorithm, this guarantees a gap-less coverage of the fluid.

    //Section 3.3.2
    //At their creation, grids are slightly larger than kernel size, to avoid triggering condition 1 too early. We create
    //grids of width (2 + β)d, with a small β (in our implementation, β = 0.6). The size of the grid is a compromise
    //between particle lifetime and the number of vertices it will require to ensure a given resolution

    float beta = params.Yu2011Beta;
    float d = params.poissondiskradius;
    float gridwidth = (2+beta)*d;

    GA_RWHandleF    attTrackerLife(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"life",1));

    GA_RWHandleF    attNumberOfPrimitives(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"numberOfPrimitives",1));


    GA_RWHandleV3   attUV(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,uvName, 3));
    GA_RWHandleF    attAlpha(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"Alpha",1));
    GA_RWHandleI    attIsGrid(this->deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"isGrid",1));
    GA_RWHandleF    attInitArea(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"initArea",1));
    GA_RWHandleI    attInitId(this->deformableGridsGdp->addIntTuple(GA_ATTRIB_PRIMITIVE,"initId",1));
    GA_RWHandleI    attIsTreated(this->deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"isTreated",1));
    GA_RWHandleV3   attV(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   attCd(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));
    GA_RWHandleI    attGridId(this->deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));

    GA_RWHandleF    attAlpha0(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,alpha0Name,1));
    GA_RWHandleF    attVW(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,vertexWeightName,1));
    GA_RWHandleF    attW(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,distortionWeightName,1));
    GA_RWHandleF    attW0(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,distortionWeight0Name,1));

    GA_RWHandleV3   attSs(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"Ss",3));
    GA_RWHandleV3   attSt(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"St",3));
    GA_RWHandleF    attDMax(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"dmax",1));
    GA_RWHandleF    attDMin(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"dmin",1));
    GA_RWHandleF    attDistortion(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"distortion",1));
    GA_RWHandleF    attQt(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"Qt",1));
    GA_RWHandleF    attQv(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"Qv",1));

    GA_RWHandleF    attA(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"a",1));
    GA_RWHandleF    attB(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"b",1));
    GA_RWHandleF    attC(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"c",1));
    GA_RWHandleF    attArea(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"area",1));

    GA_RWHandleF    attDP0(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"dP0",1));

    GA_RWHandleV3 attRP(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_VERTEX,"refPosition",3));
    GA_RWHandleF attVA(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_VERTEX,initialVertexAngle,1));
    GA_RWHandleF attLife(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleI attPrimLife(this->deformableGridsGdp->addIntTuple(GA_ATTRIB_PRIMITIVE,"life",1));

    GA_RWHandleV3 attNSurface(this->surfaceLowResGdp->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleV3 attVSurface(this->surfaceLowResGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));


    UT_Vector3 TrackerN;
    UT_Vector3 N;
    UT_Vector3 v;
    UT_Vector3 trackerPositition;
    GA_Offset newPoint;
    float life = 1.0f;
    float cs = params.CellSize;

    string groupName = "grids";
    //GA_PointGroup *grpGrid = this->deformableGridsGdp->newPointGroup(groupName.c_str());

    float scaling = gridwidth;

    //cout << "go through all trackers"<<endl;

    int id =0;
    //for(it = trackers.begin(); it != trackers.end(); it++)
    //GA_Offset ppt;

    bool toDelete = false;
    id = attId.get(ppt);

    //cout << "tracker "<<id<< endl;
    if (params.testPatch == 1 && params.patchNumber != id)
        return;

    life = attTrackerLife.get(ppt);
//    //int spawn = attSpawn.get(ppt);
//    if (spawn != 1 || attActive.get(ppt) == 0)
//        return;

    //cout << "Create Grid "<<id;

    GU_Detail tempGdp;
    set<GA_Offset> tempGdpListOffset;
    GA_Offset tempNewPoint;
    GA_RWHandleI attInitVertexId(tempGdp.addIntTuple(GA_ATTRIB_VERTEX,"initVerterxId",1));

    TrackerN = attN.get(ppt);
    UT_Vector3 p = trackersGdp->getPos3(ppt);

    std::clock_t startMeshCreation;
    startMeshCreation = std::clock();

    string str = std::to_string(id);
    groupName = "grid"+str;
    GA_PointGroup *pointGroup = this->deformableGridsGdp->newPointGroup(groupName.c_str());
    GA_PointGroup *tempPointGroup = tempGdp.newPointGroup(groupName.c_str());
    GA_PrimitiveGroup *primGroup = this->deformableGridsGdp->newPrimitiveGroup(groupName.c_str());

    //cout << "[DeformableGridsManager] CreateGridBasedOnMesh  "<<groupName<<endl;

    trackerPositition = trackersGdp->getPos3(ppt);
    set<GA_Offset> primList;
    vector<GA_Offset> pointList;
    vector<GA_Offset> tempPointList;

    GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;
    this->surfaceLowResTree.findAllCloseIdx(trackerPositition,
                         gridwidth*2,
                         close_particles_indices);

    GA_Offset surfaceClosestPoint = this->surfaceLowResTree.findNearestIdx(trackerPositition);
    GA_Offset closestPoint = -1;
    set<GA_Offset> pointsAround;

    map<GA_Offset,GA_Offset> pointsLink;
    map<GA_Offset,GA_Offset> tempPointsLink;
    unsigned close_particles_count = close_particles_indices.entries();
    int nbDistorted = 0;
    if (close_particles_count > 0)
    {
        GA_Offset neighbor;
        for(unsigned int j=0; j<close_particles_count;j++ )
        {
            neighbor = close_particles_indices.array()[j];
            N = attNSurface.get(neighbor);
            v = attVSurface.get(neighbor);
            N.normalize();
            if (dot(N,TrackerN) < params.angleNormalThreshold)
            {
                nbDistorted++;
                continue;
            }
            //respect poisson disk criterion
            //UT_Vector3 pos          = trackersGdp->getPos3(neighbor);
            UT_Vector3 pos          = this->surfaceLowResGdp->getPos3(neighbor);
            //=====================================================
            UT_Vector3 pNp          = p - pos;
            pNp.normalize();
            float dotP              = dot(pNp, N);
            float d              = distance3d( pos, p );
            float dp                = abs(dotP);
            float k        = (1-dp)*gridwidth;
            if (k < cs)
                k = cs;
            bool insideBigEllipse    = d < k;
            if (!insideBigEllipse)
                continue;
            //=====================================================
            //--------create new points ------------
            newPoint = this->deformableGridsGdp->appendPointOffset();
            tempNewPoint = tempGdp.appendPointOffset();
            tempGdpListOffset.insert(tempNewPoint);

            if (surfaceClosestPoint == neighbor)
            {
                closestPoint = tempNewPoint;
            }

            this->deformableGridsGdp->setPos3(newPoint,this->surfaceLowResGdp->getPos3(neighbor));
            tempGdp.setPos3(tempNewPoint,this->surfaceLowResGdp->getPos3(neighbor));
            pointGroup->addOffset(newPoint);
            tempPointGroup->addOffset(tempNewPoint);
            pointsAround.insert(tempNewPoint);

            //grpGrid->addOffset(newPoint);

            attIsGrid.set(newPoint,1);
            attAlpha.set(newPoint,(float)life/(float)params.fadingTau);
            attVW.set(newPoint,0);
            attAlpha0.set(newPoint,(float)life/(float)params.fadingTau);
            attLife.set(newPoint,params.fadingTau);
            attV.set(newPoint,v);
            attIsTreated.set(newPoint,0);
            UT_Vector3 patchColor = AttCd.get(ppt);

            attCd.set(newPoint,patchColor);
            attGridId.set(newPoint,id);
            attQv.set(newPoint,1.0f);
            pointList.push_back(newPoint);
            tempPointList.push_back(tempNewPoint);
            pointsLink[neighbor] = newPoint;
            tempPointsLink[neighbor] = tempNewPoint;

            //----------------- Dynamic Tau ------------------------
            float dP0 = distance3d(p,pos);
            attDP0.set(newPoint,dP0);
            //------------------------------------------------------
        }

        set<GA_Offset> neighborPrims;
        for(int j=0; j<close_particles_count;j++ )
        {
            neighbor = close_particles_indices.array()[j];

            //neighborPrims = HoudiniUtils::GetPrimitivesNeighbors(surfaceGdp,neighbor);
            GA_OffsetArray primitives;
            GA_Offset prim_off;
            this->surfaceLowResGdp->getPrimitivesReferencingPoint(primitives,neighbor);
            for(GA_OffsetArray::const_iterator prims_it = primitives.begin(); prims_it != primitives.end(); ++prims_it)
            {
                bool add = true;
                //we need to be sure that the primitives have all its points in the point list selected before
                GEO_Primitive* prim = this->surfaceLowResGdp->getGEOPrimitive(*prims_it);
                int nbVertex = prim->getVertexCount();
                for(int i = 0; i<nbVertex; i++)
                {
                    GA_Offset vertex = prim->getVertexOffset(i);
                    GA_Offset point = this->surfaceLowResGdp->vertexPoint(vertex);
                    if (!pointsLink.count(point))
                    {
                        add = false;
                        break;
                    }
                }
                if(add)
                {
                    prim_off = *prims_it;
                    neighborPrims.insert(prim_off);
                }
            }
            set<GA_Offset>::iterator itPrim;
            for (itPrim = neighborPrims.begin(); itPrim != neighborPrims.end(); ++itPrim)
                primList.insert(*itPrim);
        }
    }

    //cout << "Create primitives"<<endl;
    //cout << "There is "<<primList.size() <<" primitives"<<endl;
    set<GA_Offset>::iterator itPrim;
    for(itPrim = primList.begin(); itPrim != primList.end(); ++itPrim)
    {
        //primGroup->addOffset(*itPrim);
        vector<UT_Vector3> trianglePoints;
        GEO_PrimPoly *prim_poly_ptr = (GEO_PrimPoly *)this->deformableGridsGdp->appendPrimitive(GA_PRIMPOLY);
        prim_poly_ptr->setSize(0);

        GEO_PrimPoly *temp_prim_poly_ptr = (GEO_PrimPoly *)tempGdp.appendPrimitive(GA_PRIMPOLY);
        temp_prim_poly_ptr->setSize(0);

        GEO_Primitive* prim = this->surfaceLowResGdp->getGEOPrimitive(*itPrim);
        //cout << "Creating prim "<<*itPrim<<endl;
        int nbVertex = prim->getVertexCount();
        map<GA_Offset,GA_Offset>::iterator m;
        for(int i = 0; i < nbVertex; i++)
        {
            GA_Offset vertex = prim->getVertexOffset(i);
            GA_Offset point = this->surfaceLowResGdp->vertexPoint(vertex);

            trianglePoints.push_back(this->surfaceLowResGdp->getPos3(point));

            //------------- VERTEX ANGLE ----------------
            int b = i-1;
            if (b < 0)
                b = nbVertex-1;
            int c = i+1;
            if (c >= nbVertex)
                c = 0;

            GA_Offset vB = prim->getVertexOffset(b);
            GA_Offset pB = this->surfaceLowResGdp->vertexPoint(vB);
            GA_Offset vC = prim->getVertexOffset(c);
            GA_Offset pC = this->surfaceLowResGdp->vertexPoint(vC);

            UT_Vector3 AB = this->surfaceLowResGdp->getPos3(pB)-this->surfaceLowResGdp->getPos3(point);
            UT_Vector3 AC = this->surfaceLowResGdp->getPos3(pC)-this->surfaceLowResGdp->getPos3(point);
            AB.normalize();
            AC.normalize();
            float angle = dot(AB,AC);
            //-------------------------------------------

            //cout <<"append vertex "<<point<<endl;
            m = pointsLink.find(point);
            if (m != pointsLink.end())
            {
                GA_Size idx = prim_poly_ptr->appendVertex(pointsLink[point]);
                GA_Size idx2 = temp_prim_poly_ptr->appendVertex(tempPointsLink[point]);
                GA_Offset newVertex = prim_poly_ptr->getVertexOffset(idx);
                attVA.set(newVertex,angle);

                GA_Offset newVertex2 = temp_prim_poly_ptr->getVertexOffset(idx2);
                attInitVertexId.set(newVertex2,newVertex);
            }
            else
            {
                cout << "there is no point "<<point<<endl;
            }
        }
        attW.set(prim_poly_ptr->getMapOffset(),0);
        attW0.set(prim_poly_ptr->getMapOffset(),0);
        attPrimLife.set(prim_poly_ptr->getMapOffset(),(float)life/params.fadingTau);
        attInitId.set(prim_poly_ptr->getMapOffset(),prim_poly_ptr->getMapOffset());

        //================================= REF POSITION ========================================
        //compute triangle reference for section 3.3.3
        UT_Vector3 A = trianglePoints[0];
        UT_Vector3 B = trianglePoints[1];
        UT_Vector3 C = trianglePoints[2];
        UT_Vector3 p = (A+B+C)/3;

        UT_Vector3 i = B-A;
        i.normalize();
        UT_Vector3 j = C-A;
        j.normalize();
        UT_Vector3 k = cross(i,j);
        k.normalize();
        j = cross(i,k);
        j.normalize();

        // Transform into local patch space (where ijk is aligned with XYZ at the origin)
        UT_Vector3 relativePosistion = A-p;
        UT_Vector3 triangleSpacePosA;
        triangleSpacePosA.x() = relativePosistion.dot(i);
        triangleSpacePosA.y() = relativePosistion.dot(j);
        triangleSpacePosA.z() = relativePosistion.dot(k);

        relativePosistion = B-p;
        UT_Vector3 triangleSpacePosB;
        triangleSpacePosB.x() = relativePosistion.dot(i);
        triangleSpacePosB.y() = relativePosistion.dot(j);
        triangleSpacePosB.z() = relativePosistion.dot(k);

        relativePosistion = C-p;
        UT_Vector3 triangleSpacePosC;
        triangleSpacePosC.x() = relativePosistion.dot(i);
        triangleSpacePosC.y() = relativePosistion.dot(j);
        triangleSpacePosC.z() = relativePosistion.dot(k);

        GA_Offset vertexA = prim_poly_ptr->getVertexOffset(0);
        GA_Offset vertexB = prim_poly_ptr->getVertexOffset(1);
        GA_Offset vertexC = prim_poly_ptr->getVertexOffset(2);

        prim_poly_ptr->close();
        temp_prim_poly_ptr->close();

        attRP.set(vertexA,triangleSpacePosA);
        attRP.set(vertexB,triangleSpacePosB);
        attRP.set(vertexC,triangleSpacePosC);

        //======================== SORKINE 2002 SECTION 3.2 =========================
        //this part of the code is also in Yu2011Distortion.cpp
        UT_Vector3 p1 = triangleSpacePosA;
        UT_Vector3 p2 = triangleSpacePosB;
        UT_Vector3 p3 = triangleSpacePosC;


        float s1 = p1.x();
        float s2 = p2.x();
        float s3 = p3.x();

        float t1 = p1.y();
        float t2 = p2.y();
        float t3 = p3.y();

        UT_Vector3 q1 = trianglePoints[0];
        UT_Vector3 q2 = trianglePoints[1];
        UT_Vector3 q3 = trianglePoints[2];

        float area = abs(((s2 - s1)*(t3-t1) - (s3-s1)*(t2-t1))/2.0f);
        attArea.set(prim_poly_ptr->getMapOffset(),area);

        if (area != 0.0f)
        {
            UT_Vector3 Ss = ( (t2-t3)*q1 + (t3-t1) *q2+ (t1-t2)*q3)/(2.0f*area);
            UT_Vector3 St = ( (s3-s2)*q1 + (s1-s3) *q2+ (s2-s1)*q3)/(2.0f*area);

            float a = dot(Ss,Ss);
            float b = dot(Ss,St);
            float c = dot(St,St);

            float gmax = sqrt(0.5f*((a+c)+sqrt((a-c)*(a-c) + 4*(b*b))));
            float gmin = sqrt(0.5f*((a+c)-sqrt((a-c)*(a-c) + 4*(b*b))));
            float dt = std::max(gmax,1/gmin);

            attA.set(prim_poly_ptr->getMapOffset(),a);
            attB.set(prim_poly_ptr->getMapOffset(),b);
            attC.set(prim_poly_ptr->getMapOffset(),c);

            attSs.set(prim_poly_ptr->getMapOffset(),Ss);
            attSt.set(prim_poly_ptr->getMapOffset(),St);
            attDMax.set(prim_poly_ptr->getMapOffset(),gmax);
            attDMin.set(prim_poly_ptr->getMapOffset(),gmin);
            attDistortion.set(prim_poly_ptr->getMapOffset(),dt);
            //cout << "set Qt "<<1.0<<endl;
            attQt.set(prim_poly_ptr->getMapOffset(),1.0f);

        }
        //====================================================================
        //=====================================================================================

        primGroup->addOffset(prim_poly_ptr->getMapOffset());
    }

    int numberOfPrimitives = primList.size();
    attNumberOfPrimitives.set(ppt,numberOfPrimitives);
    if (numberOfPrimitives == 0)
        toDelete = true;

    this->gridMeshCreation += (std::clock() - startMeshCreation) / (double) CLOCKS_PER_SEC;

    if (close_particles_count == 0)
    {
        if (params.testPatch == 1 && params.patchNumber == id)
        {
            cout << " not ok"<<endl;
        }
        toDelete = true;
        return;
    }
    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;

    //=====================================================================================
    //--------------------- UV FLATENING-------------------
    bool flattening = true;
    if (flattening)
    {
        //cout << "UV Flattening"<<endl;
        bool flattened = this->UVFlattening(tempGdp, ppt, closestPoint, pointGroup, tempPointGroup, pointsAround, scaling );
        if (!flattened)
        {
            toDelete = true;
        }
    }
    //cout << "UV Flattening done"<<endl;
    //--------------------------------------------------
    //Take a random part of the input texture uv space
    //- compute a random translation
    //- scale up
//    float scaleup = params.UVScaling;
//    if (scaleup == 0)
//        scaleup = 1;
//    int seed = id;
//    float randomScale = scaleup/2.0f;
//    srand(seed);
//    float tx = (((double) rand()/(RAND_MAX)))*randomScale;
//    srand(seed+1);
//    float ty = (((double) rand()/(RAND_MAX)))*randomScale;
//    srand(seed+2);
//    float tz = (((double) rand()/(RAND_MAX)))*randomScale;
//    {
//        GA_Offset gppt;
//        GA_FOR_ALL_GROUP_PTOFF(this->deformableGridsGdp,pointGroup,gppt)
//        {
//            UT_Vector3 uv = attUV.get(gppt);
//            if (uv.x() != uv.x())
//            {
//                //where have nan value
//                uv = UT_Vector3(0,0,0);
//            }
//            uv += UT_Vector3(tx,ty,tz);
//            uv /= scaleup;
//            attUV.set(gppt,uv);
//        }
//    }
    UT_Vector3 centerUV = UT_Vector3(0,0,0);
    int i = 0;
    {
        GA_Offset gppt;
        GA_FOR_ALL_GROUP_PTOFF(this->deformableGridsGdp,pointGroup,gppt)
        {
            UT_Vector3 uv = attUV.get(gppt);
            uv.z() = 0;
            centerUV += uv;
            i++;
        }
    }
    if (i > 0)
    {
        centerUV /= i;
        attCenterUV.set(ppt,centerUV);
    }
    //-----------------------------------------------------
    GEO_Primitive *prim;
    float area;
    GA_FOR_ALL_GROUP_PRIMITIVES(this->deformableGridsGdp,primGroup,prim)
    {
        area = prim->calcArea();
        attInitArea.set(prim->getMapOffset(),area);
    }

    if(pointGroup->entries() == 0)
    {
        //delete prim point and prim group
        cout <<"[DeformableGrids]CreateGridBasedOnMesh: delete patch because there is no point"<< pointGroup->getName()<<endl;
        this->deformableGridsGdp->deletePoints(*pointGroup,mode);
        this->deformableGridsGdp->destroyPointGroup(pointGroup);
        this->deformableGridsGdp->destroyPrimitiveGroup(primGroup);
        toDelete = true;
        //DeleteTracker(trackersGdp,id);
    }
    if (params.testPatch == 1 && params.patchNumber == id)
    {
        //cout << " ok"<<endl;
    }
    if (toDelete)
    {
        attTrackerLife.set(ppt,0);
        attActive.set(ppt,0);
        this->numberOfDegeneratedGrid++;
    }
    //cout << "Grid Creation done"<<endl;
    //this->FlagBoundaries();
    this->FlagBoundariesForPatch(ppt);
}


//================================================================================================

//                                       ADD DEFORMABLE GRID

//================================================================================================

void DeformableGridsManager::CreateGridsBasedOnMesh( vector<GA_Offset> trackers)
{
    cout << "[DeformableGridsManager] CreateGridBasedOnMesh with beta "<<params.Yu2011Beta<<endl;
    vector<GA_Offset>::iterator itT;
    for (itT = trackers.begin(); itT != trackers.end(); itT++)
    {
        GA_Offset ppt= *itT;
        //GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {
           this->CreateGridBasedOnMesh(ppt);
           //this->FlagBoundariesForPatch(ppt);
        }
    }
    //this->FlagBoundaries();
}

//================================================================================================

//                                      ADVECT GRIDS

//================================================================================================


void DeformableGridsManager::AdvectGrids()
{
    //cout << "[Yu2011] Advect markers"<<endl;
    cout << this->approachName<<" Advect grids";
    if (params.computeDistortion == 1)
    {
        cout << " with distortion"<<endl;
    }
    else
    {
        cout << " without distortion"<<endl;
    }

    std::clock_t startAdvection;
    startAdvection = std::clock();

    GA_RWHandleV3   attVDeformable(this->deformableGridsGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   attNSurface(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleV3   attCd(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));
    GA_RWHandleF    attAlpha(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"Alpha",1));
    GA_RWHandleI    attGridId(this->deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleF    attDP0(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"dP0",1));
    GA_RWHandleF    attDPi(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"dPi",1));
    GA_RWHandleF    attDeltaOnD(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"deltaOnD",1));
    GA_RWHandleF    attQt(this->deformableGridsGdp->findFloatTuple(GA_ATTRIB_PRIMITIVE,"Qt",1));

    GA_RWHandleV3   refAttV(this->surface->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   refAttN(this->surface->addFloatTuple(GA_ATTRIB_POINT,"N", 3));

    if (attV.isInvalid())
    {
        cout << this->approachName << " Grids have no velocity"<<endl;
        return;
    }

    UT_Vector3 velocity;
    UT_Vector3 p;
    UT_Vector3 p1;
    float dt = 1.0f/24.0f;
    float thresholdDistance = params.maximumProjectionDistance;
    float poissonDiskD = params.poissondiskradius;

    //----------------------distortion---------------------------


    //---------------------------------------------------
    float cs = params.CellSize;
    float r = params.poissondiskradius;

    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;
    GA_PointGroup * grpToDestroy;
    grpToDestroy = (GA_PointGroup *)this->deformableGridsGdp->newPointGroup("GridPointToDelete");
    GA_PrimitiveGroup *primGrpToDestroy = (GA_PrimitiveGroup *)this->deformableGridsGdp->newPrimitiveGroup("PrimToDelete");
    GU_MinInfo mininfo;
    GU_RayIntersect ray(this->surface);
    ray.init();

    GA_GroupType groupType = GA_GROUP_POINT;
    const GA_GroupTable *gtable = this->deformableGridsGdp->getGroupTable(groupType);
    GA_GroupType primGroupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *gPrimTable = this->deformableGridsGdp->getGroupTable(primGroupType);

    GA_PointGroup* pointGrp;
    UT_Vector3 N;
    GA_Offset ppt;
    GA_Offset trackerPpt;
    int id;
    int active;
    float life;

    GA_FOR_ALL_PTOFF(trackersGdp,trackerPpt)
    {
        id = attId.get(trackerPpt);

        if (params.testPatch == 1 && params.patchNumber != id)
            continue;

        active = attActive.get(trackerPpt);
        int spawn = attSpawn.get(trackerPpt);
        life = attLife.get(trackerPpt);
        float gridAlpha = (float)life/(float)params.fadingTau;
        UT_Vector3 trackerPosition = trackersGdp->getPos3(trackerPpt);
        UT_Vector3 trackerN = attN.get(trackerPpt);

        string str = std::to_string(id);
        string groupName = "grid"+str;
        pointGrp = (GA_PointGroup*)gtable->find(groupName.c_str());
        GA_PrimitiveGroup *primGroup = (GA_PrimitiveGroup*)gPrimTable->find(groupName.c_str());
        float averageDeltaOnD = 0.0f;
        float maxDeltaOnD = 0.0f;
        int nbOfPoint = 0;

        if (pointGrp)
        {
            if (life <= 0)
            {
                //we don't deal with dead patches.
                if (params.testPatch == 1 && params.patchNumber == id)
                {
                    cout <<"we don't deal with dead patches."<<endl;
                }
            }
            else
            {
                {
                    bool first = true;
                    UT_Vector3 averagePosition;

                    GA_FOR_ALL_GROUP_PTOFF(this->deformableGridsGdp,pointGrp,ppt)
                    {

                        //check if it is a lonely point
                        GA_OffsetArray primitivesList;
                        GA_Size numberOfPrimitives = this->deformableGridsGdp->getPrimitivesReferencingPoint(primitivesList,ppt);
                        if (numberOfPrimitives == 0)
                        {
                            //cout << "Tracker with no primivites"<<endl;
                            continue;
                        }
                        velocity = attVDeformable.get(ppt);
                        N = attNSurface.get(ppt);
                        p = this->deformableGridsGdp->getPos3(ppt);

                        //advect
                        UT_Vector3 d = velocity*dt;
                        p1 = p+d;
                        this->deformableGridsGdp->setPos3(ppt,p1);
                        attGridId.set(ppt,id);
                        //------------------------------------------------------------------------
                        bool projection = true;
                        if(projection)
                        {
                            //reproject on the N plan if this is the tangent tracker
                            mininfo.init(thresholdDistance,0.0001);
                            ray.minimumPoint(p1,mininfo);

                            if (!mininfo.prim)
                            {
                                cout << "No primitive to project on"<<endl;
                                continue;
                            }
                            const GEO_Primitive *geoPrim = mininfo.prim;
                            int vertexCount = geoPrim->getVertexCount();
                            if (vertexCount > 5 || vertexCount < 3)
                            {
                                cout << "vertex count "<<vertexCount<<" for primitive "<<geoPrim->getMapOffset()<<endl;
                                continue;
                            }
                            //get pos of hit
                            UT_Vector4 hitPos;
                            mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
                            if (distance3d(p1,hitPos) < thresholdDistance)
                            {
                                p1 = hitPos;
                                this->deformableGridsGdp->setPos3(ppt,p1);

                                //respect poisson disk criterion
                                //UT_Vector3 pos          = trackersGdp->getPos3(neighbor);
                                //=====================================================

                                UT_Vector3 pNp          = trackerPosition - hitPos;
                                pNp.normalize();
                                float dotP              = dot(pNp, trackerN);

                                float d              = distance3d( trackerPosition, hitPos );
                                float dp                = abs(dotP);

                                float k        = (1-dp)*r*2;
                                if (k < cs)
                                    k = cs;

                                //------------------------------PARAMETRIC COORDINATE -----------------------------------
                                GA_Offset primOffset = mininfo.prim->getMapOffset();
                                float u = mininfo.u1;
                                float v = mininfo.v1;
                                GEO_Primitive *prim = this->surface->getGEOPrimitive(primOffset);

                                GA_Offset vertexOffset0 = prim->getVertexOffset(0);
                                //UT_Vector3 v0 = attUV.get(vertexOffset0);
                                GA_Offset pointOffset0  = this->surface->vertexPoint(vertexOffset0);
                                UT_Vector3 n0 = refAttN.get(pointOffset0);
                                UT_Vector3 v0 = refAttV.get(pointOffset0);

                                GA_Offset vertexOffset1 = prim->getVertexOffset(1);
                                GA_Offset pointOffset1  = this->surface->vertexPoint(vertexOffset1);
                                UT_Vector3 n1 = refAttN.get(pointOffset1);
                                UT_Vector3 v1 = refAttV.get(pointOffset1);
                                //UT_Vector3 v1 = attUV.get(vertexOffset1);

                                GA_Offset vertexOffset2 = prim->getVertexOffset(2);
                                GA_Offset pointOffset2  = this->surface->vertexPoint(vertexOffset2);
                                UT_Vector3 n2 = refAttN.get(pointOffset2);
                                UT_Vector3 v2 = refAttV.get(pointOffset2);;
                                //UT_Vector3 v2 = attUV.get(vertexOffset2);

                                UT_Vector3 normal   = n0+u*(n1-n0)+v*(n2-n0);
                                UT_Vector3 velocity = v0+u*(v1-v0)+v*(v2-v0);
                                attVDeformable.set(ppt,velocity);

                                attNSurface.set(ppt,normal);
                                attAlpha.set(ppt,gridAlpha);
                                //------------------------------------------------------------------------------------
                            }
                            else
                            {
                                this->deformableGridsGdp->setPos3(ppt,p1);
                                //for debuging purposes
                                //attCd.set(ppt,UT_Vector3(1,0,0));
                                attAlpha.set(ppt,gridAlpha);
                                grpToDestroy->addOffset(ppt);
                            }
                        }
                        //---------------------- Dynamic Tau -----------------------------------
                        float dP0 = attDP0.get(ppt);
                        float dPi = distance3d(p1,trackerPosition);
                        attDPi.set(ppt,dPi);

                        float deltaOnD = abs(dPi-dP0)/poissonDiskD;
                        attDeltaOnD.set(ppt,deltaOnD);

                        averageDeltaOnD += deltaOnD;
                        if(maxDeltaOnD < deltaOnD)
                            maxDeltaOnD = deltaOnD;

                        attDP0.set(ppt,distance3d(p1,trackerPosition));
                        //--------------- Compute average position for the grid ----------------
                        if (first)
                        {
                            first = false;
                            averagePosition = p1;
                        }
                        else
                        {
                            averagePosition += p1;
                        }
                        nbOfPoint++;
                        //----------------------------------------------------------------------
                    }
                    if(nbOfPoint > 0)
                    {
                        averagePosition /= nbOfPoint;
                        this->gridCenterPosition[id] = averagePosition;
                    }
                }
                //if (active == 1)
                {
                    //cout << "Point "<<id<< " is active"<<endl;
                    DistortionMetricSorkine2002 distortionComputer;
                    bool distorted = distortionComputer.ComputeDistortion(this->trackersGdp,this->deformableGridsGdp,trackerPpt,pointGrp,primGroup,distortionParams);
                    if (distorted)
                        this->numberOfDistortedPatches++;
                }
            }
        }

        if (nbOfPoint == 0)
        {
            //No point in the patch, we need to delete the tracker.
            //this->numberOfLonelyTracker++;
            if (spawn <= 1)
                this->numberOfNewAndLonelyTracker++;
            else
                this->numberOfLonelyTracker++;
            attLife.set(trackerPpt,0);
            cout << "No point in the patch"<<endl;
            attActive.set(trackerPpt,0);

            continue;
        }
        averageDeltaOnD = averageDeltaOnD/(float)nbOfPoint;

        if (averageDeltaOnD > params.fadingTau)
            averageDeltaOnD = params.fadingTau;
        if (averageDeltaOnD < 0.0f )
            averageDeltaOnD = 0.0f;

        attMaxDeltaOnD.set(trackerPpt,averageDeltaOnD);
        //------------------------------------------------
        //delete too distorted primitives
        GEO_Primitive *prim;
        int numberOfPrimitives = 0;
        GA_FOR_ALL_GROUP_PRIMITIVES(this->deformableGridsGdp,primGroup,prim)
        {
            float qt = attQt.get(prim->getMapOffset());
            if (qt < 0.001)
                primGrpToDestroy->add(prim);
            else
                numberOfPrimitives++;
        }
        //------------------------------------------------------
    }

    if (primGrpToDestroy != 0x0)
    {
        cout << "Destroying groups"<<endl;
        this->deformableGridsGdp->destroyPrimitiveGroup(primGrpToDestroy);
    }
    this->deformableGridsGdp->deletePoints(*grpToDestroy,mode);
    //------------------------------------------------------


    this->FlagBoundaries();
    cout << "Number of lonely patches: "<<this->numberOfLonelyTracker<<endl;
    cout << "Ok"<<endl;

    this->gridAdvectionTime += (std::clock() - startAdvection) / (double) CLOCKS_PER_SEC;
}


//================================================================================================

//                                      UV FLATTENING

//================================================================================================
bool DeformableGridsManager::UVFlattening(GU_Detail &tempGdp,
                                   GA_Offset tracker, GA_Offset closestPoint,
                                   GA_PointGroup *pointGroup, GA_PointGroup *tempPointGroup,
                                   set<GA_Offset> &pointsAround,
                                   float scaling)
{
    GA_RWHandleI    attFlattening(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"flattening",1));
    GA_RWHandleI    attInitVertexId(tempGdp.addIntTuple(GA_ATTRIB_VERTEX,"initVerterxId",1));
    GA_RWHandleV3   attUV(this->deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,uvName, 3));
    GA_RWHandleV3   attTempVertexUV(tempGdp.addFloatTuple(GA_ATTRIB_VERTEX,"uv", 3));
    GA_RWHandleI    attIsTreated(this->deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"isTreated",1));
    GA_RWHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));

    int id = attId.get(tracker);

    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;
    attFlattening.set(tracker,1);
    std::clock_t startFlattening;
    startFlattening = std::clock();

    GEO_Primitive *prim;
    bool useInputUv = false;
    GU_Flatten flattener(&tempGdp,NULL,NULL,NULL,useInputUv);
    flattener.flattenAndPack();

    int numberOfIslands = flattener.getNumIslands();

    if (numberOfIslands == 0)
    {
        cout << "There is no possible island in the flattening."<<endl;
        return false;
    }

    if (numberOfIslands > 1)
    {
        //retry to flatten the uv
        set<GA_Offset> connectedOffset;
        if (params.testPatch == 1 && params.patchNumber == id)
        {
            cout << "number of island "<<numberOfIslands <<" retry to flatten the uv"<<endl;
        }

        if (closestPoint == -1)
        {
            cout << this->approachName << "UV Flattening - can't find closest point"<<endl;
            return false;
        }
    }
    UT_Vector3 uvCenter(0.5,0.5,0);
    int nbUv = 0;
    float ratioUv = 0;
    float ratioAverage = 0;

//    bool ratioComputed = false;
//    {
//        GA_FOR_ALL_PRIMITIVES(&tempGdp,prim)
//        {
//            GEO_Primitive *primitive = tempGdp.getGEOPrimitive(prim->getMapOffset());
//            int nbVertex = primitive->getVertexCount();
//            if (nbVertex < 3)
//                continue;

//            GA_Offset vertex;
//            GA_Offset initVertex;
//            if (!ratioComputed)
//            {
//                //-------------compute ratio ------------------
//                vertex = primitive->getVertexOffset(0);
//                initVertex = attInitVertexId.get(vertex);
//                UT_Vector3 uv1 = attTempVertexUV.get(vertex);
//                GA_Offset point1 = tempGdp.vertexPoint(vertex);
//                UT_Vector3 pos1 = tempGdp.getPos3(point1);

//                vertex = primitive->getVertexOffset(1);
//                UT_Vector3 uv2 = attTempVertexUV.get(vertex);
//                GA_Offset point2 = tempGdp.vertexPoint(vertex);
//                UT_Vector3 pos2 = tempGdp.getPos3(point2);

//                float d3d = distance3d(pos1,pos2);
//                float dUv = distance3d(uv1,uv2);
//                if (dUv == 0)
//                    continue;
//                ratioUv = d3d/dUv;

//                ratioAverage += ratioUv;
//                nbUv++;

//                //ratioComputed = true;
//            }
//        }
//    }

//    if (nbUv == 0)
//        nbUv = 1;
//    ratioUv = ratioAverage / nbUv;
//    nbUv = 0;
//    if (ratioUv == 0)
//        ratioUv = 1;
//    if (scaling == 0)
//        scaling = 1;

    GA_FOR_ALL_PRIMITIVES(&tempGdp,prim)
    {
        //---------------------------------------------
        GEO_Primitive *primitive = tempGdp.getGEOPrimitive(prim->getMapOffset());
        int nbVertex = primitive->getVertexCount();
        if (nbVertex < 3)
            continue;

        GA_Offset vertex;
        GA_Offset initVertex;
        //int vindex = nbVertex-1;
        for(int i = 0; i< nbVertex; i++)
        {
            vertex = primitive->getVertexOffset(i);
            initVertex = attInitVertexId.get(vertex);
            UT_Vector3 uv = attTempVertexUV.get(vertex);
//            uv /= 1/ratioUv;
//            uv /= scaling;
            uvCenter += uv;
            nbUv++;

            GA_Offset point = this->deformableGridsGdp->vertexPoint(initVertex);
            attIsTreated.set(point,1);
            //UT_Vector3 uv = attVertexUV.get(vertex);
            attUV.set(point,uv);
        }
    }

//    GA_PrimitiveGroup *primGroup = 0;
//    GA_Range range = (this->deformableGridsGdp)->getPrimitiveRange((primGroup));
//    GA_Iterator begin = range.begin();
//    GA_Iterator end = range.end();
//    GA_Iterator itTest(range);

//    for (GA_Iterator it((this->deformableGridsGdp)->getPrimitiveRange(primGroup)); (!it.atEnd() || (prim = nullptr)) &&
//            ((prim)=GA_Detail::GB_MACRO_CAST((this->deformableGridsGdp), (this->deformableGridsGdp)->getPrimitive(*it)));
//            ++it)
//    {
//        GEO_Primitive *primitive = this->deformableGridsGdp->getGEOPrimitive(prim->getMapOffset());
//        int nbVertex = primitive->getVertexCount();
//        if (nbVertex < 3)
//            continue;

//        //-------------compute ratio ------------------
//        GA_Offset vertex1 = primitive->getVertexOffset(0);
//        GA_Offset point1 = this->deformableGridsGdp->vertexPoint(vertex1);
//        UT_Vector3 uv1 = attUV.get(point1);
//        UT_Vector3 pos1 = this->deformableGridsGdp->getPos3(point1);

//        GA_Offset vertex2 = primitive->getVertexOffset(1);
//        GA_Offset point2 = this->deformableGridsGdp->vertexPoint(vertex2);
//        UT_Vector3 uv2 = attUV.get(point2);
//        UT_Vector3 pos2 = this->deformableGridsGdp->getPos3(point2);

//        float d3d = distance3d(pos1,pos2);
//        float dUv = distance3d(uv1,uv2);
//        ratioUv = d3d/dUv;
//        //---------------------------------------------

//        //int vindex = nbVertex-1;
//        for(int i = 0; i< nbVertex; i++)
//        {
//            GA_Offset vertex = primitive->getVertexOffset(i);
//            GA_Offset point = this->deformableGridsGdp->vertexPoint(vertex);

//            UT_Vector3 uv = attUV.get(point);

//            uvCenter += uv;
//            nbUv++;
//            //UT_Vector3 uv = attVertexUV.get(vertex);
//            //attUV.set(point,uv);
//        }
//    }

    if (params.testPatch == 1 && params.patchNumber == id)
    {
        if (nbUv == 0)
            cout << "There are no uv coordiantes." << endl;
    }
    //----------------- Center UV --------------------
    UT_Vector3 destCenter(0.5,0.5,0);
    if (nbUv != 0)
        uvCenter /= nbUv;

    //UT_Vector3 translation = destCenter - uvCenter;

    GA_PointGroup *toDestroy = this->deformableGridsGdp->newPointGroup("ToDestroy");
    GA_Offset ppt;
    GA_FOR_ALL_GROUP_PTOFF(this->deformableGridsGdp,pointGroup,ppt)
    {
        /*
        if (attIsTreated.get(ppt) == 0)
        {
            pointGroup->removeOffset(ppt);
            toDestroy->addOffset(ppt);
        }
        else
        {
        */
            UT_Vector3 uv = attUV.get(ppt);
            uv -= uvCenter;
            uv += destCenter;
            attUV.set(ppt,uv);
        //}
    }
    this->deformableGridsGdp->deletePoints(*toDestroy,mode);

    //-------------------------------------------------
    float time = (std::clock() - startFlattening) / (double) CLOCKS_PER_SEC;
    //cout << " in "<< time<<endl;
    this->uvFlatteningTime += time;
    this->nbOfFlattenedPatch++;
    return true;
}


void DeformableGridsManager::FlagBoundariesForPatch(GA_Offset ppt)
{

    int id = this->attId.get(ppt);
    //cout << "[DeformableGridsManager] add border attribute for patch"<<id<<endl;
    string str = std::to_string(id);
    string groupName = "grid"+str;
    GA_PrimitiveGroup *primitiveGroup = (GA_PrimitiveGroup *)this->deformableGridsGdp->primitiveGroups().find(groupName);
    if (primitiveGroup == 0x0)
    {
        cout << " Can't find prim group"<<endl;
        return;
    }
    GA_Primitive *prim;
    GA_RWHandleI    attBorder(this->deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"border",1));
    //cout << "For all primitives, add border attribute."<<endl;
    GA_FOR_ALL_GROUP_PRIMITIVES(this->deformableGridsGdp, primitiveGroup, prim)
    {
        int nb = prim->getVertexCount();
        if (nb != 3)
            continue;
        GA_Offset vertexA = prim->getVertexOffset(0);
        GA_Offset vertexB = prim->getVertexOffset(1);
        GA_Offset vertexC = prim->getVertexOffset(2);
        GA_Offset pointA = this->deformableGridsGdp->vertexPoint(vertexA);
        GA_Offset pointB = this->deformableGridsGdp->vertexPoint(vertexB);
        GA_Offset pointC = this->deformableGridsGdp->vertexPoint(vertexC);
        vector<GA_Offset> points;
        points.push_back(pointA);
        points.push_back(pointB);
        points.push_back(pointC);
        //int vertices[] = primvertices(0,@primnum);

        //edges to check
        int AB = 0;
        int AC = 0;
        int BC = 0;

        for(int i=0;i<nb;i++)
        {
            GA_Offset point = points[i];
            //int pointvertices[] = pointvertices(0,point);
            GA_OffsetArray primitives;
            this->deformableGridsGdp->getPrimitivesReferencingPoint(primitives,point);
            for(GA_OffsetArray::const_iterator prims_it = primitives.begin(); prims_it != primitives.end(); ++prims_it)
            {

                GEO_Primitive* nprim = this->deformableGridsGdp->getGEOPrimitive(*prims_it);
                if (nprim->getMapOffset() == prim->getMapOffset())
                    continue;

                //we are on a neighbour primitive
                //check if we have the same edge
                int nbn = nprim->getVertexCount();
                int A = 0;
                int B = 0;
                int C = 0;
                for(int k=0;k<nbn;k++)
                {
                    GA_Offset nvertex = nprim->getVertexOffset(k);
                    GA_Offset npoint = this->deformableGridsGdp->vertexPoint(nvertex);
                    if (npoint == pointA)
                        A = 1;
                    if (npoint == pointB)
                        B = 1;
                    if (npoint == pointC)
                        C = 1;
                }

                if (A == 1 && B == 1)
                    AB = 1;
                if (A == 1 && C == 1)
                    AC = 1;
                if (C == 1 && B == 1)
                    BC = 1;

            }
        }
        if (AB == 0 || AC == 0 || BC == 0)
        {
            for(int i=0;i<nb;i++)
            {
                GA_Offset point = points[i];
                attBorder.set(point,1);
            }

        }
    }
}

void DeformableGridsManager::FlagBoundaries()
{

    //cout << "[DeformableGridsManager] add border attribute."<<endl;
    GA_RWHandleI    attBorder(this->deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"border",1));

    GA_Primitive *prim;
    GA_FOR_ALL_PRIMITIVES(this->deformableGridsGdp, prim)
    {

        int nb = prim->getVertexCount();
        if (nb != 3)
            continue;
        GA_Offset vertexA = prim->getVertexOffset(0);
        GA_Offset vertexB = prim->getVertexOffset(1);
        GA_Offset vertexC = prim->getVertexOffset(2);
        GA_Offset pointA = this->deformableGridsGdp->vertexPoint(vertexA);
        GA_Offset pointB = this->deformableGridsGdp->vertexPoint(vertexB);
        GA_Offset pointC = this->deformableGridsGdp->vertexPoint(vertexC);
        vector<GA_Offset> points;
        points.push_back(pointA);
        points.push_back(pointB);
        points.push_back(pointC);
        //int vertices[] = primvertices(0,@primnum);

        //edges to check
        int AB = 0;
        int AC = 0;
        int BC = 0;

        for(int i=0;i<nb;i++)
        {
            GA_Offset point = points[i];
            //int pointvertices[] = pointvertices(0,point);
            GA_OffsetArray primitives;
            this->deformableGridsGdp->getPrimitivesReferencingPoint(primitives,point);
            for(GA_OffsetArray::const_iterator prims_it = primitives.begin(); prims_it != primitives.end(); ++prims_it)
            {

                GEO_Primitive* nprim = this->deformableGridsGdp->getGEOPrimitive(*prims_it);
                if (nprim->getMapOffset() == prim->getMapOffset())
                    continue;

                //we are on a neighbour primitive
                //check if we have the same edge
                int nbn = nprim->getVertexCount();
                int A = 0;
                int B = 0;
                int C = 0;
                for(int k=0;k<nbn;k++)
                {
                    GA_Offset nvertex = nprim->getVertexOffset(k);
                    GA_Offset npoint = this->deformableGridsGdp->vertexPoint(nvertex);
                    if (npoint == pointA)
                        A = 1;
                    if (npoint == pointB)
                        B = 1;
                    if (npoint == pointC)
                        C = 1;
                }

                if (A == 1 && B == 1)
                    AB = 1;
                if (A == 1 && C == 1)
                    AC = 1;
                if (C == 1 && B == 1)
                    BC = 1;

            }
        }
        if (AB == 0 || AC == 0 || BC == 0)
        {
            for(int i=0;i<nb;i++)
            {
                GA_Offset point = points[i];
                attBorder.set(point,1);
            }

        }
    }

}
