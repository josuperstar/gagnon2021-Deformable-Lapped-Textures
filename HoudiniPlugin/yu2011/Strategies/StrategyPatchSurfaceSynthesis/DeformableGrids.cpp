#include "DeformableGrids.h"

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

#include <Core/HoudiniUtils.h>


DeformableGrids::DeformableGrids()
{
    this->numberOfPatches = 0;
    this->maxId = 0;
    this->gridCenterPosition.clear();

    this->uvFlatteningTime = 0;
    this->gridMeshCreation = 0;
    this->gridAdvectionTime = 0;
    this->markerAdvectionTime = 0;

    this->nbOfFlattenedPatch = 0;

    this->numberOfConcealedPatches = 0;
    this->numberOfNewPatches = 0;
    this->numberOfDetachedPatches = 0;

}

bool DeformableGrids::SynthesisSurface(GU_Detail *gdp, ParametersDeformablePatches params)
{
    return true;
}

//================================================================================================

//                                       ADD DEFORMABLE GRID

//================================================================================================

void DeformableGrids::CreateGridBasedOnMesh(GU_Detail *deformableGridsGdp,GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params, vector<GA_Offset> trackers,  GEO_PointTreeGAOffset &tree)
{
    //cout << "Yu 2011 Create grids"<<endl;

    //Yu2011 Section 3.3.1
    //For each particle, we create a regular grid (see Fig. 2, left), centered on it, of width larger than 2d. Combined
    //with the properties of the boundary sampling algorithm, this guarantees a gap-less coverage of the fluid.

    //Section 3.3.2
    //At their creation, grids are slightly larger than kernel size, to avoid triggering condition 1 too early. We create
    //grids of width (2 + β)d, with a small β (in our implementation, β = 0.6). The size of the grid is a compromise
    //between particle lifetime and the number of vertices it will require to ensure a given resolution

    float beta = 0.6f;
    float d = params.poissondiskradius;
    float gridwidth = (2+beta)*d;

    //cout << this->approachName<<" creating grids based on mesh"<<endl;
    //cout << "beta "<<beta<<endl;
    //cout << "d "<<d<<endl;
    //cout << "gridwidth "<<gridwidth<<endl;

    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));

    GA_RWHandleF    attTrackerLife(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"life",1));

    GA_RWHandleV3   attUV(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,uvName, 3));
    GA_RWHandleF    attAlpha(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"Alpha",1));
    GA_RWHandleI    attIsGrid(deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"isGrid",1));
    GA_RWHandleF    attInitArea(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"initArea",1));
    GA_RWHandleI    attInitId(deformableGridsGdp->addIntTuple(GA_ATTRIB_PRIMITIVE,"initId",1));
    GA_RWHandleI    attIsTreated(deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"isTreated",1));
    GA_RWHandleV3   attV(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   attCd(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));
    GA_RWHandleI    attGridId(deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));

    GA_RWHandleF    attAlpha0(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,alpha0Name,1));
    GA_RWHandleF    attVW(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,vertexWeightName,1));
    GA_RWHandleF    attW(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,distortionWeightName,1));
    GA_RWHandleF    attW0(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,distortionWeight0Name,1));

    GA_RWHandleV3   attSs(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"Ss",3));
    GA_RWHandleV3   attSt(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"St",3));
    GA_RWHandleF    attDMax(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"dmax",1));
    GA_RWHandleF    attDMin(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"dmin",1));
    GA_RWHandleF    attDistortion(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"distortion",1));
    GA_RWHandleF    attQt(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"Qt",1));

    GA_RWHandleF    attA(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"a",1));
    GA_RWHandleF    attB(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"b",1));
    GA_RWHandleF    attC(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"c",1));
    GA_RWHandleF    attArea(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"area",1));

    GA_RWHandleF    attDP0(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"dP0",1));

    GA_RWHandleV3 attRP(deformableGridsGdp->addFloatTuple(GA_ATTRIB_VERTEX,"refPosition",3));
    GA_RWHandleF attVA(deformableGridsGdp->addFloatTuple(GA_ATTRIB_VERTEX,initialVertexAngle,1));
    GA_RWHandleF attLife(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleI attPrimLife(deformableGridsGdp->addIntTuple(GA_ATTRIB_PRIMITIVE,"life",1));
    //GA_RWHandleF attTMin(gdp->addFloatTuple(GA_ATTRIB_POINT,disrortionMinThreshold,1));
    //GA_RWHandleF attTMax(gdp->addFloatTuple(GA_ATTRIB_POINT,distortionMaxThreshold,1));

    GA_RWHandleV3 attNSurface(surfaceGdp->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleV3 attVSurface(surfaceGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));

    GA_Offset ppt;
    UT_Vector3 TrackerN;
    UT_Vector3 N;
    UT_Vector3 v;
    UT_Vector3 trackerPositition;
    GA_Offset newPoint;
    UT_Vector3Array         triangleArrayData;
    float life = 1.0f;

    string groupName = "grids";
    GA_PointGroup *grpGrid = deformableGridsGdp->newPointGroup(groupName.c_str());

    float scaling = gridwidth;

    //cout << "go through all trackers"<<endl;
    vector<GA_Offset>::iterator it;
    int id =0;
    for(it = trackers.begin(); it != trackers.end(); it++)
    {
        ppt = *it;
        id = attId.get(ppt);

        if (params.testPatch == 1 && params.patchNumber != id)
            continue;

        life = attTrackerLife.get(ppt);
        GU_Detail tempGdp;
        set<GA_Offset> tempGdpListOffset;
        GA_Offset tempNewPoint;
        GA_RWHandleI attInitVertexId(tempGdp.addIntTuple(GA_ATTRIB_VERTEX,"initVerterxId",1));
        GA_RWHandleV3 attTempVertexUV(tempGdp.addFloatTuple(GA_ATTRIB_VERTEX,"uv", 3));

        TrackerN = attN.get(ppt);
        UT_Vector3 p = trackersGdp->getPos3(ppt);
        UT_Vector3 pt = trackersGdp->getPos3(ppt+1);

        TrackerN.normalize();
        UT_Vector3 T = pt-p;
        T.normalize();
        UT_Vector3 S = cross(TrackerN,T);
        S.normalize();

        std::clock_t startMeshCreation;
        startMeshCreation = std::clock();

        string str = std::to_string(id);
        string groupName = "grid"+str;
        GA_PointGroup *pointGroup = deformableGridsGdp->newPointGroup(groupName.c_str());
        GA_PointGroup *tempPointGroup = tempGdp.newPointGroup(groupName.c_str());
        GA_PrimitiveGroup *primGroup = deformableGridsGdp->newPrimitiveGroup(groupName.c_str());

        trackerPositition = trackersGdp->getPos3(ppt);
        set<GA_Offset> primList;
        vector<GA_Offset> pointList;
        vector<GA_Offset> tempPointList;

        GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;
        tree.findAllCloseIdx(trackerPositition,
                             gridwidth/2.0f,
                             close_particles_indices);

        GA_Offset surfaceClosestPoint = tree.findNearestIdx(trackerPositition);
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
                //--------create new points ------------
                newPoint = deformableGridsGdp->appendPointOffset();
                tempNewPoint = tempGdp.appendPointOffset();
                tempGdpListOffset.insert(tempNewPoint);

                if (surfaceClosestPoint == neighbor)
                {
                    closestPoint = tempNewPoint;
                }

                UT_Vector3 pos = surfaceGdp->getPos3(neighbor);
                deformableGridsGdp->setPos3(newPoint,surfaceGdp->getPos3(neighbor));
                tempGdp.setPos3(tempNewPoint,surfaceGdp->getPos3(neighbor));
                pointGroup->addOffset(newPoint);
                tempPointGroup->addOffset(tempNewPoint);
                pointsAround.insert(tempNewPoint);

                grpGrid->addOffset(newPoint);

                attIsGrid.set(newPoint,1);
                attAlpha.set(newPoint,(float)life/(float)params.fadingTau);
                attVW.set(newPoint,0);
                attAlpha0.set(newPoint,(float)life/(float)params.fadingTau);
                attLife.set(newPoint,params.fadingTau);
                attV.set(newPoint,v);
                attIsTreated.set(newPoint,0);
                attCd.set(newPoint,UT_Vector3(1,1,1));
                attGridId.set(newPoint,id);

                pointList.push_back(newPoint);
                tempPointList.push_back(tempNewPoint);
                pointsLink[neighbor] = newPoint;
                tempPointsLink[neighbor] = tempNewPoint;

                //------------- UV Orthogonal Projection ---------------

                // Transform into local patch space (where STN is aligned with XYZ at the origin)
                const UT_Vector3 relativePosistion = pos-p;
                UT_Vector3 triangleSpacePos;
                triangleSpacePos.x() = relativePosistion.dot(S);
                triangleSpacePos.y() = relativePosistion.dot(T);
                triangleSpacePos.z() = relativePosistion.dot(TrackerN);

                UT_Vector3 uv;
                uv.x() = triangleSpacePos.x();
                uv.y() = triangleSpacePos.y();
                uv.z() = 0;

                float mid = 0.5;
                uv /= scaling;
                uv += mid;
                if (attUV.isValid())
                    attUV.set(newPoint,uv);

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
                surfaceGdp->getPrimitivesReferencingPoint(primitives,neighbor);
                for(GA_OffsetArray::const_iterator prims_it = primitives.begin(); prims_it != primitives.end(); ++prims_it)
                {
                    bool add = true;
                    //we need to be sure that the primitives have all its points in the point list selected before
                    GEO_Primitive* prim = surfaceGdp->getGEOPrimitive(*prims_it);
                    int nbVertex = prim->getVertexCount();
                    for(int i = 0; i<nbVertex; i++)
                    {
                        GA_Offset vertex = prim->getVertexOffset(i);
                        GA_Offset point = surfaceGdp->vertexPoint(vertex);
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
            GEO_PrimPoly *prim_poly_ptr = (GEO_PrimPoly *)deformableGridsGdp->appendPrimitive(GA_PRIMPOLY);
            prim_poly_ptr->setSize(0);

            GEO_PrimPoly *temp_prim_poly_ptr = (GEO_PrimPoly *)tempGdp.appendPrimitive(GA_PRIMPOLY);
            temp_prim_poly_ptr->setSize(0);

            GEO_Primitive* prim = surfaceGdp->getGEOPrimitive(*itPrim);
            //cout << "Creating prim "<<*itPrim<<endl;
            int nbVertex = prim->getVertexCount();
            map<GA_Offset,GA_Offset>::iterator m;
            for(int i = 0; i < nbVertex; i++)
            {
                GA_Offset vertex = prim->getVertexOffset(i);
                GA_Offset point = surfaceGdp->vertexPoint(vertex);

                trianglePoints.push_back(surfaceGdp->getPos3(point));

                //------------- VERTEX ANGLE ----------------
                int b = i-1;
                if (b < 0)
                    b = nbVertex-1;
                int c = i+1;
                if (c >= nbVertex)
                    c = 0;

                GA_Offset vB = prim->getVertexOffset(b);
                GA_Offset pB = surfaceGdp->vertexPoint(vB);
                GA_Offset vC = prim->getVertexOffset(c);
                GA_Offset pC = surfaceGdp->vertexPoint(vC);

                UT_Vector3 AB = surfaceGdp->getPos3(pB)-surfaceGdp->getPos3(point);
                UT_Vector3 AC = surfaceGdp->getPos3(pC)-surfaceGdp->getPos3(point);
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
            vector<UT_Vector3> triangleRefPositions;
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


            //======================== JULIAN MAGIC HERE =========================
            /*
            UT_Vector3 p1 = UT_Vector3(0.0902777,-0.176871,0);//triangleSpacePosA;
            UT_Vector3 p2 = UT_Vector3(0.0902777,0.0884352,0);//triangleSpacePosB;
            UT_Vector3 p3 = UT_Vector3(-0.18055,0.0884352,0);//triangleSpacePosC;
            */

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

            float area = ((s2 - s1)*(t3-t1) - (s3-s1)*(t2-t1))/2.0f;
            attArea.set(prim_poly_ptr->getMapOffset(),area);

            //cout << "s1 "<<s1<< " s2 "<<s2 << " s3 "<<s3 << " t1 "<<t1 << " t2 "<<t2 << " t3 "<<t3<<endl;
            //cout << "q1 "<<q1<<" q2 "<<q2 << " q3 "<<q3<<endl;
            //cout << "area "<<area<<endl;

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
                attQt.set(prim_poly_ptr->getMapOffset(),1.0f);
            }
            //====================================================================
            //=====================================================================================

            primGroup->addOffset(prim_poly_ptr->getMapOffset());
        }

        //cout << "compute areas"<<endl;
        this->gridMeshCreation += (std::clock() - startMeshCreation) / (double) CLOCKS_PER_SEC;

        if (close_particles_count == 0)
            continue;

        GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;
        //if ( (float)nbDistorted/(float)close_particles_count > 0.2)
        //if ( nbDistorted > 1)
        //=====================================================================================
        //--------------------- UV FLATENING-------------------
        bool flattening = true;
        if (flattening)
        {
            this->UVFlattening(tempGdp, trackersGdp, deformableGridsGdp, ppt, closestPoint, pointGroup, tempPointGroup, pointsAround, scaling );

        }//======================================END UV FLATENING===============================================


        //--------------------------------------------------
        //Take a random part of the input texture uv space
        //- compute a random translation
        //- scale up

        int seed = id;
        float offset = 0.5;
        float randomScale = 1.0;
        srand(seed);
        float tx = (((double) rand()/(RAND_MAX)))*randomScale;
        srand(seed+1);
        float ty = (((double) rand()/(RAND_MAX)))*randomScale;
        srand(seed+2);
        float tz = (((double) rand()/(RAND_MAX)))*randomScale;

        float scaleup = 2.0;

        GA_Offset ppt;
        GA_FOR_ALL_GROUP_PTOFF(deformableGridsGdp,pointGroup,ppt)
        {
            UT_Vector3 uv = attUV.get(ppt);
            uv += UT_Vector3(tx,ty,tz);
            uv /= scaleup;
            attUV.set(ppt,uv);
        }

        //-----------------------------------------------------
        GEO_Primitive *prim;
        float area;
        GA_FOR_ALL_GROUP_PRIMITIVES(deformableGridsGdp,primGroup,prim)
        {
            area = prim->calcArea();
            attInitArea.set(prim->getMapOffset(),area);
        }

        if(pointGroup->entries() == 0)
        {
            //cout << "Was not able to add points in "<<pointGroup->getName()<<endl;
            //delete prim point and prim group
            deformableGridsGdp->deletePoints(*pointGroup,mode);
            deformableGridsGdp->destroyPointGroup(pointGroup);
            deformableGridsGdp->destroyPrimitiveGroup(primGroup);
            DeleteTracker(trackersGdp,id);

        }
        //cout << "done"<<endl;
    }
}

//================================================================================================

//                                      ADVECT MARKERS

//================================================================================================


void DeformableGrids::AdvectGrids(GU_Detail *deformableGridsgdp, GU_Detail *trackersGdp, ParametersDeformablePatches params,GEO_PointTreeGAOffset &tree, vector<GA_Offset> trackers, GU_Detail *surfaceGdp)
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

    GA_RWHandleV3   attV(deformableGridsgdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3   attN(deformableGridsgdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleV3   attCd(deformableGridsgdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));
    GA_RWHandleF    attAlpha(deformableGridsgdp->addFloatTuple(GA_ATTRIB_POINT,"Alpha",1));
    GA_RWHandleI    attGridId(deformableGridsgdp->addIntTuple(GA_ATTRIB_POINT,"id",1));

    GA_RWHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleI    attActive(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"active",1));
    GA_RWHandleF    attLife(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleF    attRandT(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,randomThresholdDistortion,1));
    GA_RWHandleF    attMaxDeltaOnD(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"maxDeltaOnD",1));

    GA_RWHandleV3 refAttV(surfaceGdp->findFloatTuple(GA_ATTRIB_POINT,"v", 3));
    GA_RWHandleV3 refAttN(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));


    GA_RWHandleF    attDP0(deformableGridsgdp->addFloatTuple(GA_ATTRIB_POINT,"dP0",1));
    GA_RWHandleF    attDPi(deformableGridsgdp->addFloatTuple(GA_ATTRIB_POINT,"dPi",1));
    GA_RWHandleF    attDeltaOnD(deformableGridsgdp->addFloatTuple(GA_ATTRIB_POINT,"deltaOnD",1));

    if (attV.isInvalid())
    {
        cout << this->approachName << " Grids have no velocity"<<endl;
        return;
    }

    UT_Vector3 v;
    UT_Vector3 p;
    UT_Vector3 p1;
    float dt = 1.0f/24.0f;
    float thresholdDistance = params.maximumProjectionDistance;
    float poissonDiskD = params.poissondiskradius;

    ParametersDistortion distortionParams;
    distortionParams.dilatationMin              = params.dilatationMin;
    distortionParams.dilatationMax              = params.dilatationMax;
    distortionParams.squeezeMin                 = params.squeezeMin;
    distortionParams.squeezeMax                 = params.squeezeMax;
    distortionParams.deletionLife               = params.fadingTau;
    distortionParams.distortionRatioThreshold   = params.distortionRatioThreshold ;
    distortionParams.Yu2011DMax                 = params.Yu2011DMax ;
    distortionParams.QvMin                      = params.QvMin;


    distortionParams.alphaName = "Alpha";
    distortionParams.temporalRemoveName = "temporalRemove";
    distortionParams.randomThresholdDistortion = randomThresholdDistortion;
    distortionParams.initialVertexAngle = initialVertexAngle;
    distortionParams.distortionWeightName = distortionWeightName;
    distortionParams.primLifeName = "life";

    GA_PointGroup * grpToDestroy;
    grpToDestroy = (GA_PointGroup *)deformableGridsgdp->newPointGroup("GridPointToDelete");

    GU_MinInfo mininfo;
    GU_RayIntersect ray(surfaceGdp);
    ray.init();

    GA_GroupType groupType = GA_GROUP_POINT;
    const GA_GroupTable *gtable = deformableGridsgdp->getGroupTable(groupType);
    GA_GroupType primGroupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *gPrimTable = deformableGridsgdp->getGroupTable(primGroupType);

    GA_PointGroup* pointGrp;
    vector<GA_Offset>::iterator it;
    UT_Vector3 N;
    GA_Offset ppt;
    GA_Offset trackerPpt;
    int id;
    int active;
    float life;
    set<int> toDelete;
    set<int> patchAdvected;


    for(it = trackers.begin(); it != trackers.end(); ++it)
    {
        trackerPpt = *it;
        id = attId.get(trackerPpt);

        if (params.testPatch == 1 && params.patchNumber != id)
            continue;

        active = attActive.get(trackerPpt);
        float randT = attRandT.get(trackerPpt);
        distortionParams.randT = randT;

        life = attLife.get(trackerPpt);

        float gridAlpha = (float)life/(float)params.fadingTau;

        UT_Vector3 trackerPosition = trackersGdp->getPos3(trackerPpt);

        patchAdvected.insert(id);

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
                GA_FOR_ALL_GROUP_PTOFF(deformableGridsgdp,pointGrp,ppt)
                {
                    //cout << this->approachName<<" "<< "Deleting "<<id<<endl;
                    grpToDestroy->addOffset(ppt);
                    attCd.set(ppt,UT_Vector3(0,0,1));
                    //change the alpha of the deformable grid:
                    attAlpha.set(ppt,gridAlpha);
                }
                toDelete.insert(id);
            }
            else
            {
                {
                    bool first = true;
                    UT_Vector3 averagePosition;

                    GA_FOR_ALL_GROUP_PTOFF(deformableGridsgdp,pointGrp,ppt)
                    {
                        //check if it is a lonely point
                        GA_OffsetArray primitivesList;
                        GA_Size numberOfPrimitives = deformableGridsgdp->getPrimitivesReferencingPoint(primitivesList,ppt);
                        if (numberOfPrimitives == 0)
                        {
                            grpToDestroy->addOffset(ppt);
                            continue;
                        }
                        //toAdd = true;
                        v = attV.get(ppt);
                        N = attN.get(ppt);
                        p = deformableGridsgdp->getPos3(ppt);

                        //advect
                        UT_Vector3 d = v*dt;
                        p1 = p+d;
                        deformableGridsgdp->setPos3(ppt,p1);

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
                                deformableGridsgdp->setPos3(ppt,p1);

                                //------------------------------PARAMETRIC COORDINATE -----------------------------------
                                GA_Offset primOffset = mininfo.prim->getMapOffset();
                                float u = mininfo.u1;
                                float v = mininfo.v1;
                                GEO_Primitive *prim = surfaceGdp->getGEOPrimitive(primOffset);

                                GA_Offset vertexOffset0 = prim->getVertexOffset(0);
                                //UT_Vector3 v0 = attUV.get(vertexOffset0);
                                GA_Offset pointOffset0  = surfaceGdp->vertexPoint(vertexOffset0);
                                UT_Vector3 n0 = refAttN.get(pointOffset0);
                                UT_Vector3 v0 = refAttV.get(pointOffset0);


                                GA_Offset vertexOffset1 = prim->getVertexOffset(1);
                                GA_Offset pointOffset1  = surfaceGdp->vertexPoint(vertexOffset1);
                                UT_Vector3 n1 = refAttN.get(pointOffset1);
                                UT_Vector3 v1 = refAttV.get(pointOffset1);
                                //UT_Vector3 v1 = attUV.get(vertexOffset1);


                                GA_Offset vertexOffset2 = prim->getVertexOffset(2);
                                GA_Offset pointOffset2  = surfaceGdp->vertexPoint(vertexOffset2);
                                UT_Vector3 n2 = refAttN.get(pointOffset2);//gdp->getPos3(pointOffset3);
                                UT_Vector3 v2 = refAttV.get(pointOffset2);;//gdp->getPos3(pointOffset2);
                                //UT_Vector3 v2 = attUV.get(vertexOffset2);

                                UT_Vector3 normal   = n0+u*(n1-n0)+v*(n2-n0);
                                UT_Vector3 velocity = v0+u*(v1-v0)+v*(v2-v0);
                                attV.set(ppt,velocity);
                                attN.set(ppt,normal);
                                attAlpha.set(ppt,gridAlpha);
                                //------------------------------------------------------------------------------------

                            }
                            else
                            {
                                deformableGridsgdp->setPos3(ppt,p1);
                                //for debuging purposes
                                attCd.set(ppt,UT_Vector3(1,0,0));
                                attAlpha.set(ppt,gridAlpha);
                                //since the vertex of the grid is not projected on the surface, we should delete it
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
                    Yu2011Distortion distortionComputer;
                    distortionComputer.ComputeDistortion(trackersGdp,deformableGridsgdp,trackerPpt,pointGrp,primGroup,distortionParams);
                }
            }
            //---------------------------------------------------------------------
        }

        if (nbOfPoint == 0)
            continue;
        averageDeltaOnD = averageDeltaOnD/(float)nbOfPoint;

        if (averageDeltaOnD > params.fadingTau)
            averageDeltaOnD = params.fadingTau;
        if (averageDeltaOnD < 0.0f )
            averageDeltaOnD = 0.0f;

        //attMaxDeltaOnD.set(trackerPpt,maxDeltaOnD);
        attMaxDeltaOnD.set(trackerPpt,averageDeltaOnD);
        //cout << "Max Delta on D "<<maxDeltaOnD<<endl;
    }


    //GA_GroupType groupType = GA_GROUP_POINT;
    const GA_GroupTable *pointGtable = deformableGridsgdp->getGroupTable(groupType);
    //GA_GroupType primGroupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *primitiveGtable = deformableGridsgdp->getGroupTable(primGroupType);
    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;

    GA_PrimitiveGroup *primGroup;
    //delete grid without trackers
    GA_FOR_ALL_PRIMGROUPS(deformableGridsgdp,primGroup)
    {
        //"grid" has 4 letters
        string groupName = primGroup->getName().toStdString();
        string groupString = groupName;
        groupString.replace(0,4,"");

        int patchId = std::stoi( groupString );
        //cout << "Is patch "<<patchId<<" exists? Then we should delete "<<groupName<<endl;
        if (patchAdvected.count(patchId) == 0)
        {
            //delete this grid
            pointGrp = (GA_PointGroup*)pointGtable->find(groupName.c_str());
            deformableGridsgdp->deletePoints(*pointGrp,mode);
            deformableGridsgdp->destroyPointGroup(pointGrp);
            deformableGridsgdp->destroyPrimitiveGroup(primGroup);
        }
    }

    deformableGridsgdp->deletePoints(*grpToDestroy,mode);

    //GA_PointGroup* pointGrp;
    GA_PrimitiveGroup* primitiveGrp;
    int patchId;
    set<int>::iterator itGrid;
    for(itGrid = toDelete.begin(); itGrid != toDelete.end(); ++itGrid)
    {
        patchId = *itGrid;
        string str = std::to_string(patchId);
        string groupName = "grid"+str;
        pointGrp = (GA_PointGroup*)pointGtable->find(groupName.c_str());
        primitiveGrp = (GA_PrimitiveGroup*)primitiveGtable->find(groupName.c_str());
        if (pointGrp)
        {
            //gdp->deletePoints(*pointGrp,mode);
            deformableGridsgdp->destroyPointGroup(pointGrp);
            deformableGridsgdp->destroyPrimitiveGroup(primitiveGrp);
        }
    }

    this->gridAdvectionTime += (std::clock() - startAdvection) / (double) CLOCKS_PER_SEC;
}

//================================================================================================

//                                      CONNECTIVITY TEST

//================================================================================================

void DeformableGrids::ConnectivityTest(const GU_Detail *gdp, GA_Offset point,GA_PointGroup *grp,  set<GA_Offset> &pointsAround,set<GA_Offset> &group)
{

    if (grp == 0x0)
        return;

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

            int found = pointsAround.count(pointOffset);
            if(found > 0 && group.count(pointOffset) == 0)
            {
                group.insert(pointOffset);
                ConnectivityTest(gdp,pointOffset,grp, pointsAround,group);
            }
        }
    }
}


//================================================================================================

//                                      UV FLATTENING

//================================================================================================
void DeformableGrids::UVFlattening(GU_Detail &tempGdp, GU_Detail *trackersGdp, GU_Detail *deformableGridsGdp,
                                   GA_Offset tracker, GA_Offset closestPoint,
                                   GA_PointGroup *pointGroup, GA_PointGroup *tempPointGroup,
                                   set<GA_Offset> &pointsAround,
                                   float scaling)
{
    GA_RWHandleI    attFlattening(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"flattening",1));
    GA_RWHandleI    attInitVertexId(tempGdp.addIntTuple(GA_ATTRIB_VERTEX,"initVerterxId",1));
    GA_RWHandleV3   attUV(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,uvName, 3));
    GA_RWHandleV3   attTempVertexUV(tempGdp.addFloatTuple(GA_ATTRIB_VERTEX,"uv", 3));
    GA_RWHandleI    attIsTreated(deformableGridsGdp->addIntTuple(GA_ATTRIB_POINT,"isTreated",1));

    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;
    attFlattening.set(tracker,1);
    std::clock_t startFlattening;
    startFlattening = std::clock();

    GEO_Primitive *prim;
    bool useInputUv = false;
    GU_Flatten flattener(&tempGdp,NULL,NULL,NULL,useInputUv);
    flattener.flattenAndPack();

    int numberOfIslands = flattener.getNumIslands();

    if (numberOfIslands > 1)
    {
        set<GA_Offset> connectedOffset;

        if (closestPoint == -1)
        {
            cout << "can't find closest point"<<endl;
            return;
        }
        //cout << "closest point = "<<closestPoint<<endl;
        ConnectivityTest(&tempGdp,closestPoint,tempPointGroup,pointsAround,connectedOffset);

        GA_PointGroup *toDestroy = tempGdp.newPointGroup("ToDestroy");
        GA_PointGroup *tempToKeep = tempGdp.newPointGroup("ToKeep");

        set<GA_Offset>::iterator itG;
        for(itG = connectedOffset.begin(); itG != connectedOffset.end(); ++itG)
        {
            tempToKeep->addOffset(*itG);
        }
        int toDestroyCount = 0;
        {
            GA_Offset ppt2;

            GA_FOR_ALL_GROUP_PTOFF(&tempGdp,tempPointGroup,ppt2)
            {
                if (!tempToKeep->containsOffset(ppt2))
                {
                    toDestroy->addOffset(ppt2);
                    toDestroyCount++;
                }
            }
        }

        tempGdp.deletePoints(*toDestroy,mode);

        GU_Flatten flattener2(&tempGdp,NULL,NULL,NULL,useInputUv);
        flattener2.flattenAndPack();
        numberOfIslands = flattener2.getNumIslands();

        if (numberOfIslands > 1)
        {
            cout << "The flattening is not workking with conenctivity test"<<endl;
            return;
        }

    }
    UT_Vector3 uvCenter(0.5,0.5,0);
    int nbUv = 0;
    float ratioUv = 0;
    float ratioAverage = 0;

    bool ratioComputed = false;
    {
        GA_FOR_ALL_PRIMITIVES(&tempGdp,prim)
        {
            GEO_Primitive *primitive = tempGdp.getGEOPrimitive(prim->getMapOffset());
            int nbVertex = primitive->getVertexCount();
            if (nbVertex < 3)
                continue;

            GA_Offset vertex;
            GA_Offset initVertex;
            if (!ratioComputed)
            {
                //-------------compute ratio ------------------
                vertex = primitive->getVertexOffset(0);
                initVertex = attInitVertexId.get(vertex);
                UT_Vector3 uv1 = attTempVertexUV.get(vertex);
                GA_Offset point1 = tempGdp.vertexPoint(vertex);
                UT_Vector3 pos1 = tempGdp.getPos3(point1);

                vertex = primitive->getVertexOffset(1);
                UT_Vector3 uv2 = attTempVertexUV.get(vertex);
                GA_Offset point2 = tempGdp.vertexPoint(vertex);
                UT_Vector3 pos2 = tempGdp.getPos3(point2);

                float d3d = distance3d(pos1,pos2);
                float dUv = distance3d(uv1,uv2);
                ratioUv = d3d/dUv;

                ratioAverage += ratioUv;
                nbUv++;

                //ratioComputed = true;
            }
        }
    }

    ratioUv = ratioAverage / nbUv;
    nbUv = 0;

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
            uv /= 1/ratioUv;
            uv /= scaling;
            //uvCenter += uv;
            //nbUv++;

            GA_Offset point = deformableGridsGdp->vertexPoint(initVertex);
            attIsTreated.set(point,1);
            //UT_Vector3 uv = attVertexUV.get(vertex);
            attUV.set(point,uv);
        }
    }

    GA_PrimitiveGroup *primGroup;
    //rescale uv on every primitive
    GA_FOR_ALL_GROUP_PRIMITIVES(deformableGridsGdp,primGroup,prim)
    {
        GEO_Primitive *primitive = deformableGridsGdp->getGEOPrimitive(prim->getMapOffset());
        int nbVertex = primitive->getVertexCount();
        if (nbVertex < 3)
            continue;

        //-------------compute ratio ------------------
        GA_Offset vertex1 = primitive->getVertexOffset(0);
        GA_Offset point1 = deformableGridsGdp->vertexPoint(vertex1);
        UT_Vector3 uv1 = attUV.get(point1);
        UT_Vector3 pos1 = deformableGridsGdp->getPos3(point1);

        GA_Offset vertex2 = primitive->getVertexOffset(1);
        GA_Offset point2 = deformableGridsGdp->vertexPoint(vertex2);
        UT_Vector3 uv2 = attUV.get(point2);
        UT_Vector3 pos2 = deformableGridsGdp->getPos3(point2);

        float d3d = distance3d(pos1,pos2);
        float dUv = distance3d(uv1,uv2);
        ratioUv = d3d/dUv;
        //---------------------------------------------

        //int vindex = nbVertex-1;
        for(int i = 0; i< nbVertex; i++)
        {
            GA_Offset vertex = primitive->getVertexOffset(i);
            GA_Offset point = deformableGridsGdp->vertexPoint(vertex);

            UT_Vector3 uv = attUV.get(point);

            uvCenter += uv;
            nbUv++;
            //UT_Vector3 uv = attVertexUV.get(vertex);
            //attUV.set(point,uv);
        }
    }

    //----------------- Center UV --------------------
    UT_Vector3 destCenter(0.5,0.5,0);
    uvCenter /= nbUv;

    UT_Vector3 translation = destCenter - uvCenter;

    GA_PointGroup *toDestroy = deformableGridsGdp->newPointGroup("ToDestroy");
    GA_Offset ppt;
    GA_FOR_ALL_GROUP_PTOFF(deformableGridsGdp,pointGroup,ppt)
    {
        if (attIsTreated.get(ppt) == 0)
        {
            pointGroup->removeOffset(ppt);
            toDestroy->addOffset(ppt);
        }
        else
        {
            UT_Vector3 uv = attUV.get(ppt);
            uv += translation;
            attUV.set(ppt,uv);
        }
    }
    deformableGridsGdp->deletePoints(*toDestroy,mode);

    //-------------------------------------------------
    float time = (std::clock() - startFlattening) / (double) CLOCKS_PER_SEC;
    //cout << " in "<< time<<endl;
    this->uvFlatteningTime += time;
    this->nbOfFlattenedPatch++;
}


