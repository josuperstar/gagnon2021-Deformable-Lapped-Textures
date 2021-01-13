#include "AtlasTestingConcealed.h"
#include <GU/GU_PrimPoly.h>
//#include "TBBAtlas.h"

#include "BlendingTestingConcealed.h"
#include "BlendingAnimatedTexture.h"
#include "../HoudiniUtils.h"

#include <Core/Gagnon2020/PatchedSurface.h>

#include <iostream>
#include <iomanip>


std::string format_account_number_concealed(int acct_no) {
  ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(4) << acct_no;
  return out.str();
}

AtlasTestingConcealed::~AtlasTestingConcealed()
{
    cout << "Destroy Atlas Testing Concealed"<<endl;

    for (int i = 0; i < this->numberOfTextureSampleFrame-1; i++)
    {
        //cout << "delete texture examplar memory "<<i<<endl;
        delete this->textureExemplars[i];
    }

    delete diffuseImageBlendingYu2011Equation4;
    delete displacementMapImage;

    trackerNormal.clear();

}

bool AtlasTestingConcealed::initImages(int w, int h)
{

    diffuseImageBlendingYu2011Equation4 = new ImageCV();
    diffuseImageBlendingYu2011Equation4->CreateImage(w,h,-1);


    std::size_t found = textureExemplar1Name.find("$F");
    if (found !=std::string::npos)
    {
        //cout << "texture example is a list"<<endl;
        // Here we need to replace $F with the sequence of texture exemplar between 1 and NumberOfTextureSampleFrame
        for (int i = 0; i < this->numberOfTextureSampleFrame; i++)
        {
            ImageCV *image = new ImageCV();
            textureExemplars.push_back(image);
            //string currentName = std::regex_replace(textureExemplar1Name, std::regex("\\$F"), std::to_string(i));
            string currentName = textureExemplar1Name;
            // HACK !!!!
            // Does not work with padding, ex : 0001, 0002..

            string paddedNumber = format_account_number_concealed(i+1);
            string noPaddedNumber = std::to_string(i+1);

            currentName.replace(currentName.find("$F"), sizeof("$F") - 1, paddedNumber);
            //cout << "[AtlasTestingConcealed::BuildAtlas] Opening "<<currentName<<endl;
            bool opened = textureExemplars[i]->OpenImage(currentName,-1);
            if (!opened)
            {
                cout << "[AtlasTestingConcealed::BuildAtlas] Can't open "<< currentName<<endl;
                return false;
            }

        }
    }
    else
    {
        //cout << "[AtlasTestingConcealed::BuildAtlas] Opening single texture examplar "<<textureExemplar1Name<<endl;
        ImageCV *image = new ImageCV();
        textureExemplars.push_back(image);
        bool opened = textureExemplars[0]->OpenImage(textureExemplar1Name,-1);
        if (!opened)
        {
            cout << "[AtlasTestingConcealed::BuildAtlas] Can't open "<< textureExemplar1Name<<endl;
            return false;
        }
    }
    if (displacementMapImageName.size() != 0)
    {
        displacementMapImage = new ImageCV();
        //cout << "[AtlasAnimatedTexture::BuildAtlas] Opening "<<displacementMapImageName<<endl;
        computeDisplacement = displacementMapImage->OpenImage(displacementMapImageName,-1);
        cout << "[AtlasAnimatedTexture::BuildAtlas] Done"<<endl;
        if (computeDisplacement)
        {
            displacementMapEquation3 = new ImageCV();
            displacementMapEquation3->CreateImage(w,h,-1);
            displacementMapEquation4 = new ImageCV();
            displacementMapEquation4->CreateImage(w,h,-1);
        }
        else
        {
            cout << "[AtlasAnimatedTexture::BuildAtlas] Can't open displacement map file "<<displacementMapImageName<<endl;
        }
    }
    imageInitied = true;
}

bool AtlasTestingConcealed::initVariables(int life)
{
    if (surface == 0x0 || (deformableGrids == 0x0 && useDeformableGrids) || trackers == 0x0)
        return false;


    surfaceGroup = (GA_PointGroup *)surface->pointGroups().find(surfaceGroupName.c_str());



    //cout << "[AtlasTestingConcealed::BuildAtlas] setting varialbes"<<endl;
    //-------------------------------------------------------
    UT_String patchname("patchIds");
    patchIds = surface->findIntArray(GA_ATTRIB_POINT,patchname,-1, -1);
    if (!patchIds)
    {
        cout << "[AtlasTestingConcealed::BuildAtlas] There is no patch id attribute"<<endl;
        return false;
    }
    patchArray = patchIds->getAIFNumericArray();
    //-------------------------------------------------------

    attLife = life;

    attBlend = GA_RWHandleF(trackers->findFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    attCenterUV = GA_RWHandleV3(trackers->addFloatTuple(GA_ATTRIB_POINT,"centerUV", 3));
    attPointUV = GA_RWHandleV3(deformableGrids->findFloatTuple(GA_ATTRIB_POINT,"uvw", 3));
    attAlpha = GA_ROHandleF(deformableGrids->findFloatTuple(GA_ATTRIB_POINT,"Alpha", 1));
    pointGroupTable = deformableGrids->getGroupTable(pointGroupType);
    primGroupTable = deformableGrids->getGroupTable(primGroupType);
    //cout << "[AtlasTestingConcealed::BuildAtlas] Atlas uses deformable grids"<<endl;

    attUV = GA_RWHandleV3(surface->findFloatTuple(GA_ATTRIB_VERTEX,"uv", 3));
    if (attUV.isInvalid())
    {
        cout << "[AtlasTestingConcealed::BuildAtlas] There is no uv on the surface"<<endl;
        return false;
    }

    if (textureExemplar1Name.size() == 0)
    {
        cout << "[AtlasTestingConcealed::BuildAtlas] There is no texture exemplar name assigned"<<endl;
        return false;
    }
    return true;
}

void AtlasTestingConcealed::buildRayList()
{
    GA_PrimitiveGroup *primGroup;
    vector<string> groupNames;
    rays.clear();
    //cout << "[AtlasTestingConcealed::BuildAtlas] Create map of RayIntersect using deformable grids."<<endl;
    GA_FOR_ALL_PRIMGROUPS(deformableGrids,primGroup)
    {
        //GA_PrimitiveGroup *primGroup = (GA_PrimitiveGroup*)gPrimTable->find(groupName.c_str());
        string name = primGroup->getName().toStdString();

        // This version of the constructor is like the one that takes a group
        // except the group is from a different gdp that has the same topology
        // (ie. prim counts are the sames). Note that if usevisibility is true,
        // then the visibility from the gdp is used (NOT limit_gdp).
        //GU_RayIntersect(const GU_Detail *gdp,
        //        const GU_Detail *limit_gdp,
        //        const GA_PrimitiveGroup *vis_group,
         //       bool picking = false, bool polyline = false, bool harden = false,
        //       bool usevisibility = false);

        GU_RayIntersect *ray = new GU_RayIntersect(deformableGrids,primGroup);

        ray->init(deformableGrids,primGroup);
        rays[name] = ray;
        //deformableGrids->destroyPrimitiveGroup(name.c_str());
    }
}

void AtlasTestingConcealed::buildArrayList()
{
    GA_RWHandleI    attId(trackers->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleV3   attN(trackers->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_Offset ppt;
    //cout << "[AtlasTestingConcealed::BuildAtlas] There is " << trackers->getNumPoints() << " trackers" << endl;

    temporalComponetKt.clear();
    trackerUVPosition.clear();
    trackerNormal.clear();
    trackersPosition.clear();
    usePatches.clear();
    GA_FOR_ALL_PTOFF(trackers,ppt)
    {
        float blend = attBlend.get(ppt);
        int patchId =   attId.get(ppt);

        if (isinf(blend))
            blend = 1.0f;
        temporalComponetKt[patchId] = blend;
        trackerUVPosition[patchId] = attCenterUV.get(ppt);
        trackerNormal[patchId] = attN.get(ppt);
        trackersPosition[patchId] = trackers->getPos3(ppt);
        usePatches[patchId] = false;
    }
}

bool AtlasTestingConcealed::BuildAtlas(int w, int h, int life)
{
    cout << "[AtlasTestingConcealed::BuildAtlas]("<<w<<","<< h <<")"<<endl;
    if (!initVariables(life))
        return false;

    if (!imageInitied)
    {
        if (!initImages(w,h))
            return false;
    }


    buildRayList();
    buildArrayList();

    if(renderColoredPatches)
        initPatchColors(trackers);

    this->pixelUsed.clear();
    for(int i =0; i < w; i++)
    {
        vector<bool> line;
        for(int j =0; j < h; j++)
        {
            line.push_back(false);
        }
        this->pixelUsed.push_back(line);
    }

    return true;

}

//================================= CREATE LIST OF GEO GU_DETAIL =================================
void AtlasTestingConcealed::CreateListGUDetails()
{
    GA_PrimitiveGroup *primGroup;

    GA_RWHandleV3   attNRef(deformableGrids->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_FOR_ALL_PRIMGROUPS(deformableGrids,primGroup)
    {
        map<GA_Offset,GA_Offset>::iterator m;
        GU_Detail *geoGdp = new GU_Detail();
        GA_RWHandleV3   attN(geoGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
        string name = primGroup->getName().toStdString();
        map<GA_Offset,GA_Offset> initialOffset;

        GA_PointGroup *pointGrp = (GA_PointGroup*)pointGroupTable->find(name.c_str());
        GA_Offset ppt;
        {
            //cout << "Working with point group "<<pointGrp->getName()<<endl;
            GA_FOR_ALL_GROUP_PTOFF(deformableGrids,pointGrp,ppt)
            {
                GA_Offset newPoint = geoGdp->appendPoint();
                pointsList[ppt] = newPoint;

                UT_Vector3 newPosition = deformableGrids->getPos3(ppt);
                geoGdp->setPos3(newPoint,newPosition);
                initialOffset[newPoint] = ppt;
            }
        }

        initialOffsetList[name] = initialOffset;

        GEO_Primitive* prim;
        {
            GA_FOR_ALL_GROUP_PRIMITIVES(deformableGrids,primGroup,prim)
            {
                int nbVertex = prim->getVertexCount();
                GEO_PrimPoly *prim_poly_ptr = (GEO_PrimPoly *)geoGdp->appendPrimitive(GA_PRIMPOLY);
                prim_poly_ptr->setSize(0);

                for(int i = 0; i < nbVertex; i++)
                {
                    GA_Offset vertex = prim->getVertexOffset(i);
                    GA_Offset point = deformableGrids->vertexPoint(vertex);
                    m = pointsList.find(point);
                    if (m != pointsList.end())
                    {
                        prim_poly_ptr->appendVertex(pointsList[point]);
                    }
                }
                prim_poly_ptr->close();
            }
        }
        patchesGeo[name] = geoGdp;
        GU_RayIntersect *ray = new GU_RayIntersect(geoGdp);
        ray->init();
        rays[name] = ray;
    }
}

//================================= RASTERIZE PRIMITIVE =================================

vector<GA_Offset> AtlasTestingConcealed::RasterizePrimitive(PatchedSurfaceGagnon2020 &patchedSurface, GA_Offset primOffset, int w, int h,  ParametersDeformablePatches params)
{
    vector<GA_Offset> newPoints;
    GA_Primitive *prim = surface->getPrimitive(primOffset);
    if(prim == 0x0)
        return newPoints;


    Pixel displacementSumEq3 = Pixel(0,0,0);
    Pixel displacementSumEq4 = Pixel(0,0,0);
    Pixel R_eq3;

    GEO_Primitive *geoPrim = surface->getGEOPrimitive(primOffset);
    UT_Vector3 N = geoPrim->computeNormal();

    //get triangle vertex position in UV space
    GA_Size vertexCount = prim->getVertexCount();
    if (vertexCount != 3)
    {
        cout << "Primitive "<<prim->getMapOffset()<< " has "<<vertexCount<< " vertices"<<endl;
        return newPoints;
    }

    GA_GroupType primitiveGroupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *primitiveGTable = deformableGrids->getGroupTable(primitiveGroupType);

    //--------------------- SORTED DATA PER PATCH ------------------------
    // put this in surfacePatched class:
    vector<UT_Vector3> surfaceTexturePosition;
    vector<UT_Vector3> surfaceUv;
    vector<UT_Vector3> surfacePosition;
    vector<int> sortedPatches;
    map<int,int> numberOfLinkedPatch;

    //-------------initializinb maps------------------
    for(int i = 0; i < vertexCount; i++)
    {
        GA_Offset vertexPoint = prim->getVertexOffset(i);
        GA_Offset pointOffset = surface->vertexPoint(vertexPoint);

        UT_IntArray         patchesData;
        patchArray->get(patchIds, pointOffset, patchesData);

        vector<UT_Vector3> uvTemp;
        uvTemp.resize(3);
        vector<float> alphaTemp;
        alphaTemp.resize(3);
        int nb = patchesData.size();
        for (int k = 0; k< nb; k++)
        {
            int index = patchesData.array()[k];
            numberOfLinkedPatch[index] = 0;
        }
    }

    //------------filling maps---------------
    for(int vertexIt = 0; vertexIt < vertexCount; vertexIt++)
    {
        GA_Offset vertexPoint = prim->getVertexOffset(vertexIt);
        GA_Offset pointOffset = surface->vertexPoint(vertexPoint);

        UT_Vector3 uv = attUV.get(vertexPoint);
        surfaceUv.push_back(uv);
        uv.x() *= w;
        uv.y() *= h;
        surfaceTexturePosition.push_back(uv);

        surfacePosition.push_back(surface->getPos3(pointOffset));

        //put this in another function
        UT_IntArray         patchesData;
        patchArray->get(patchIds, pointOffset, patchesData);

        //for this vertex, we go through all patches
        //we are trying to keep only patches that are on the three vertices
        //vector<UT_Vector3> sortedUVs;
        int nb = patchesData.size();
        for (int patchIndex = 0; patchIndex< nb; patchIndex++)
        {

            int patchId = patchesData.array()[patchIndex];
            int data = numberOfLinkedPatch[patchId];
            data++;
            numberOfLinkedPatch[patchId] = data;
            if (data == 3)
            {
                //cout << "w00t !"<<endl;
                sortedPatches.push_back(patchId);
            }
        }
    }
    //-------------------------------------------------------------------

    UT_Vector3 min, max;
    BoundingBox2D(surfaceTexturePosition[0],surfaceTexturePosition[1],surfaceTexturePosition[2],min,max);

    int pixelCellSize = 5;
    Pixel color;
    Pixel displacement;

    UT_Vector3 point;

    GA_RWHandleV3   attN(trackers->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    //-----------------------------------------------------------------
    for(int i =min.x()-pixelCellSize; i < max.x()+pixelCellSize; i++)
    {
        for(int j =min.y()-pixelCellSize; j < max.y()+pixelCellSize; j++)
        {
            if (i < 0 || j < 0)
                continue;

            Pixel Cf = Pixel(0,0,0);
            Cf.A = 1;

            Pixel R_eq3;

            Pixel colorSum0 = Pixel(0,0,0);
            colorSum0.A = 1;

            Pixel colorSum1 = Pixel(0,0,0);
            colorSum1.A = 1;

            Pixel colorSum2 = Pixel(0,0,0);
            colorSum2.A = 1;

            color = Pixel(0,0,0);
            color.A = 1;

            displacement = Pixel(0,0,0);
            Pixel displacementSumEq3 = Pixel(0,0,0);
            Pixel displacementSumEq4 = Pixel(0,0,0);

            point.x() = i;
            point.y() = j;
            point.z() = 0;

            int pixelPositionX = i;
            int pixelPositionY = j;

            while (pixelPositionX >= w)
                pixelPositionX -= w;
            while (pixelPositionY >= h)
                pixelPositionY -= h;
            while (pixelPositionX < 0)
                pixelPositionX += w;
            while (pixelPositionY < 0)
                pixelPositionY += h;

            bool notUsedYet = !this->pixelUsed[pixelPositionX][pixelPositionY];
            //cout << "not used yet "<<notUsedYet<<endl;
            UT_Vector3 a = surfaceTexturePosition[0];
            UT_Vector3 b = surfaceTexturePosition[1];
            UT_Vector3 c = surfaceTexturePosition[2];

            bool inTriangle = IsPointInTriangle(point,a,b,c);

            if (inTriangle && notUsedYet)
            {
                //test color
                color.R = 1;
                color.G = 1;
                color.B = 1;
                //======================== Yu2011  function =====================
                //Pixel R_eq4 = BlendingTestingConcealed::Blend(deformableGrids,i,j,w,h,
//                Pixel R_eq4 = BlendingAnimatedTexture::Blend(deformableGrids,i,j,w,h,
//                                          pixelPositionX,pixelPositionY,
//                                          sortedPatches,
//                                          surfaceUv,
//                                          surfacePosition,
//                                          trackerNormal,
//                                          trackersPosition,
//                                          trackerUVPosition,
//                                          usePatches,
//                                          rays,
//                                          attPointUV,
//                                          temporalComponetKt,
//                                          textureExemplars,
//                                          params);
                //cout << "Testing concealed patch"<<endl;
                Pixel R_eq4 = BlendingAnimatedTexture::Blend(deformableGrids,i,j,w,h,
                                          pixelPositionX,pixelPositionY,
                                          sortedPatches,
                                          surfaceUv,
                                          surfacePosition,
                                          trackerNormal,
                                          trackersPosition,
                                          trackerUVPosition,
                                          usePatches,
                                          rays,
                                          patchColors,
                                          RM,
                                          attPointUV,
                                          temporalComponetKt,
                                          textureExemplars,
                                          displacementMapImage,
                                          false,
                                          renderColoredPatches,
                                          R_eq3,
                                          displacementSumEq3,
                                          displacementSumEq4,
                                          params);
                //cout << "Resulting color: "<<R_eq4.R << " "<<R_eq4.G << " "<<R_eq4.B << " "<<R_eq4.A <<endl;
                bool addPatchOnRasterization = true;

                //========================== ADD A NEW PATCH ========================
//                GU_RayIntersect ray(this->deformableGrids);
//                ray.init();

                //Need another patch
                UT_Vector3 pixelPositionOnSurface;

                pixelPositionOnSurface.x() = ((float)pixelPositionX/(w-1));
                pixelPositionOnSurface.y() = ((float)pixelPositionY/(h-1));
                pixelPositionOnSurface.z() = 0;

                UT_Vector3 positionOnSurface = HoudiniUtils::GetBarycentricPosition(surfaceUv[0],surfaceUv[1],surfaceUv[2],surfacePosition[0],surfacePosition[1],surfacePosition[2],pixelPositionOnSurface);


                if (R_eq4.A < 0.1 && addPatchOnRasterization)
                {
                    //patchedSurface.CreateDebugRasterizationPoint(positionOnSurface, UT_Vector3(R_eq4.R,R_eq4.G,R_eq4.B),R_eq4.A, 1);
                    //1- Add particle
                    //Here, we should have a patch id as the result
                    //cout << "Create new paticle with position "<<positionOnSurface<< " and normal "<<N<<endl;
                    GA_Offset newPoint = patchedSurface.CreateAParticle( positionOnSurface, N);
                    newPoints.push_back(newPoint);
                    //patchedSurface.ProjectAndUpdateTracker(newPoint);
                    bool canProject = patchedSurface.ProjectTrackerOnSurface(newPoint);
                    if (!canProject)
                        continue;
                    patchedSurface.UpdateTracker(newPoint);

                    patchedSurface.CreateGridBasedOnMesh(newPoint);
                    patchedSurface.AddDeformablePatcheUsingBarycentricCoordinates(newPoint);

                    //----------------------- Rebuild data structure------------------------
                    GA_RWHandleI    attId(trackers->findIntTuple(GA_ATTRIB_POINT,"id",1));

                    int newPointId = attId.get(newPoint);
                    string str = std::to_string(newPointId);
                    string groupName = "grid"+str;
                    GA_PrimitiveGroup*  gridPrimitiveGroup  = (GA_PrimitiveGroup*)primitiveGTable->find(groupName);
                    GU_RayIntersect *new_ray = new GU_RayIntersect(deformableGrids,gridPrimitiveGroup);
                    new_ray->init(deformableGrids,gridPrimitiveGroup);
                    rays[groupName] = new_ray;

                    sortedPatches.push_back(newPointId);
                    float blend = attBlend.get(newPoint);
                    if (isinf(blend))
                        blend = 1.0f;
                    temporalComponetKt[newPointId] = blend;
                    trackerUVPosition[newPointId] = attCenterUV.get(newPoint);
                    trackerNormal[newPointId] = attN.get(newPoint);
                    trackersPosition[newPointId] = trackers->getPos3(newPoint);


                    Pixel secondTestTexel = BlendingAnimatedTexture::Blend(deformableGrids,i,j,w,h,
                                              pixelPositionX,pixelPositionY,
                                              sortedPatches,
                                              surfaceUv,
                                              surfacePosition,
                                              trackerNormal,
                                              trackersPosition,
                                              trackerUVPosition,
                                              usePatches,
                                              rays,
                                              patchColors,
                                              RM,
                                              attPointUV,
                                              temporalComponetKt,
                                              textureExemplars,
                                              displacementMapImage,
                                              false,
                                              renderColoredPatches,
                                              R_eq3,
                                              displacementSumEq3,
                                              displacementSumEq4,
                                              params);


                    if (secondTestTexel.A < 0.1 && addPatchOnRasterization)
                    {
                        GA_RWHandleF attTLife(trackers->findFloatTuple(GA_ATTRIB_POINT,"life",1));
                        GA_RWHandleI attActive(trackers->findIntTuple(GA_ATTRIB_POINT,"active",1));
                        attActive.set(newPoint,0);
                        attTLife.set(newPoint,0);

                    }
                    else
                        usePatches[newPointId] = true;
                    //cout << "sorted patch "<<newPointId<<endl;
                }
                else
                {
                    //patchedSurface.CreateDebugRasterizationPoint(positionOnSurface, UT_Vector3(R_eq4.R,R_eq4.G,R_eq4.B),R_eq4.A, 0);
                }
                if (inTriangle)
                    this->pixelUsed[pixelPositionX][pixelPositionY] = true;
                diffuseImageBlendingYu2011Equation4->SetColor(pixelPositionX,h-pixelPositionY,0,R_eq4);
            }
        }
    }//------------------------ FIN RASTERISATION ---------------------
    return newPoints;
}


void AtlasTestingConcealed::SaveAtlas()
{
    //write the image to the disk
    diffuseImageBlendingYu2011Equation4->growRegions(diffuseImageBlendingYu2011Equation4->image,diffuseImageBlendingYu2011Equation4->image,1); //
    diffuseImageBlendingYu2011Equation4->SaveImageAs(outputFilename+".yu2011equationfour.padded.png");
    cout << "Save texture atlas"<<outputFilename<<endl;
    if (computeDisplacement)
    {
        //write the image to the disk
        displacementMapEquation4->growRegions(displacementMapEquation4->image,displacementMapEquation4->image,1); //
        displacementMapEquation4->SaveImageAs(outputFilename+".displacementequationfour.padded.png");

        cout << "Save texture atlas"<<outputFilename+"displacement.png"<<endl;
    }
}

void AtlasTestingConcealed::BoundingBox2D(UT_Vector3 a, UT_Vector3 b, UT_Vector3 c,UT_Vector3 &min,UT_Vector3 &max)
{
    min.x() = a.x();
    min.y() = a.y();
    //min.z = a.z;
    max = min;

    // check min

    if (min.x() > b.x())
    {
        min.x() = b.x();
    }
    if (min.y() > b.y())
    {
        min.y() = b.y();
    }

    if (min.x() > c.x())
    {
        min.x() = c.x();
    }
    if (min.y() > c.y())
    {
        min.y() = c.y();
    }

    //check max

    if (max.x() < b.x())
    {
        max.x() = b.x();
    }
    if (max.y() < b.y())
    {
        max.y() = b.y();
    }

    if (max.x() < c.x())
    {
        max.x() = c.x();
    }
    if (max.y() < c.y())
    {
        max.y() = c.y();
    }

}

bool AtlasTestingConcealed::IsPointInTriangle(UT_Vector3  p, UT_Vector3 a,UT_Vector3 b,UT_Vector3 c)
{
    UT_Vector3 v0 = c - a;
    UT_Vector3 v1 = b - a;
    UT_Vector3 v2 = p - a;

    // Compute dot products
    float dot00 = dot(v0,v0);
    float dot01 = dot(v0,v1);
    float dot02 = dot(v0,v2);
    float dot11 = dot(v1,v1);
    float dot12 = dot(v1,v2);

    // Compute barycentric coordinates
    float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check if point is in triangle
    return (u >= 0) && (v >= 0) && (u + v <= 1);
}


Pixel AtlasTestingConcealed::SetRandomColor(int patchNumber)
{
    //initialize random seed
    srand(patchNumber);
    float r = ((double) rand()/(RAND_MAX));
    srand(patchNumber+1);
    float g = ((double) rand()/(RAND_MAX));
    srand(patchNumber+2);
    float b = ((double) rand()/(RAND_MAX));
    Pixel patchColor;
    patchColor.A = 1;
    patchColor.R = r;
    patchColor.G = g;
    patchColor.B = b;

    return patchColor;
}

void AtlasTestingConcealed::initPatchColors(GU_Detail *trackersGdp)
{

    GA_ROHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        int patchId = attId.get(ppt);
        patchColors[patchId] = SetRandomColor(patchId);
    }

}


void AtlasTestingConcealed::CleanRayMemory(GU_Detail *deformableGrids)
{

    map<string,GU_RayIntersect*>::iterator itRays;
    for(itRays=rays.begin(); itRays != rays.end(); itRays++)
    {
         string name = (itRays)->first;
         delete rays[name];
    }

}
