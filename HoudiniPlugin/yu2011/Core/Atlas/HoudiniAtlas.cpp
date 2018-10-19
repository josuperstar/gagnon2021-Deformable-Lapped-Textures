#include "HoudiniAtlas.h"
#include <GU/GU_PrimPoly.h>
//#include "TBBAtlas.h"

#include "BlendingYu2011.h"
#include "../HoudiniUtils.h"

/*
HoudiniAtlas::~HoudiniAtlas()
{

    if (this->diffuseImageBlendingGagnon->IsValid())
        delete this->diffuseImageBlendingGagnon;

    if (this->diffuseImageBlendingYu2011Equation3->IsValid())
        delete this->diffuseImageBlendingYu2011Equation3;

    if (this->diffuseImageBlendingYu2011Equation4->IsValid())
        delete this->diffuseImageBlendingYu2011Equation4;
    if (this->textureExemplar1Image->IsValid())
        delete this->textureExemplar1Image;
    if (this->textureExemplar1ImageMask->IsValid())
        delete this->textureExemplar1ImageMask;
    if (computeDisplacement)
    {
        delete this->displacementMapImage;
        delete this->displacementMap;
    }

    if (useDeformableGrids)
    {
        GA_PrimitiveGroup *primGroup;
        GA_FOR_ALL_PRIMGROUPS(deformableGrids,primGroup)
        {
             string name = primGroup->getName().toStdString();
             delete rays[name];
             delete patchesGeo[name];
        }
        //patchesGeo.clear();
    }
    trackerPosition.clear();

}
*/

bool HoudiniAtlas::BuildAtlas(int w, int h, int life)
{
    if (surface == 0x0 || (deformableGrids == 0x0 && useDeformableGrids) || trackers == 0x0)
        return false;

    cout << "[HoudiniAtlas::BuildAtlas]("<<w<<","<< h <<")"<<endl;

    cout << "[HoudiniAtlas::BuildAtlas] setting varialbes"<<endl;
    //-------------------------------------------------------
    UT_String patchname("patchIds");
    patchIds = surface->findIntArray(GA_ATTRIB_POINT,patchname,-1, -1);
    if (!patchIds)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no patch id attribute"<<endl;
        return false;
    }
    patchArray = patchIds->getAIFNumericArray();
    //-------------------------------------------------------

    //-------------------------------------------------------
    UT_String aname("alphas");
    alphas = surface->findFloatArray(GA_ATTRIB_POINT,aname,-1, -1);
    if (!alphas)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no alpha array attribute"<<endl;
        return false;
    }
    alphaArray = alphas->getAIFNumericArray();

    //-------------------------------------------------------
    UT_String uvname("uvs");
    uvsAtt = surface->findFloatArray(GA_ATTRIB_POINT, uvname,-1, -1);
    if (!uvsAtt)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no uvs array attribute"<<endl;
        return false;
    }
    uvsArray = uvsAtt->getAIFNumericArray();
    //--------------------------------------------------------------------------

    //---------------------------------------------------------------

    surfaceTree.build(surface, NULL);
    attLife = life;

    attBlend = GA_RWHandleF(trackers->findFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    GA_RWHandleV3   attCenterUV(trackers->addFloatTuple(GA_ATTRIB_POINT,"centerUV", 3));


    gridTree.build(deformableGrids,NULL);
    attPointUV = GA_RWHandleV3(deformableGrids->findFloatTuple(GA_ATTRIB_POINT,"uvw", 3));
    attAlpha = GA_ROHandleF(deformableGrids->findFloatTuple(GA_ATTRIB_POINT,"Alpha", 1));
    pointGroupTable = deformableGrids->getGroupTable(pointGroupType);
    primGroupTable = deformableGrids->getGroupTable(primGroupType);
    cout << "[HoudiniAtlas::BuildAtlas] Atlas uses deformable grids"<<endl;



    attUV = GA_RWHandleV3(surface->findFloatTuple(GA_ATTRIB_VERTEX,"uv", 3));
    if (attUV.isInvalid())
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no uv on the surface"<<endl;
        return false;
    }

    if (textureExemplar1Name.size() == 0)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no texture exemplar name assigned"<<endl;
        return false;
    }
    if (textureExemplar1MaskName.size() == 0)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no texture exemplar mask name assigned"<<endl;
        return false;
    }

    diffuseImageBlendingGagnon = new ImageCV();
    diffuseImageBlendingGagnon->CreateImage(w,h,-1);

    diffuseImageBlendingYu2011Equation3 = new ImageCV();
    diffuseImageBlendingYu2011Equation3->CreateImage(w,h,-1);

    diffuseImageBlendingYu2011Equation4 = new ImageCV();
    diffuseImageBlendingYu2011Equation4->CreateImage(w,h,-1);

    textureExemplar1Image = new ImageCV();
    cout << "[HoudiniAtlas::BuildAtlas] Opening "<<textureExemplar1Name<<endl;
    bool opened = textureExemplar1Image->OpenImage(textureExemplar1Name,-1);
    if (!opened)
    {
        cout << "[HoudiniAtlas::BuildAtlas] Can't open "<< textureExemplar1Name<<endl;
        return false;
    }

    RM = textureExemplar1Image->MeanValue();
    cout << "RM = ";
    RM.Print();
    cout<<endl;

    textureExemplar1ImageMask = new ImageCV();
    cout << "[HoudiniAtlas::BuildAtlas] Opening "<<textureExemplar1MaskName<<endl;
    opened = textureExemplar1ImageMask->OpenImage(textureExemplar1MaskName,-1);
    if (!opened)
    {
        cout << "[HoudiniAtlas::BuildAtlas] Can't open "<< textureExemplar1MaskName<<endl;
        return false;
    }

    if (displacementMapImageName.size() != 0)
    {
        displacementMapImage = new ImageCV();
        cout << "[HoudiniAtlas::BuildAtlas] Opening "<<displacementMapImageName<<endl;
        computeDisplacement = displacementMapImage->OpenImage(displacementMapImageName,-1);
        cout << "[HoudiniAtlas::BuildAtlas] Done"<<endl;
        if (computeDisplacement)
        {
            displacementMap = new ImageCV();
            displacementMap->CreateImage(w,h,-1);
        }
        else
        {
            cout << "[HoudiniAtlas::BuildAtlas] Can't open displacement map file "<<displacementMapImageName<<endl;
        }
    }
    else
    {
        cout << "[HoudiniAtlas::BuildAtlas] Displacement map name is not defined"<<endl;
    }

    if(useCopyGUDetail)
    {
        CreateListGUDetails();
    }
    else
    {
        GA_PrimitiveGroup *primGroup;

        if (useDeformableGrids)
        {
            GA_FOR_ALL_PRIMGROUPS(deformableGrids,primGroup)
            {
                //GA_PrimitiveGroup *primGroup = (GA_PrimitiveGroup*)gPrimTable->find(groupName.c_str());
                string name = primGroup->getName().toStdString();
                GU_RayIntersect *ray = new GU_RayIntersect(deformableGrids,primGroup);
                ray->init();
                rays[name] = ray;
            }
        }
        else
        {
            GA_FOR_ALL_PRIMGROUPS(surface,primGroup)
            {
                //GA_PrimitiveGroup *primGroup = (GA_PrimitiveGroup*)gPrimTable->find(groupName.c_str());
                string name = primGroup->getName().toStdString();
                GU_RayIntersect *ray = new GU_RayIntersect(surface,primGroup);
                ray->init();
                rays[name] = ray;
            }
        }
    }


    GA_RWHandleI    attId(trackers->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_Offset ppt;
    cout << "[HoudiniAtlas::BuildAtlas] There is " << trackers->getNumPoints() << " trackers" << endl;
    GA_FOR_ALL_PTOFF(trackers,ppt)
    {
        float blend = attBlend.get(ppt);
        int patchId =   attId.get(ppt);

        if (isinf(blend))
            blend = 1.0f;
        temporalComponetKt[patchId] = blend;
        trackerUVPosition[patchId] = attCenterUV.get(ppt);
    }

    if(renderColoredPatches)
        initPatchColors(trackers);


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
void HoudiniAtlas::CreateListGUDetails()
{
    GA_PrimitiveGroup *primGroup;

    GA_RWHandleV3   attNRef(deformableGrids->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_FOR_ALL_PRIMGROUPS(deformableGrids,primGroup)
    {


        //map<GA_Offset,GA_Offset> pointsListTest;
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
                //attN.set(newPoint,attNRef.get(ppt));

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
                        GA_Size idx = prim_poly_ptr->appendVertex(pointsList[point]);
                    }
                }
                prim_poly_ptr->close();
            }
        }
        //cout << "Creating temp geo "<<name<<endl;
        patchesGeo[name] = geoGdp;
        GU_RayIntersect *ray = new GU_RayIntersect(geoGdp);
        ray->init();
        rays[name] = ray;

    }
}

//================================= RASTERIZE PRIMITIVE =================================

void HoudiniAtlas::RasterizePrimitive(GA_Offset primOffset, int w, int h,ParametersDeformablePatches params)
{
    GA_Primitive *prim = surface->getPrimitive(primOffset);
    if(prim == 0x0)
        return;

    //get triangle vertex position in UV space
    GA_Size vertexCount = prim->getVertexCount();
    if (vertexCount != 3)
    {
        cout << "Primitive "<<prim->getMapOffset()<< " has "<<vertexCount<< " vertices"<<endl;
        return;
    }

    //--------------------- SORTED DATA PER PATCH ------------------------
    vector<UT_Vector3> surfaceTexturePosition;
    vector<UT_Vector3> surfaceUv;
    vector<UT_Vector3> surfacePosition;
    vector<int> sortedPatches;

    map<int,int> numberOfLinkedPatch;
    map<int, vector<UT_Vector3> > patchUvs;
    map<int, vector<float> > alphasMap;

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

        UT_FloatArray         uvsData;
        uvsArray->get(uvsAtt, pointOffset, uvsData);

        UT_FloatArray         alphasData;
        alphaArray->get(alphas, pointOffset, alphasData);

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
    Pixel alphaColor;
    UT_Vector3 point;

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
            Pixel displacementSum = Pixel(0,0,0);

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

            if (IsPointInTriangle(point,surfaceTexturePosition[0],surfaceTexturePosition[1],surfaceTexturePosition[2]))// || !this->pixelUsed[pixelPositionX][pixelPositionY]  )
            {
                //test color
                color.R = 1;
                color.G = 1;
                color.B = 1;
                //======================== Yu2011  function =====================
                Pixel R_eq4 = BlendingYu2011::Blend(deformableGrids,i,j,w,h,
                                          pixelPositionX,pixelPositionY,
                                          sortedPatches,
                                          surfaceUv,
                                          surfacePosition,
                                          trackerPosition,
                                          trackerUVPosition,
                                          //useDeformableGrids,
                                          rays,
                                          patchColors,
                                          //alphaColor,
                                          RM,
                                          //attAlpha,
                                          attPointUV,
                                          //attLife,
                                          temporalComponetKt,
                                          //patchUvs,
                                          //alphasMap,
                                          textureExemplar1Image,
                                          //textureExemplar1ImageMask,
                                          displacementMapImage,
                                          computeDisplacement,
                                          renderColoredPatches,
                                          R_eq3,
                                          displacementSum,
                                          params);

                diffuseImageBlendingYu2011Equation4->SetColor(pixelPositionX,h-pixelPositionY,0,R_eq4);
                diffuseImageBlendingYu2011Equation3->SetColor(pixelPositionX,h-pixelPositionY,0,R_eq3);
                //======================== End Test encapsulated function =====================
                if (computeDisplacement)
                    displacementMap->SetColor(pixelPositionX,h-pixelPositionY,0,displacementSum);

                if (IsPointInTriangle(point,surfaceTexturePosition[0],surfaceTexturePosition[1],surfaceTexturePosition[2]))
                    this->pixelUsed[pixelPositionX][pixelPositionY] = true;
            }
        }
    }//------------------------ FIN RASTERISATION ---------------------
}


void HoudiniAtlas::SaveAtlas()
{
    //write the image to the disk
    diffuseImageBlendingGagnon->SaveImageAs(outputFilename);
    diffuseImageBlendingYu2011Equation3->SaveImageAs(outputFilename+".yu2011equationtree.png");
    diffuseImageBlendingYu2011Equation4->SaveImageAs(outputFilename+".yu2011equationfour.png");
    diffuseImageBlendingYu2011Equation4->growRegions(diffuseImageBlendingYu2011Equation4->image,diffuseImageBlendingYu2011Equation4->image,3); //
    diffuseImageBlendingYu2011Equation4->SaveImageAs(outputFilename+".yu2011equationfour.padded.png");
    cout << "Save texture atlas"<<outputFilename<<endl;
    if (computeDisplacement)
    {
        //write the image to the disk
        displacementMap->SaveImageAs(outputFilename+"displacement.png"); //HARDCODED NAME !!!
        displacementMap->growRegions(displacementMap->image,displacementMap->image,3); //
        displacementMap->SaveImageAs(outputFilename+".displacement.padded.png");
        cout << "Save texture atlas"<<outputFilename+"displacement.png"<<endl;
    }
}

void HoudiniAtlas::BoundingBox2D(UT_Vector3 a, UT_Vector3 b, UT_Vector3 c,UT_Vector3 &min,UT_Vector3 &max)
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

bool HoudiniAtlas::IsPointInTriangle(UT_Vector3  p, UT_Vector3 a,UT_Vector3 b,UT_Vector3 c)
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


Pixel HoudiniAtlas::SetRandomColor(int patchNumber)
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

void HoudiniAtlas::initPatchColors(GU_Detail *trackersGdp)
{

    GA_ROHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        int patchId = attId.get(ppt);
        patchColors[patchId] = SetRandomColor(patchId);
    }

}

