#include "BlendingGagnon2016.h"
#include "../HoudiniUtils.h"



/* how it should be called:
 *
                Pixel Cf = BlendingGagnon2016::Blend(trackersGdp,deformableGrids,i,j,w,h,
                                          pixelPositionX,pixelPositionY,
                                          sortedPatches,
                                          surfaceUv,
                                          surfacePosition,
                                          useDeformableGrids,
                                          rays,
                                          patchColors,
                                          alphaColor,
                                          RM,
                                          attAlpha,
                                          attPointUV,
                                          attLife,
                                          patchBlend,
                                          patchUvs,
                                          alphasMap,
                                          textureExemplar1Image,
                                          textureExemplar1ImageMask,
                                          displacementMapImage,
                                          computeDisplacement,
                                          renderColoredPatches,
                                          displacementSum);

 */

//================================= RASTERIZE PRIMITIVE =================================

Pixel BlendingGagnon2016::Blend(GU_Detail* trackersGdp,GU_Detail* deformableGrids, int i, int j, float w, float h,
                                int pixelPositionX, int pixelPositionY,
                                vector<int> &sortedPatches,
                                vector<UT_Vector3> &surfaceUv,
                                vector<UT_Vector3> &surfacePosition,
                                map<int,UT_Vector3> &trackersPosition,
                                bool useDeformableGrids,
                                map<string,GU_RayIntersect*> &rays,
                                map<int,Pixel> &patchColors,
                                Pixel alphaColor,
                                Pixel RM,           //Mean Value
                                GA_ROHandleF &attAlpha,
                                GA_RWHandleV3 &attPointUV,
                                int life,
                                map<int,float> &patchBlend,
                                map<int, vector<UT_Vector3> > &patchUvs,
                                map<int, vector<float> > &alphasMap,
                                ImageCV *textureExemplar1Image,
                                ImageCV *textureExemplar1ImageMask,
                                ImageCV *displacementMapImage,
                                bool computeDisplacement,
                                bool renderColoredPatches,
                                Pixel &displacementSum,
                                ParametersDeformablePatches params)
{





    bool debug = false;
    float thresholdDistance = 0.5;
    Pixel Cf = Pixel(0,0,0);
    Cf.A = 1;

    Pixel colorSum0 = Pixel(0,0,0);
    colorSum0.A = 1;

    Pixel colorSum1 = Pixel(0,0,0);
    colorSum1.A = 1;

    Pixel colorSum2 = Pixel(0,0,0);
    colorSum2.A = 1;

    float sumW2 = 0;
    float sumW = 0;

    Pixel color = Pixel(0,0,0);
    color.A = 1;

    Pixel displacement = Pixel(0,0,0);
    //Pixel displacementSum = Pixel(0,0,0);

    UT_Vector3 point;
    point.x() = i;
    point.y() = j;
    point.z() = 0;



    //test color
    color.R = 1;
    color.G = 1;
    color.B = 1;

    UT_Vector3 pixelPositionOnSurface;
    pixelPositionOnSurface.x() = ((float)pixelPositionX/(w-1));
    pixelPositionOnSurface.y() = ((float)pixelPositionY/(h-1));
    pixelPositionOnSurface.z() = 0;

    UT_Vector3 positionOnSurface = HoudiniUtils::GetBarycentricPosition(surfaceUv[0],surfaceUv[1],surfaceUv[2],surfacePosition[0],surfacePosition[1],surfacePosition[2],pixelPositionOnSurface);

    if (debug)
    {
        cout << "============================"<<endl;
        cout << "There are "<<sortedPatches.size() << " elements in sortedPatches"<<endl;
        //debug = false;
    }

    // compute color according to the list of patch
    vector<int>::iterator itPatch;
    int k = 0;
    //for(itPatch = sortedPatches.begin(); itPatch != sortedPatches.end(); itPatch++)
    for(itPatch = --sortedPatches.end(); itPatch != --sortedPatches.begin(); itPatch--)
    {
        //int patchId = patches[k];
        int patchId = *itPatch;

        //if (debugPatch && patchNumber != patchId)
        //    continue;

        //-------------------------------------------------------------
        //get deformable grids according to patchId in deformableGrids

        string str = std::to_string(patchId);
        string groupName;
        if (useDeformableGrids)
            groupName = "grid"+str;
        else
            groupName = "patch"+str;

        GU_MinInfo mininfo;
        mininfo.init(thresholdDistance,0.0001);
        map<string,GU_RayIntersect*>::const_iterator it = rays.find(groupName);
        if (it==rays.end())
        {
            continue;
        }


        rays[groupName]->minimumPoint(positionOnSurface,mininfo);

        if (!mininfo.prim)
        {
            //cout << "No primitive to project on"<<endl;
            continue;
        }

        const GEO_Primitive *prim = mininfo.prim;
        //get pos of hit
        UT_Vector4 hitPos;
        mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);

        if (prim->getVertexCount() < 3)
        {
            cout << "prim vertex count "<<prim->getVertexCount()<<endl;
            continue;
        }
        //------------------------------PARAMETRIC COORDINATE -----------------------------------
        float u = mininfo.u1;
        float v = mininfo.v1;

        GA_Offset vertexOffset0 = prim->getVertexOffset(0);
        GA_Offset localOffset0;
        GA_Offset pointOffset0;

        GA_Offset vertexOffset1 = prim->getVertexOffset(1);
        GA_Offset localOffset1;
        GA_Offset pointOffset1;

        GA_Offset vertexOffset2 = prim->getVertexOffset(2);
        GA_Offset localOffset2;
        GA_Offset pointOffset2;


        UT_Vector3 uvPatch;
        float   alphaPatch;
        UT_Vector3 positionInPolygon;

        if (useDeformableGrids)
        {
            pointOffset0  = deformableGrids->vertexPoint(vertexOffset0);
            pointOffset1  = deformableGrids->vertexPoint(vertexOffset1);
            pointOffset2  = deformableGrids->vertexPoint(vertexOffset2);

            float a0 = attAlpha.get(pointOffset0);
            float a1 = attAlpha.get(pointOffset1);
            float a2 = attAlpha.get(pointOffset2);

            alphaPatch = a0+u*(a1-a0)+v*(a2-a0);
            if (alphaPatch < 0)
            {
                alphaPatch = 0;
            }

            if (attPointUV.isInvalid())
                continue;

            UT_Vector3 v0 = attPointUV.get(pointOffset0);
            UT_Vector3 v1 = attPointUV.get(pointOffset1);
            UT_Vector3 v2 = attPointUV.get(pointOffset2);

            uvPatch = v0+u*(v1-v0)+v*(v2-v0);
            positionInPolygon = uvPatch;

        }
        else
        {
            UT_Vector3 uvPatch1 = patchUvs[patchId][0];
            UT_Vector3 uvPatch2 = patchUvs[patchId][1];
            UT_Vector3 uvPatch3 = patchUvs[patchId][2];
            positionInPolygon = HoudiniUtils::GetBarycentricPosition(surfaceUv[0],surfaceUv[1],surfaceUv[2],uvPatch1,uvPatch2,uvPatch3,pixelPositionOnSurface);
            float a0 = alphasMap[patchId][0];
            float a1 = alphasMap[patchId][1];
            float a2 = alphasMap[patchId][2];
            alphaPatch = a0+u*(a1-a0)+v*(a2-a0);
        }

        int w = textureExemplar1Image->GetWidth();
        int h = textureExemplar1Image->GetHeight();
        int wm = textureExemplar1ImageMask->GetWidth();
        int hm = textureExemplar1ImageMask->GetHeight();

        int i2 = static_cast<int>(floor(positionInPolygon.x()*w));
        int j2 = (h-1)-static_cast<int>(floor((positionInPolygon.y())*h));

        int i3 = static_cast<int>(floor(positionInPolygon.x()*wm));
        int j3 = (hm-1)-static_cast<int>(floor((positionInPolygon.y())*hm));

        //textureExemplar1Image->GetColor(pixelPositionX,pixelPositionY,0,color);
        if (renderColoredPatches)
        {
            //set random colors per patch
            color.R = 1;
            color.G = 1;
            color.B = 1;
            color = patchColors[patchId];
        }
        else
        {
            textureExemplar1Image->GetColor(i2,j2,0,color);
        }
        textureExemplar1ImageMask->GetColor(i3,j3,0,alphaColor);
        if (computeDisplacement)
            displacementMapImage->GetColor(i2,j2,0,displacement);

        float alpha = ((alphaColor.R+alphaColor.G+alphaColor.B)/3) * alphaPatch;

        //alpha = alpha * patchBlend[patchId];

        //alphaSum += alpha;
        if (renderColoredPatches)
        {
            //alpha = 1;
        }
        //HAACCKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK
        alpha = 1;
        color.A = alpha;

        //---------------- Transparency Equation -----------------------

        sumW2 += alpha*alpha;
        sumW += alpha;

        colorSum0.R += color.R;
        colorSum0.G += color.G;
        colorSum0.B += color.B;

        Cf.R =  (alpha)*(color.R) + (1.0f-alpha)*(Cf.R);
        Cf.G =  (alpha)*(color.G) + (1.0f-alpha)*(Cf.G);
        Cf.B =  (alpha)*(color.B) + (1.0f-alpha)*(Cf.B);

        if (debug)
            cout << "patch "<<patchId<<" alpha "<<alpha<<endl;

        if (computeDisplacement)
        {
            displacementSum.R = alpha*displacement.R + (1.0f-alpha)*displacementSum.R;
            displacementSum.G = alpha*displacement.G + (1.0f-alpha)*displacementSum.G;
            displacementSum.B = alpha*displacement.B + (1.0f-alpha)*displacementSum.B;
        }
        k++;
    }

    if (Cf.R > 1)
        Cf.R = 1;
    if (Cf.G > 1)
        Cf.G = 1;
    if (Cf.B > 1)
        Cf.B = 1;

    if (Cf.R < 0)
        Cf.R = 0;
    if (Cf.G < 0)
        Cf.G = 0;
    if (Cf.B < 0)
        Cf.B = 0;

    return Cf;
}

