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
                                bool useDeformableGrids,
                                map<int,UT_Vector3> &trackersUVPosition,
                                map<string,GU_RayIntersect*> &rays,
                                map<int,Pixel> &patchColors,
                                GA_ROHandleF &attAlpha,
                                GA_RWHandleV3 &attPointUV,
                                map<int,float> &patchBlend,
                                vector<ImageCV*> textureExemplars,
                                vector<ImageCV*> textureExemplarImageMaskVector,
                                ImageCV *displacementMapImage,
                                bool computeDisplacement,
                                bool renderColoredPatches,
                                Pixel &R1,
                                Pixel &displacementSum,
                                ParametersDeformablePatches params)
{


    //Equation 2, Quality of a triangle
    GA_RWHandleF    attQt(deformableGrids->findFloatTuple(GA_ATTRIB_PRIMITIVE,"Qt",1));
    GA_RWHandleI    attBorder(deformableGrids->findIntTuple(GA_ATTRIB_PRIMITIVE,"border",1));

    Pixel alphaColor;

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


        //-----------------------------------
        //Q_v quality of the vertex, value from 0 to 1
        float   Q_t = attQt.get(prim->getMapOffset());

        if (Q_t < 0.001)
            continue;
        float   Q_V = Q_t;

        UT_Vector3 centerUV = trackersUVPosition[patchId];//UT_Vector3(0.5,0.5,0.0);
        float d_P = distance3d(positionInPolygon,centerUV);
        //float maxDUV = 0.175f; //should comme from the scaling used for the uv projection.
        //float maxDUV = (0.5f*sqrt(1.0f/params.UVScaling))/2.0f;
        float s = params.UVScaling;
        float minDUV = 0.125*s;
        float maxDUV = 0.25*s; //blending region
        //float maxDUV = 0.5f;
        //d_V =0 if V ∈ grid boundary 1 otherwise
        //float d_V = 1.0f;
        int     d_V = 1-attBorder.get(prim->getMapOffset());
        //test
        //maxDUV = params.poissondiskradius/2;
        if (d_P > maxDUV)
            d_V = 0.0f;

        float C_s = 0.0f;
        if (d_P > minDUV && d_P <= maxDUV)
            C_s = 1-((d_P-minDUV)/(maxDUV-minDUV));
        if (d_P <= minDUV)
            C_s = 1.0f;

        //float K_s = (1.0f-(d_P/maxDUV))*d_V*Q_V;
        float K_s = C_s*d_V*Q_V;
        //cout << "Ks "<<K_s<<endl;
        //cout << "d_V "<<d_V<<endl;

        //K_s should be between 0 and 1
        if (K_s < 0)
            K_s = 0;
        else if (K_s > 1.0f)
            K_s = 1.0f;

        //--------------------------Equation 6--------------------
        //temporal componet

        //from the Rapport de Chercher: https://hal.inria.fr/inria-00355827v4/document
        //Also available in the doc directory of this project: doc/yu2009SPTA.pdf

        //Temporal weights Finally we fade particles in and out at their creation and destruction,
        //using two weights Fin(t) and Fout(t). The fading period τ can be long (in our
        //implementation we used τ = 5 seconds). Fout is mainly used to force the fading out
        //of grids whose distortion would stop increasing. And if τ is longer than the particles
        //lifetime, Fin and Fout rule the weights of all particles and are thus renormalized at the end

        //The temporal component is simply a linear fade-in at the beginning of the life of a particle and a linear fade-out
        //after the particle has been killed.
        //Here, we take the fading from the particle stored in a map where the indexes are the patch number.
        float K_t = patchBlend[patchId];
        if (K_t < 0)
            K_t = 0;
        else if (K_t > 1.0f)
            K_t = 1.0f;



        int w = textureExemplars[0]->GetWidth();
        int h = textureExemplars[0]->GetHeight();
        if (textureExemplarImageMaskVector.size() < 1)
            continue;

        int wm = textureExemplarImageMaskVector[0]->GetWidth();
        int hm = textureExemplarImageMaskVector[0]->GetHeight();

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
            textureExemplars[0]->GetColor(i2,j2,0,color);
        }
        float watershedIndices = (1.0f-K_t)*100.0f;
        int index = watershedIndices;
        textureExemplarImageMaskVector[index]->GetColor(i3,j3,0,alphaColor);
        //temporal hack
        /*
        alphaColor.R = 1- alphaColor.R;
        alphaColor.G = 1- alphaColor.G;
        alphaColor.B = 1- alphaColor.B;
        */
        if (computeDisplacement)
            displacementMapImage->GetColor(i2,j2,0,displacement);

        float alpha = ((alphaColor.R+alphaColor.G+alphaColor.B)/3) * K_s;
        Pixel alphaShed;



        alpha = alpha * patchBlend[patchId];

        //alphaSum += alpha;
        if (renderColoredPatches)
        {
            //alpha = 1;
        }

        color.A = alpha;

        //---------------- Transparency Equation -----------------------

        sumW2 += alpha*alpha;
        sumW += alpha;

        float alpha2 = alpha*alpha;

        colorSum0.R += color.R;
        colorSum0.G += color.G;
        colorSum0.B += color.B;

        //colorSum0.R /= sumW;
        //colorSum0.G /= sumW;
        //colorSum0.B /= sumW;

        /*
        Cf.R =  (alpha2)*(color.R-RM.R) + (1.0f-alpha2)*(Cf.R);
        Cf.G =  (alpha2)*(color.G-RM.G) + (1.0f-alpha2)*(Cf.G);
        Cf.B =  (alpha2)*(color.B-RM.B) + (1.0f-alpha2)*(Cf.B);
        */

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

    //------------------
    //colorSum =textureExamplarMeanValue;

    //---------------------
    //Do we really need this ?
    //Cf.R += RM.R;
    //Cf.G += RM.G;
    //Cf.B += RM.B;
    //---------------------

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
    //------------------

}

