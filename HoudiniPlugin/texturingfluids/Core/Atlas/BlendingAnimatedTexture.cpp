#include "BlendingAnimatedTexture.h"
#include "../HoudiniUtils.h"

//================================= RASTERIZE PRIMITIVE =================================

Pixel BlendingAnimatedTexture::Blend(GU_Detail* deformableGrids, int i, int j, float w, float h,
                                int pixelPositionX, int pixelPositionY,
                                vector<int> &sortedPatches,
                                vector<UT_Vector3> &surfaceUv,
                                vector<UT_Vector3> &surfacePosition,
                                map<int,UT_Vector3> &trackersNormal,
                                map<int,UT_Vector3> &trackersPosition,
                                map<int,UT_Vector3> &trackersUVPosition,
                                map<int, bool> &usePatches,
                                map<string,GU_RayIntersect*> &rays,
                                map<int,Pixel> &patchColors,
                                Pixel RM,           //Mean Value
                                GA_RWHandleV3 &attPointUV,
                                map<int,float> &fading,
                                vector<ImageCV*> textureExemplars,
                                ImageCV *displacementMapImage,
                                bool computeDisplacement,
                                bool renderColoredPatches,
                                Pixel &R_eq3,
                                Pixel &displacementSumEq3,
                                Pixel &displacementSumEq4,
                                ParametersDeformablePatches params)
{


    bool debug = false;
    float d = params.poissondiskradius;

    int tw = textureExemplars[0]->GetWidth();
    int th = textureExemplars[0]->GetHeight();

    float epsilon = 0.0001f;

    vector<Pixel> colorsList;

    Pixel color = Pixel(0,0,0);
    color.A = 1;

    Pixel patchColor = Pixel(0,0,0);

    Pixel Cf = Pixel(0,0,0);
    Cf.A = 0;

    Pixel displacement = Pixel(0,0,0);

    GA_RWHandleF    attQv(deformableGrids->findFloatTuple(GA_ATTRIB_POINT,"Qv",1));
    //cout << "get border attribute"<<endl;
    GA_RWHandleI    attBorder(deformableGrids->findIntTuple(GA_ATTRIB_POINT,"border",1));
    if (attBorder.isInvalid())
        return Cf;
    UT_Vector3 pixelPositionOnSurface;

    //We don't work with an image with no width of height
    if (w-1.0f == 0.0f || h-1.0f == 0.0f)
        return color;

    pixelPositionOnSurface.x() = ((float)pixelPositionX/(w-1));
    pixelPositionOnSurface.y() = ((float)pixelPositionY/(h-1));
    pixelPositionOnSurface.z() = 0;

    UT_Vector3 positionOnSurface = HoudiniUtils::GetBarycentricPosition(surfaceUv[0],surfaceUv[1],surfaceUv[2],surfacePosition[0],surfacePosition[1],surfacePosition[2],pixelPositionOnSurface);

    float thresholdProjectionDistance = d/2.0f;
    int k = sortedPatches.size();

    vector<int>::iterator itPatch;
    //cout << "For each patch"<<endl;
    for(itPatch = --sortedPatches.end(); itPatch != --sortedPatches.begin(); itPatch--)
    {
        int patchId = *itPatch;

        if ( fading.find(patchId) == fading.end())
            continue;

        //-------------------------------------------------------------
        //get deformable grids according to patchId in deformableGrids

        string str = std::to_string(patchId);
        string groupName;
        groupName = "grid"+str;

        GU_RayIntersect *ray;

        map<string,GU_RayIntersect*>::const_iterator it = rays.find(groupName);
        if (it==rays.end())
            continue;
        ray = rays[groupName];

        //--------------------------------------------------
        //Can we project the pixel on the current patch ?
        //We may want to put this test outside this function, especially if we want to create a shader.
        GU_MinInfo mininfo;
        mininfo.init(thresholdProjectionDistance,0.0001f);

        ray->minimumPoint(positionOnSurface,mininfo);
        if (!mininfo.prim)
            continue;
        //--------------------------------------------------

        const GEO_Primitive *prim = mininfo.prim;

        UT_Vector3 primN = prim->computeNormal();
//        if (dot(primN, trackersNormal[patchId]) < 0.7)
//            continue;
        if(debug)
            cout << "dealing with prim "<<prim->getMapOffset()<<endl;
        //We don't work with primitive that don't have at least 3 vertices
        if (prim->getVertexCount() < 3)
            continue;

        //------------------------------ PARAMETRIC COORDINATE -----------------------------------
        float u = mininfo.u1;
        float v = mininfo.v1;

        //get pos of hit
        UT_Vector4 hitPos;
        mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);

        //------------------------------ Test Hit Distance --------------------------
        float dist = distance3d(positionOnSurface,hitPos);
        if (dist > thresholdProjectionDistance)
            continue;

        if (dist > d/10.0f)
        {
            UT_Vector3 AB = (positionOnSurface - hitPos);
            AB = AB.normalize();
            // This can flicker
            float angle = dot(AB,primN);
            if (abs(angle) < 0.2)
                continue;
        }

        // We want to check that the pixel is not outside of the uv patch radius transferred on the surface.
        UT_Vector3 tracker = trackersPosition[patchId];
        float gridwidth = (2+params.Yu2011Beta)*params.poissondiskradius;
        float d_P = distance3d(hitPos,tracker);
        if (d_P > gridwidth)
            continue;

        GA_Offset vertexOffset0 = prim->getVertexOffset(0);
        GA_Offset vertexOffset1 = prim->getVertexOffset(1);
        GA_Offset vertexOffset2 = prim->getVertexOffset(2);

        if (attPointUV.isInvalid())
            continue;

        GA_Offset pointOffset0  = deformableGrids->vertexPoint(vertexOffset0);
        GA_Offset pointOffset1  = deformableGrids->vertexPoint(vertexOffset1);
        GA_Offset pointOffset2  = deformableGrids->vertexPoint(vertexOffset2);

        UT_Vector3 v0 = attPointUV.get(pointOffset0);
        UT_Vector3 v1 = attPointUV.get(pointOffset1);
        UT_Vector3 v2 = attPointUV.get(pointOffset2);

        //UT_Vector3 centerUV = trackersUVPosition[patchId];//UT_Vector3(0.5,0.5,0.0);
        //UT_Vector3 centerUV(0,0,0);
        if(debug)
            cout << "Computing uv coordinates "<<endl;
        UT_Vector3 centerUV(0.5,0.5,0.0);
        float s = params.UVScaling;

        v0 = UT_Vector3(v0.x()-centerUV.x(),v0.y()-centerUV.y(),v0.z()-centerUV.z());
        v1 = UT_Vector3(v1.x()-centerUV.x(),v1.y()-centerUV.y(),v1.z()-centerUV.z());
        v2 = UT_Vector3(v2.x()-centerUV.x(),v2.y()-centerUV.y(),v2.z()-centerUV.z());

        v0 *= s;
        v1 *= s;
        v2 *= s;

        v0 = UT_Vector3(v0.x()+centerUV.x(),v0.y()+centerUV.y(),v0.z()+centerUV.z());
        v1 = UT_Vector3(v1.x()+centerUV.x(),v1.y()+centerUV.y(),v1.z()+centerUV.z());
        v2 = UT_Vector3(v2.x()+centerUV.x(),v2.y()+centerUV.y(),v2.z()+centerUV.z());

        UT_Vector3 positionInPolygon = v0+u*(v1-v0)+v*(v2-v0);
        float B_V = 1.0f;
        UT_Vector3 centerUV2(0.5,0.5,0.0);
        float uv_distance = distance3d(positionInPolygon,centerUV2);
        float minD = 0.25;
        float maxD = 0.5; //edge region
        if (uv_distance > maxD)
        {
            B_V = 0.0f;

            //cout << "uv distance > maxD"<<endl;
        }
        float D_v = 0.0f;
        if (uv_distance > minD && uv_distance <= maxD)
            D_v = 1-((uv_distance-minD)/(maxD-minD));
        if (uv_distance <= minD)
            D_v = 1.0f;

        //cout << "UV Distance = "<<uv_distance<<endl;
        //-----------------------------------
        //Q_v quality of the vertex, value from 0 to 1
        //The weights are computed for each vertex. During reconstruction, weights at arbitrary locations are interpolated
        //from vertices values.

        // If the patch has been created on a curved region, it is possible to have the center of the patch closed to a border.
        // We want to avoid treating the polygon closed to the center as border has it can create holes on the surface.
        //float d_V = 1.0f;
        if (uv_distance <= maxD && d_P > gridwidth/10)
        //if (uv_distance <= maxD)
        {
            float bv1 = attBorder.get(pointOffset0);
            float bv2 = attBorder.get(pointOffset1);
            float bv3 = attBorder.get(pointOffset2);

            float   d_V1 = 1.0f - bv1;
            float   d_V2 = 1.0f - bv2;
            float   d_V3 = 1.0f - bv3;

            B_V = d_V1+u*(d_V2-d_V1)+v*(d_V3-d_V1);
//            if (B_V != 1)
//                cout << B_V<<endl;
        }

        float Q_t1 = attQv.get(pointOffset0);
        float Q_t2 = attQv.get(pointOffset1);
        float Q_t3 = attQv.get(pointOffset2);

        float Q_t = Q_t1+u*(Q_t2-Q_t1)+v*(Q_t3-Q_t1);

        if (Q_t < 0.001)
            continue;


        //-----------------------------------------------------------------
        //getting the color from the texture exemplar
        int i2 = static_cast<int>(floor(positionInPolygon.x()*tw));
        int j2 = ((int)th-1)-static_cast<int>(floor((positionInPolygon.y())*th));


        //float K_s = C_s*d_V*Q_V;
        //float w_pt = Q_V_B_v;
        //float w_pt = Q_t*B_V*D_v;
        //float w_pt = Q_t*B_V;
        //float w_pt = Q_t*D_v;
        float w_pt = Q_t*B_V;
        //w_pt should be between 0 and 1
        if (w_pt < 0)
            w_pt = 0;
        else if (w_pt > 1.0f)
            w_pt = 1.0f;

        if (w_pt < epsilon)
        {
            if(debug)
                cout << "w_pt too small: "<<w_pt<<endl;
            continue;
        }

        int seamCarvingIndex = ((1-w_pt) * params.NumberOfTextureSampleFrame);
        //int seamCarvingIndex = 0;

        if (renderColoredPatches)
        {
            //set random colors per patch
            patchColor = patchColors[patchId];
            textureExemplars[seamCarvingIndex]->GetColor(i2,j2,0,color);
            float tempAlpha = color.A;
            color = patchColor;
            color.A = tempAlpha;
        }
        else
        {
            if(debug)
                cout << "getting texel at index "<<seamCarvingIndex<<endl;
            textureExemplars[seamCarvingIndex]->GetColor(i2,j2,0,color);
        }
            //cout << "Animated Color "<<color.R<<" "<<color.G<<" "<<color.B<<endl;

        if(computeDisplacement)
            displacementMapImage->GetColor(i2,j2,0,displacement);

//        float F = displacement.R;
//        float D = (D_v);
//        float E = F*D + D*D;
//        float threshold = 1-w_pt;

        // Flag that we use this patch during the synthesis.
        // We could therefore delete unused patches in the future.

        float alpha = 0;
        if (color.A > 0.0f)
        //if (E > threshold)
        {
            alpha = 1;
            usePatches[patchId] = true;
        }

        //clamping color values ...
        if (color.B > 1.0f)
            color.B = 1.0f;
        if (color.G > 1.0f)
            color.G = 1.0f;
        if (color.R > 1.0f)
            color.R = 1.0f;
        if (computeDisplacement)
        {
            if (displacement.B > 1.0f)
                displacement.B = 1.0f;
            if (displacement.G > 1.0f)
                displacement.G = 1.0f;
            if (displacement.R > 1.0f)
                displacement.R = 1.0f;
        }

        // We use the alpha from the animated images to influence the weight

        //float alpha = color.A;// * w_v;
        Cf.R =  (alpha)*(color.R) + (1.0f-alpha)*(Cf.R);
        Cf.G =  (alpha)*(color.G) + (1.0f-alpha)*(Cf.G);
        Cf.B =  (alpha)*(color.B) + (1.0f-alpha)*(Cf.B);
        Cf.A += color.A;
        colorsList.push_back(color);
        if (computeDisplacement)
        {
            displacementSumEq4.R =  (alpha)*(displacement.R) + (1.0f-alpha)*(displacementSumEq4.R);
            displacementSumEq4.G =  (alpha)*(displacement.G) + (1.0f-alpha)*(displacementSumEq4.G);
            displacementSumEq4.B =  (alpha)*(displacement.B) + (1.0f-alpha)*(displacementSumEq4.B);
        }
        k--;
        if(debug)
            cout << "color computed "<<endl;
    }

    //cout << "done"<<endl;
    //========================= END SUM ==================================

    return Cf;
}




