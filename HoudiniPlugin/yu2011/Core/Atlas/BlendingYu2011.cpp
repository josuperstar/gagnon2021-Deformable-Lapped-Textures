#include "BlendingYu2011.h"
#include "../HoudiniUtils.h"

//================================= RASTERIZE PRIMITIVE =================================

Pixel BlendingYu2011::Blend(GU_Detail* trackersGdp,GU_Detail* deformableGrids, int i, int j, float w, float h,
                                int pixelPositionX, int pixelPositionY,
                                vector<int> &sortedPatches,
                                vector<UT_Vector3> &surfaceUv,
                                vector<UT_Vector3> &surfacePosition,
                                map<int,UT_Vector3> &trackersPosition,
                                map<string,GU_RayIntersect*> &rays,
                                map<int,Pixel> &patchColors,
                                Pixel RM,           //Mean Value
                                GA_RWHandleV3 &attPointUV,
                                map<int,float> &fading,
                                ImageCV *textureExemplar1Image,
                                ImageCV *displacementMapImage,
                                bool computeDisplacement,
                                bool renderColoredPatches,
                                Pixel &R_eq3,
                                Pixel &displacementSum,
                                ParametersDeformablePatches params)
{

    float d = params.poissondiskradius;

    Pixel displaceMean;

    if(computeDisplacement)
    {
        displaceMean = displacementMapImage->MeanValue();
    }

    int tw = textureExemplar1Image->GetWidth();
    int th = textureExemplar1Image->GetHeight();
    //int wm = textureExemplar1ImageMask->GetWidth();
    //int hm = textureExemplar1ImageMask->GetHeight();

    Pixel R_eq4 = Pixel(0,0,0);
    R_eq4.A = 1;
    R_eq3 = Pixel(0,0,0);
    R_eq3.A = 1;
    float sumW2 = 0;
    float sumW = 0;

    Pixel color = Pixel(0,0,0);
    color.A = 1;

    Pixel displacement = Pixel(0,0,0);

    //Equation 2, Quality of a triangle
    GA_RWHandleF    attQt(deformableGrids->findFloatTuple(GA_ATTRIB_PRIMITIVE,"Qt",1));
    UT_Vector3 pixelPositionOnSurface;

    //We don't work with an image with no width of height
    if (w-1.0f == 0.0f || h-1.0f == 0.0f)
        return color;

    pixelPositionOnSurface.x() = ((float)pixelPositionX/(w-1));
    pixelPositionOnSurface.y() = ((float)pixelPositionY/(h-1));
    pixelPositionOnSurface.z() = 0;

    UT_Vector3 positionOnSurface = HoudiniUtils::GetBarycentricPosition(surfaceUv[0],surfaceUv[1],surfaceUv[2],surfacePosition[0],surfacePosition[1],surfacePosition[2],pixelPositionOnSurface);

    float thresholdProjectionDistance = d/2.0f;

    //================================= SUMARIZE ========================================
    // compute color according to the list of patch
    //for each pixel patch loop
    vector<int>::iterator itPatch;
    for(itPatch = --sortedPatches.end(); itPatch != --sortedPatches.begin(); itPatch--)
    {
        int patchId = *itPatch;
        //-------------------------------------------------------------
        //get deformable grids according to patchId in deformableGrids

        string str = std::to_string(patchId);
        string groupName;
        groupName = "grid"+str;

        //--------------------------------------------------
        //Can we project the pixel on the current patch ?
        GU_MinInfo mininfo;
        mininfo.init(thresholdProjectionDistance,0.0001f);
        map<string,GU_RayIntersect*>::const_iterator it = rays.find(groupName);
        if (it==rays.end())
            continue;
        rays[groupName]->minimumPoint(positionOnSurface,mininfo);
        if (!mininfo.prim)
            continue;
        //--------------------------------------------------

        const GEO_Primitive *prim = mininfo.prim;
        //We don't work with primitive that don't have at least 3 vertices
        if (prim->getVertexCount() < 3)
            continue;

        //------------------------------PARAMETRIC COORDINATE -----------------------------------
        float u = mininfo.u1;
        float v = mininfo.v1;
        GA_Offset vertexOffset0 = prim->getVertexOffset(0);
        GA_Offset pointOffset0;
        GA_Offset vertexOffset1 = prim->getVertexOffset(1);
        GA_Offset pointOffset1;
        GA_Offset vertexOffset2 = prim->getVertexOffset(2);
        GA_Offset pointOffset2;

        //------------------- secion 3.3.3 Estimating Grid Distortion --------------

        //------- position in polygon of the deformable grid --------
        if (attPointUV.isInvalid())
            continue;
        pointOffset0  = deformableGrids->vertexPoint(vertexOffset0);
        pointOffset1  = deformableGrids->vertexPoint(vertexOffset1);
        pointOffset2  = deformableGrids->vertexPoint(vertexOffset2);
        UT_Vector3 v0 = attPointUV.get(pointOffset0);
        UT_Vector3 v1 = attPointUV.get(pointOffset1);
        UT_Vector3 v2 = attPointUV.get(pointOffset2);

        UT_Vector3 positionInPolygon = v0+u*(v1-v0)+v*(v2-v0);

        //-----------------------------------
        //Q_v quality of the vertex, value from 0 to 1
        float   Q_V = attQt.get(prim->getMapOffset());

        //we don't want to have a black spot so we put a minimum value
        //we should add a new Poisson disk when we have a black pixel
        //if (Qv <=0 )
        //    Qv = 0.001;
        //-----------------------------------------------------------------
        //getting the color from the texture exemplar


        int i2 = static_cast<int>(floor(positionInPolygon.x()*tw));
        int j2 = ((int)th-1)-static_cast<int>(floor((positionInPolygon.y())*th));

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

        if (computeDisplacement)
            displacementMapImage->GetColor(i2,j2,0,displacement);

        //-----------------------------------------------------------------

        //--------------------------Equation 7--------------------
        //spatial component

        /*
        The spatial component merges three factors: the quality around each grid vertex (QV , defined in section 3.3.3),
        a fall-off with the distance to the particle (in our implementation we take it linear), and a continuity factor
        ensuring a weight 0 on the boundary of the grid (to avoid spatial discontinuities during blending):

        K_s(V) = (1- (||v||-p)/d)*d_V*Q_V
        where d_V =0 if V ∈ grid boundary
        1 otherwise

        The weights are computed for each vertex. During reconstruction, weights at arbitrary locations are interpolated
        from vertices values.
        */

        //Here, we use a alpha chanel with linear fading from the center to compute the fall-off
        //Qv is an interpolation of the alpha chanel of the polygon use in the grid, for this patch.
        //Therefore, the Quality of the vertex has been computed before and stored in the alpha chanel

        //float dP = ((falloff.R+falloff.G+falloff.B)/3);

        //tracker position
        UT_Vector3 trackerPosition = trackersPosition[patchId];//trackersGdp->getPos3(patchId);

        //UT_Vector3 diffP = positionOnSurface-trackerPosition;
        float d_P = distance3d(positionOnSurface,trackerPosition);//diffP.length();
        float d_V = 1.0f;

        if (d_P > d)
            d_V = 0.0f;
        float K_s = (1.0f-(d_P/d))*d_V*Q_V;

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
        //after the particle has been killed:
        float K_t = fading[patchId];

        //--------------------------Equation 5--------------------
        // section 3.4.1 Vertex Weights
        //The weight for each vertex is defined as the product of a spatial component and a temporal component
        float w_v = K_s * K_t;

        //--------------------------------------------------------
        //Section 3.5.1
        //• Allocate the intermediate texture R at the required resolution (see discussion in 3.5.3), with one channel
        //for each aj (RGB), plus one channel for the wi (resp. wi2 if using Eq. 4).

        //• For each particle i: splat its grid into the texture (e.g., using a render target and the ordinary drawing API),
        //thus accumulating the sum(wi(x)aj (ui(x)) and the sum(wi(x)) (resp. sum(w2i) ) into their respective channels.

        sumW2 += w_v*w_v;
        sumW += w_v;

        //---------------- YU 2011 Equation 3 -----------------------
        //blending function
        R_eq3.R += w_v*(color.R);
        R_eq3.G += w_v*(color.G);
        R_eq3.B += w_v*(color.B);

        //---------------- YU 2011 Equation 4 -----------------------
        //blending function
        R_eq4.R += w_v*(color.R - RM.R);
        R_eq4.G += w_v*(color.G - RM.G);
        R_eq4.B += w_v*(color.B - RM.B);

        if (computeDisplacement)
        {

            displacementSum.R += w_v*(displacement.R - displaceMean.R);
            displacementSum.G += w_v*(displacement.G - displaceMean.G);
            displacementSum.B += w_v*(displacement.B - displaceMean.B);
        }
    }
    //========================= END SUM ==================================

    float epsilon = 0.0001f;
    //if the sumW is close to 0, that means we should have a new poisson disk there.
    //But how to handle the creation of a new patch from here ?

    if (sumW <= epsilon && sumW >= -epsilon )
    {
        R_eq3.R = 0;
        R_eq3.G = 0;
        R_eq3.B = 0;
        R_eq3.A = 0;
        //----------
        R_eq4.R = 0;
        R_eq4.G = 0;
        R_eq4.B = 0;
        R_eq4.A = 0;
        return R_eq4;
    }

    //Section 3.5.1
    //During rendering in the fragment shader, for a given pixel (having texture coordinates x):
    //• For each channel aj : finalize the channel value computation by dividing the accumulated values by
    //sum(wi)
    //(resp. by sqrt(sum(wi2)) if using Eq. 4).
    //• Compute the texture value for the current pixel,
    //F (R(x)).
    //• Use the texture value as we would with a standard texture.

    //---------------- YU 2011 Equation 3 -----------------------
    //blending function, division part
    R_eq3.R = R_eq3.R/sumW;
    R_eq3.G = R_eq3.G/sumW;
    R_eq3.B = R_eq3.B/sumW;

    //---------------- YU 2011 Equation 4 -----------------------
    //blending function, division part
    float sqw = sqrtf(sumW2);

    R_eq4.R = (R_eq4.R)/sqw    + RM.R;
    R_eq4.G = (R_eq4.G)/sqw    + RM.G;
    R_eq4.B = (R_eq4.B)/sqw    + RM.B;

    if (computeDisplacement)
    {
        displacementSum.R = (displacementSum.R)/sqw    + displaceMean.R;
        displacementSum.G = (displacementSum.G)/sqw    + displaceMean.G;
        displacementSum.B = (displacementSum.B)/sqw    + displaceMean.B;

        Clamp(displacementSum);

    }

    //affect by the alpha ???
    float Af = sumW;
    if (Af > 1)
        Af = 1;

    //If we don't use the alpha, we are having black spot where sumW is 0
    //R2.A = Af;
    //R1.A = Af;
    //we need to invertigate why we need to check that:
    Clamp(R_eq4);

    return R_eq4;
}



