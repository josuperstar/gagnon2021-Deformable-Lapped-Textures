#include "DistortionManager.h"

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

#include <GU/GU_Flatten.h>



//================================================================================================

//                                       COMPUTE DISTORTION

//================================================================================================

void DistortionComputation::ComputeDistortionBasedOnEdges(GU_Detail *gdp)
{

    //We should use YU 2010 Lagrangian Texture Advection approach, section 3.3.3 Estimating the Grid Distortion

    /*
    For a single triangle in the grid, we compute distortion from
     the initial state with the singular values gMin and gMax of the
    Jacobian of the transform between the original triangle and
    the advected triangle [22], [23]. We define the distortion of a
    single triangle as

    dT = max(gMax,1/gmin)

    We then define the quality of a triangle as the ratio of its
     distortion with the maximum acceptable distortion, dMax

     Qt = max((dMax - dT)/(dMax - 1), 0)

     Qt is equal to 1 for an undistorted triangle, and is equal
    to 0 for a triangle where the distortion is larger than dMax .
    For each grid vertex V , we then compute its quality, Q_V as
    the mean of the quality of its incident triangles. We kill a
     particle if, for any vertex in the grid, we have Q_V < 1/2 (i.e.,
    we keep a margin of quality for the fading-out).


    //---------------------------
    Since the vertices are advected and the primitives split and merge, we cannot have the right tracking for "triangles"
    I think we should compute distortion per point and advect the dMax and dMin.

    */

    cout << "Compute distortion"<<endl;

    //------------------------------ GETTING PATCH IDS ARRAY ---------------------------
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
    //----------------------------------------------------------------------------------

    GEO_PointTreeGAOffset tree;
    tree.build(gdp, NULL);

    GU_NeighbourList neighborList;
    GU_NeighbourListParms neighborListParams;
    //neighborListParams.
    neighborList.build(gdp,neighborListParams);


    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(gdp,ppt)
    {

        //There are a lot of NAN values in the uv attribute. We need to clean that.
        //For this point, take the patch id list and for each one check if there is a nan value
        UT_FloatArray         data;
        aif->get(patchIds, ppt, data);
        int nb = data.size();


        set<GA_Offset> ptlist = HoudiniUtils::GetNeighbors(gdp,ppt);

        for (int i = 0; i< nb; i++)
        {
            UT_Vector3 uv, uvN;
            UT_Vector3 p, pN;
            int patchId = data.array()[i];
            string uvname = "uv"+std::to_string(patchId);

            string ratioMin0Name = "ratioMin0"+std::to_string(patchId);
            string ratioMax0Name = "ratioMax0"+std::to_string(patchId);

            string ratioMintName = "ratioMint"+std::to_string(patchId);
            string ratioMaxtName = "ratioMaxt"+std::to_string(patchId);

            GA_RWHandleV3 attUVcopy(gdp->findFloatTuple(GA_ATTRIB_POINT,uvname.c_str(), 3));

            GA_RWHandleF attratioMin0(gdp->addFloatTuple(GA_ATTRIB_POINT,ratioMin0Name.c_str(), 1));
            GA_RWHandleF attratioMax0(gdp->addFloatTuple(GA_ATTRIB_POINT,ratioMax0Name.c_str(), 1));

            GA_RWHandleF attratioMint(gdp->addFloatTuple(GA_ATTRIB_POINT,ratioMintName.c_str(), 1));
            GA_RWHandleF attratioMaxt(gdp->addFloatTuple(GA_ATTRIB_POINT,ratioMaxtName.c_str(), 1));


            float rationMin0 = attratioMin0.get(ppt);
            float rationMax0 = attratioMax0.get(ppt);
            float rationMint = 9999999.0f;
            float rationMaxt = 0.0f;
            bool firstTime = false;

            if (rationMin0 <= 0.00001)
            {
                rationMin0 = 9999999;
                firstTime = true;
            }
            if (rationMax0 <= 0.000001)
            {
                rationMax0 = 0.0f;
                firstTime = true;
            }
            if (!firstTime)
            {
                rationMaxt = rationMax0;
                rationMint = rationMin0;
            }

            if(attUVcopy.isValid())
            {

                uv = attUVcopy.get(ppt);

                p = gdp->getPos3(ppt);


                //compute uv distortion according to neighborhood
                //....
                float alphaValue = 1.0f;
                int neighborProcessed = 0;

                //----------------------------Writing Alpha Value--------------------------------------
                string alphaname = "Alpha"+std::to_string(patchId);
                GA_RWHandleF attAlpha(gdp->findFloatTuple(GA_ATTRIB_POINT,alphaname.c_str(), 1));
                if (attAlpha.isValid())
                {
                    //attAlpha = gdp->addFloatTuple(GA_ATTRIB_POINT,alphaname.c_str(), alphaValue);
                    //attAlpha.set(ppt,alphaValue);
                    int size = ptlist.size();
                    GA_Offset noffset;


                    for (std::set<GA_Offset>::iterator it=ptlist.begin(); it!=ptlist.end(); ++it)
                    {
                        //GEO_Point *p = ptlist[j];
                        noffset = *it;

                        //check if the patch id of this neighborhood vertex exists
                        UT_FloatArray         dataN;
                        aif->get(patchIds, noffset, dataN);
                        if (!dataN.find(noffset))
                        {
                            //there is no such patch id in this vertex matrix
                            continue;
                        }

                        pN = gdp->getPos3(noffset);
                        uvN = attUVcopy.get(noffset);

                        //compute ration between current point p and neighbor point pN
                        float dP = distance3d(p,pN);
                        float dU = distance3d(uv,uvN);

                        if (dU < 0.001)
                            continue;

                        float ratio = dP/dU;
                        if (ratio > rationMax0)
                            rationMaxt = ratio;
                        if (ratio <= rationMin0)
                            rationMint = ratio;

                        neighborProcessed++;

                    }
                }

                if(neighborProcessed == 0) //the vertex of this patch is alone, should be removed.
                {
                    attAlpha.set(ppt,0);
                    continue;
                }

                if (firstTime)
                {
                    rationMax0 = rationMaxt;
                    rationMin0 = rationMint;
                    attratioMax0.set(ppt,rationMax0);
                    attratioMin0.set(ppt,rationMin0);
                }

                attratioMint.set(ppt,rationMint);
                attratioMaxt.set(ppt,rationMaxt);


                //==================== COMPUTE ALPHA =====================
                //comptue alpha according to the ratio value
                float offset = 1.5;

                alphaValue = 0;
                if (rationMint <= rationMin0)
                    alphaValue = (1-(rationMin0-rationMint)*2);
                else if(rationMaxt > rationMax0)
                    alphaValue = (1-(rationMaxt-rationMax0)*2);
                else
                    alphaValue = 1;

                if (alphaValue >1)
                    alphaValue = 1;
                else if (alphaValue < 0)
                    alphaValue = 0;

                attAlpha.set(ppt,alphaValue);

            }

            //-------------------------------------------------------------------------------------

        }
    }


}



//================================================================================================

//                                       COMPUTE DISTORTION

//================================================================================================

void DistortionComputation::ComputeDistortionBasedOnAreas(GU_Detail *gdp)
{



    cout << "Compute distortion based on Areas"<<endl;

    //------------------------------ GETTING PATCH IDS ARRAY ---------------------------
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
    //----------------------------------------------------------------------------------

    GEO_PointTreeGAOffset tree;
    tree.build(gdp, NULL);

    GU_NeighbourList neighborList;
    GU_NeighbourListParms neighborListParams;
    //neighborListParams.
    neighborList.build(gdp,neighborListParams);


    GA_RWHandleF attArea3D(gdp->findFloatTuple(GA_ATTRIB_DETAIL,"area3D", 1));
    GA_RWHandleF attAreaUV(gdp->findFloatTuple(GA_ATTRIB_DETAIL,"areaUV", 1));

    GA_RWHandleV3 attUVcopy(gdp->findFloatTuple(GA_ATTRIB_POINT,"uv", 3));

    bool firstFrame = false;

    if (attArea3D.isInvalid() || attAreaUV.isInvalid())
    {
        cout << "There is no area3D or areaUV computed on detail attribute. This is probably the first frame."<<endl;
        firstFrame = true;
    }




    float sumArea3D = 0.0f;
    float sumAreaUV = 0.0f;

    GA_Offset ppt;
    GA_Primitive *prim;
    GA_Offset vertexOffset;
    GA_FOR_ALL_PRIMITIVES(gdp,prim)
    {

        float area3D = prim->calcArea();
        sumArea3D += area3D;

        int nbrVertex = (prim)->getVertexCount();
        set<int> patchList;
        set<int> sortedPatchList;
        for(int j= 0; j< nbrVertex; j++)
        {
            vertexOffset = (prim)->getVertexOffset(j);
            ppt = gdp->vertexPoint(vertexOffset);
            //------------ PATCH ARRAY ---------------
            UT_FloatArray         data;
            aif->get(patchIds, ppt, data);
            int nb = data.size();
            for(int i = 0; i < nb; i++)
            {
                patchList.insert(data.array()[i]);
            }
            //----------------------------------------
        }

        //1 - check if all the vertices are in the same patch

        set<int>::iterator its;
        //for (int i=0; i < sizePatchList; i++)
        for(its = patchList.begin(); its != patchList.end(); ++its)
        {
            int currentPatchOffset = *its;
            int nbOfVertexUsed = 0;
            for(int j= 0; j< nbrVertex; j++)
            {
                vertexOffset = (prim)->getVertexOffset(j);
                ppt = gdp->vertexPoint(vertexOffset);

                UT_FloatArray         data;
                aif->get(patchIds, ppt, data);
                if (data.find(currentPatchOffset))
                {
                    nbOfVertexUsed++;
                }
            }
            if (nbOfVertexUsed == nbrVertex)
            {
                sortedPatchList.insert(currentPatchOffset);
            }

        }

        for(its = sortedPatchList.begin(); its != sortedPatchList.end(); ++its)
        {
            //for this patch, compute the area
            //in 3D space
            //float area3D = TriangleAir(a,b,c);

            int patchId = *its;
            UT_Vector3 uv1, uv2, uv3;

            string uvname = "uv"+std::to_string(patchId);
            GA_RWHandleV3 attUVcopy(gdp->findFloatTuple(GA_ATTRIB_POINT,uvname.c_str(), 3));

            string alphaname = "Alpha"+std::to_string(patchId);
            GA_RWHandleF attAlpha(gdp->findFloatTuple(GA_ATTRIB_POINT,alphaname.c_str(), 1));


            GA_Offset vertexOffset0 = (prim)->getVertexOffset(0);
            GA_Offset vertexOffset1 = (prim)->getVertexOffset(1);
            GA_Offset vertexOffset2 = (prim)->getVertexOffset(2);

            ppt = gdp->vertexPoint(vertexOffset0);
            uv1 = attUVcopy.get(ppt);
            ppt = gdp->vertexPoint(vertexOffset1);
            uv2 = attUVcopy.get(ppt);
            ppt = gdp->vertexPoint(vertexOffset2);
            uv3 = attUVcopy.get(ppt);

            float areaUV = TriangleAir(uv1,uv2,uv3);

            //==================== COMPUTE ALPHA =====================
            //comptue alpha according to the ratio value
            //float offset = 1.5;
            // from vex code:
            /*
            int success = 0;
            @initRatio = detailattrib(0,"area",0,success)/detailattrib(0,"areaUV",0,success);;
            @ratio = @area/@areaUV;
            @Alpha = 1;
            if (@ratio > @initRatio*1.20)
            {
                @Alpha = 1-(abs(@ratio-@initRatio)/@ratio);
            }
            else if (@ratio < @initRatio*0.80)
            {
                @Alpha = 1-(abs(@ratio-@initRatio)/@ratio);
            }
            */

            if (!firstFrame)
            {
                float initArea3D = attArea3D.get(0);
                float initAreaUV = attAreaUV.get(0);
                float alphaValue = 0;
                float initRatio = initArea3D/initAreaUV;
                float ratio = area3D/areaUV;
                if (ratio > initRatio*1.20)
                {
                    alphaValue = 1-(abs(ratio-initRatio)/ratio);
                }
                else if (ratio < initRatio*0.80)
                {
                    alphaValue = 1-(abs(ratio-initRatio)/ratio);
                }

                attAlpha.set(ppt,alphaValue);
            }
            else
            {

                sumAreaUV += areaUV;
            }

        }
     }

    if (firstFrame)
    {

        GA_RWHandleF attArea3D(gdp->addFloatTuple(GA_ATTRIB_DETAIL,"area3D", 1));
        GA_RWHandleF attAreaUV(gdp->addFloatTuple(GA_ATTRIB_DETAIL,"areaUV", 1));

        long nbPrim = gdp->primitives().entries();

        float area3D = sumArea3D/nbPrim;
        float areaUV = sumAreaUV/nbPrim;

        attArea3D.set(GA_Offset(0),area3D);
        attAreaUV.set(GA_Offset(0),areaUV);
    }


}




//-------------------------------------------------------------------------------------




