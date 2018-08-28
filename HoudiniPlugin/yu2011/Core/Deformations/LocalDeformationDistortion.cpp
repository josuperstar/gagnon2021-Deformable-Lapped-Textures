#include "LocalDeformationDistortion.h"


void LocalDeformationDistortion::ComputeDistortion(GU_Detail *trackersGdp, GU_Detail *deformableGridsGdp, GA_Offset trackerPpt,GA_PointGroup* pointGrp,GA_PrimitiveGroup *primGroup, ParametersDistortion params)
{
    //cout << "DISTORTION : Local Deformation Distortion"<<endl;

    GA_RWHandleF    attAlpha(deformableGridsGdp->addFloatTuple(GA_ATTRIB_POINT,"Alpha",1));
    GA_RWHandleF    attPrimLife(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,"life",1));
    GA_RWHandleF    attInitArea(deformableGridsGdp->findFloatTuple(GA_ATTRIB_PRIMITIVE,"initArea",1));
    GA_RWHandleF    attVA(deformableGridsGdp->addFloatTuple(GA_ATTRIB_VERTEX,params.initialVertexAngle.c_str(),1));
    GA_RWHandleF    attW(deformableGridsGdp->addFloatTuple(GA_ATTRIB_PRIMITIVE,params.distortionWeightName.c_str(),1));

    GA_RWHandleV3    attV(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"v",3));
    //GA_RWHandleI    attM(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"M",0));
    GA_RWHandleI    attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleI    attTemporalRemove(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"temporalRemove",1));
    GA_RWHandleF    attLife(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    //GA_RWHandleF    attRandT(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,randomThresholdDistortion,1));
    GA_RWHandleF attDivergence(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"divergence",1));


    float life = attLife.get(trackerPpt);
    float maxLife = params.deletionLife;

    int nbPoints = 0;
    int nbBelowAlpha = 0;
    //set<GA_Offset> pointDistorted;
    int nbDistorted = 0;
    {
        GEO_Primitive *prim;

        //GA_Offset primPoint;

        GA_Offset ppt;
        GA_FOR_ALL_GROUP_PTOFF(deformableGridsGdp,pointGrp,ppt)
        {
            if (attAlpha.get(ppt) <= 0)
                nbBelowAlpha++;
        }

        GA_FOR_ALL_GROUP_PRIMITIVES(deformableGridsGdp,primGroup,prim)
        {
            //move this into a function

            //DistortionUtils::MultipleVariableDistortionEstimation(deformableGridsGdp,prim,params,nbPoints,nbDistorted,attVA,attW,attAlpha,attPrimLife,attInitArea);
        }

    }
    float distortionRatio = (float)nbDistorted/(float)nbPoints;
    //if (nbDistorted > 0)
    if (distortionRatio > params.distortionRatioThreshold)
    {
        attTemporalRemove.set(trackerPpt,1);
        attTemporalRemove.set(trackerPpt+1,1);
    }

    if(attTemporalRemove.get(trackerPpt) == 1)
    {

        //we decrease patch life from f value according to the speed
        //if the patch is moving slowy, f = 1
        //if the patch is greater than maxSpeed, then f = maxLife
        //between slow and maxSpeed, we have a interpolation v according to maxlife and minlife

        // s = (v-minSpeed)/(maxSpeed-minSpeed)
        // f = s * (maxLife-minLife)

        float divergence = 1.0f;
        if (attDivergence.isValid())
        {
            divergence = attDivergence.get(trackerPpt);
        }

        UT_Vector3 speed = attV.get(trackerPpt);
        float v = speed.length();
        float minSpeed = 0.1;
        float maxSpeed = 15;
        float minLife = 0;

        float s = (v-minSpeed)/(maxSpeed-minSpeed);
        float f = s*((float)maxLife-minLife)*(divergence*3);


        life = life -1; // - f;
        GA_Offset ppt;
        GA_FOR_ALL_GROUP_PTOFF(deformableGridsGdp,pointGrp,ppt)
        {
            float currentAlpha = attAlpha.get(ppt);
            if (currentAlpha > (float)life/(float)maxLife)
            {
                currentAlpha = (float)life/(float)maxLife;
            }
            attAlpha.set(ppt,currentAlpha);
        }
        cout << "[LocalDeformationDistortion] Writing life "<<life<<endl;
        attLife.set(trackerPpt,life);
    }
}
