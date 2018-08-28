#include "DistortionUtils.h"


using namespace Mokko;
using namespace std;

void DistortionUtils::SorkineDistortionEstimation(GU_Detail *gdp,GA_Offset primitive, ParametersDistortion params)
{

}


void DistortionUtils::MultipleVariableDistortionEstimation(GU_Detail *deformableGridsGdp, GEO_Primitive *prim, ParametersDistortion params, int &nbPoints, int &nbDistorted, GA_RWHandleF    &attVA, GA_RWHandleF &attW, GA_RWHandleF &attAlpha, GA_RWHandleI &attPrimLife, GA_RWHandleF &attInitArea)
{


    GA_RWHandleI    attDilatation(deformableGridsGdp->addIntTuple(GA_ATTRIB_PRIMITIVE,"dilatation",1));
    GA_RWHandleI    attSqueeze(deformableGridsGdp->addIntTuple(GA_ATTRIB_PRIMITIVE,"squeeze",1));
    GA_RWHandleI    attShear(deformableGridsGdp->addIntTuple(GA_ATTRIB_PRIMITIVE,"shear",1));

    bool verbose = false;
    bool distorted = false;
    //------------- VERTEX ANGLE ----------------
    float maxAngleDiff = 0;

    float area;
    float initArea;
    int nbVertex = prim->getVertexCount();

    float randT = params.randT/5;
    float maxAngleT = 0.5+randT;
    float minAngleT = 0.2+randT;


    for (int i = 0; i < nbVertex; i++)
    {
        GA_Offset vertexPoint = prim->getVertexOffset(i);
        GA_Offset point = deformableGridsGdp->vertexPoint(vertexPoint);
        int b = i-1;
        if (b < 0)
            b = nbVertex-1;
        int c = i+1;
        if (c >= nbVertex)
            c = 0;

        GA_Offset vB = prim->getVertexOffset(b);
        GA_Offset pB = deformableGridsGdp->vertexPoint(vB);
        GA_Offset vC = prim->getVertexOffset(c);
        GA_Offset pC = deformableGridsGdp->vertexPoint(vC);

        UT_Vector3 AB = deformableGridsGdp->getPos3(pB)-deformableGridsGdp->getPos3(point);
        UT_Vector3 AC = deformableGridsGdp->getPos3(pC)-deformableGridsGdp->getPos3(point);
        AB.normalize();
        AC.normalize();
        float angle = dot(AB,AC);
        float initAngle = attVA.get(vertexPoint);
        float deltaAngle = fabs(angle-initAngle)/(maxAngleT-minAngleT);
        if (deltaAngle > maxAngleDiff)
            maxAngleDiff = deltaAngle;
    }





    //-------------------------------------------

    attDilatation.set(prim->getMapOffset(),0);
    attSqueeze.set(prim->getMapOffset(),0);
    attShear.set(prim->getMapOffset(),0);

    area = prim->calcArea();
    initArea = attInitArea.get(prim->getMapOffset());
    if (initArea !=0)
    {

        float dmin = params.dilatationMin+randT;
        float dmax = params.dilatationMax+randT;
        float smin = params.squeezeMin+randT/5;
        float smax = params.squeezeMax+randT/5;

        float L = attPrimLife.get(prim->getMapOffset());
        float w0 = attW.get(prim->getMapOffset());
        float t = params.deletionLife;

        bool overThreshold = false;

        float maxDCoef = 0;

        //====================== DILATATION ===========================
        //bool squeezed = false;
        if (params.computeDilatation && initArea < area)
        {

            //Dilatation dmin have to be > 1 and dmax  > dmin and < 2-3

            //std:cout << "Dilatation"<<endl;
            //double w = initArea-area;
            //w /= initArea;
            //w = fabs(w);
            double wa = fabs(initArea-area)/initArea;

            if(verbose)
            {
                cout << "----------- Dilatation-----------"<<endl;
                cout << "w "<<wa<<endl;
                cout << "dmin "<<dmin<<endl;
                cout << "dmax "<<dmax<<endl;


            }
            //attW.set(prim->getMapOffset(),w);

            if (wa > dmin && wa < dmax)
            {
                distorted = true;
                float newLife = L-1.0f;
                attPrimLife.set(prim->getMapOffset(),newLife);

                float dCoef = (wa-dmin)/(dmax-dmin);
                if (maxDCoef < dCoef)
                    maxDCoef = dCoef;

                nbDistorted++;
                attDilatation.set(prim->getMapOffset(),1);
                if(verbose)
                {
                    cout << "----------- Dilatation-----------"<<endl;
                    cout << "w "<<wa<<endl;
                    cout << "dmin "<<dmin<<endl;
                    cout << "dmax "<<dmax<<endl;
                    cout << "dCoef "<<dCoef<<endl;

                }
            }
            else if (wa >= dmax)
            {
                overThreshold = true;
                nbDistorted++;

                if(verbose)
                {
                    cout << "----------- Dilatation OverThreshold-----------"<<endl;
                    cout << "wa "<<wa<<endl;
                    cout << "dmin "<<dmin<<endl;
                    cout << "dmax "<<dmax<<endl;
                }

            }
            nbPoints++;

        }
        //========================== END DILATATION ===========================

        //====================== SQUEEZE ===========================
        //bool squeezed = false;
        else if (params.computeSqueeze && initArea > area)
        {

            //double w = initArea-area;
            //w /= initArea;
            //w = fabs(w);
            double wa = fabs(initArea-area)/(initArea);

            //attW.set(prim->getMapOffset(),w);

            if (wa > smin && wa < smax)
            {
                distorted = true;
                if(verbose)
                    cout << "Squeeze"<<endl;
                float newLife = L-1.0f;
                attPrimLife.set(prim->getMapOffset(),newLife);

                float dCoef = (wa-smin)/(smax-smin);
                if (maxDCoef < dCoef)
                    maxDCoef = dCoef;


                nbDistorted++;
                attSqueeze.set(prim->getMapOffset(),1);
            }
            else if (wa >= smax)
            {
                if(verbose)
                    cout << "Squeeze over threshold"<<endl;
                overThreshold = true;
                nbDistorted++;
            }
            nbPoints++;

        }
        //====================== SHEARING ===========================

        if (params.computeShearing && maxAngleDiff > minAngleT)
        {

            float ws = maxAngleDiff;


            //attW.set(prim->getMapOffset(),w);

            if (ws > minAngleT && ws < maxAngleT)
            {
                distorted = true;
                if(verbose)
                    cout << "Shearing"<<endl;
                float newLife = L-1.0f;
                attPrimLife.set(prim->getMapOffset(),newLife);

                float dCoef = (ws-minAngleT)/(maxAngleT-minAngleT);
                if (maxDCoef < dCoef)
                    maxDCoef = dCoef;


                attShear.set(prim->getMapOffset(),1);
            }
            else if (ws >= maxAngleT)
            {
                if(verbose)
                    cout << "Shearing over threshold"<<endl;
                overThreshold = true;
                nbDistorted++;
            }
            nbPoints++;
        }

        if(distorted)
        {
            float w = maxDCoef;

            attW.set(prim->getMapOffset(),w);

            if (w > 1)
                w = 1;

            float a = 1-(w+(1-(L/t)));
            if (a < 0)
                a = 0;

            nbVertex = prim->getVertexCount();
            for (int i = 0; i < nbVertex; i++)
            {
                GA_Offset vertexPoint = prim->getVertexOffset(i);
                GA_Offset pointOffset = deformableGridsGdp->vertexPoint(vertexPoint);

                float ca = attAlpha.get(pointOffset);
                if (ca > a)
                    ca = a;
                attAlpha.set(pointOffset,ca);
            }

            if(verbose)
                cout << "a "<<a<<endl;

        }


        if (overThreshold)
        {
            nbVertex = prim->getVertexCount();
            for (int i = 0; i < nbVertex; i++)
            {
                GA_Offset vertexPoint = prim->getVertexOffset(i);
                GA_Offset pointOffset = deformableGridsGdp->vertexPoint(vertexPoint);
                attAlpha.set(pointOffset,0);
            }
        }

    }// end if initarea != 0
}
