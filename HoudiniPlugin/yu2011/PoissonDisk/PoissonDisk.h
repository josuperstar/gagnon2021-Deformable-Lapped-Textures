#ifndef __PoissonDisk__
#define __PoissonDisk__
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <GEO/GEO_PointTree.h>
//#include <GA_ElementGroup.h>
#include <vector>
#include <openvdb/openvdb.h>

#include <iostream>
using namespace std;

namespace Mokko {

class PoissonDiskLegacy
{
public:
    PoissonDiskLegacy(){ this->valid = 1; this->isMature = 0;}
    PoissonDiskLegacy(UT_Vector3 d) {position = d; this->valid = true; this->isMature = 0;}
    PoissonDiskLegacy(float x, float y,float z){position = UT_Vector3(x,y,z);}
    PoissonDiskLegacy(openvdb::Vec3f vdbPoint){ position = UT_Vector3(vdbPoint.x(),vdbPoint.y(),vdbPoint.z());}

    //================= ACCESSORS =================

    int IsValid(){return valid;}
    void SetValid(int data){valid = data;}

    int IsMature(){return isMature;}
    void SetMature(int data){isMature = data;}

    int GetDensity(){return density;}
    void SetDensity(int data){density = data;}

    int GetId(){return id;}
    void SetId(int data){id = data;}

    UT_Vector3 GetNormal(){return n;}
    void SetNormal(UT_Vector3 data){n = data;}

    UT_Vector3 GetVelocity(){return v;}
    void SetVelocity(UT_Vector3 data){v = data;}

    UT_Vector3 GetCenterUV(){return centerUV;}
    void SetCenterUV(UT_Vector3 data){centerUV = data;}

    UT_Vector3 GetPosition(){return position;}


    float GetLife(){return life;}
    void SetLife(float data){life = data;}


    int GetSpawn(){return spawn;}
    void SetSpawn(int data){spawn = data;}

    float GetDynamicTau(){return dynamicTau;}
    void SetDynamicTau(float data){dynamicTau = data;}

    //===========================================

    void Print()
    {
        cout << this->GetId() << " P [" << this->GetPosition() << "] N [" << this->GetNormal() << "] V ["<<this->GetVelocity()<<"]" << endl;
    }

    bool IsInRectangle() const
    {
        return position.x() >= 0 && position.y() >= 0 && position.x() <= 1 && position.y() <= 1;
    }
    //
    bool IsInCircle() const
    {
        float fx = position.x() - 0.5f;
        float fy = position.y() - 0.5f;
        return ( fx*fx + fy*fy ) <= 0.25f;
    }

    //bool IsValid(){return valid;}

private:
    UT_Vector3 position;
    UT_Vector3 n;
    UT_Vector3 v;
    UT_Vector3 centerUV;
    int valid;
    int isMature;
    int id = 0;
    int density = 0;
    float life = 0.0f;
    int spawn = 0;
    float dynamicTau = -1.0f;
};

/*
ostream& operator<<(ostream& os, PoissonDisk& p)
{
    os << p.GetId() << ' P [' << p.Data() << '] N [' << p.GetNormal() << "] V ["<<p.GetVelocity()<<"]";
    return os;
}
*/
}

#endif
