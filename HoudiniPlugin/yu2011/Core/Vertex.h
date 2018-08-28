#ifndef __VERTEX_h__
#define __VERTEX_h__

#include <Math/Vec3.h>


using namespace Mokko;

namespace Mokko {


class Vertex
{
public:
    void SetColor(Vec3f c){this->color = c;}
    void SetPosition(Vec3f p){this->position = p;}
    void SetNormal(Vec3f n){this->normal = n;}
    void SetOrientation(Vec3f o){this->orientation = o;}
    void SetId(long id){this->id = id;}
    void SetOriginOffset(GA_Offset offset){this->originOffset = offset;}
    void SetBestMatchPosition(Vec3f bestMatch){this->bestMatchPosition = bestMatch;}
    void SetToSynthesis(int value){ this->toSynthesis = value;}
    void SetKwatraInterval(int winId, int value) {this->kwatraIntervals.at(winId) = value;}

    Vec3f GetPosition() const {return this->position;}
    Vec3f GetColor() const {return this->color;}
    Vec3f GetOrientation() const {return this->orientation;}
    Vec3f GetBestMatchPosition() const {return this->bestMatchPosition;}
    Vec3f GetNormal() const {return this->normal;}
    long GetId() const {return this->id;}
    long GetOffset() const {return this->originOffset;}
    int GetToSynthesis() const {return this->toSynthesis;}
    int IsKwatraInterval(int winId) const {return this->kwatraIntervals.at(winId);}

    Vertex() : kwatraIntervals(3, 1) {}

private :
    Vec3f position;
    Vec3f bestMatchPosition;
    Vec3f color;
    Vec3f orientation;
    Vec3f normal;
    long id;
    long originOffset;
    int toSynthesis;
    std::vector<int> kwatraIntervals;

};

}

#endif
