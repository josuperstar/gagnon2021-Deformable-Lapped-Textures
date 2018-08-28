#ifndef __PRIMITIVE_h__
#define __PRIMITIVE_h__

#include <vector>
#include <Math/Vec3.h>


using namespace Mokko;

namespace Mokko {


class Primitive
{
public:
    std::vector<long> GetVertices(){return vertices;}
    long GetId(){return id;}

    void SetVertices(std::vector<long> data){ this->vertices = data;}
    void SetId(long data) { this->id = data;}
    float area;

private :
    std::vector<long> vertices;
    long id;


};

}

#endif
