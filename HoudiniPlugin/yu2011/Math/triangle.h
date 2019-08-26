#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__

#include "Vec3.h"
#include "math.h"

using namespace std;


namespace Mokko {

class Triangle{
	public :

    bool static SameSide(Mokko::Vec3d p1,Mokko::Vec3d p2,Mokko::Vec3d a, Mokko::Vec3d b);
    bool static PointInTriangle2d(Mokko::Vec3d p, Mokko::Vec3d a,Mokko::Vec3d b,Mokko::Vec3d c);
    Mokko::Vec3d static CreateVector( Mokko::Vec3d a,Mokko::Vec3d b);
    static double TriangleAir(Mokko::Vec3d a,Mokko::Vec3d b,Mokko::Vec3d c);
    bool static IsPointInTriangle(Mokko::Vec3d  p, Mokko::Vec3d a,Mokko::Vec3d b,Mokko::Vec3d c);
    bool static IsPointInTriangle2(Mokko::Vec3d  p, Mokko::Vec3d a,Mokko::Vec3d b,Mokko::Vec3d c);
    bool static IsPointInTriangle2D(const Mokko::Vec3d& p, const Mokko::Vec3d& a, const Mokko::Vec3d& b, const Mokko::Vec3d& c);
    bool static IsPointInTriangleRange(Mokko::Vec3d  p, Mokko::Vec3d a,Mokko::Vec3d b,Mokko::Vec3d c, float s);
    static Mokko::Vec3d ProjectPointOnTriangle(Mokko::Vec3d p,  Mokko::Vec3d a,Mokko::Vec3d b,Mokko::Vec3d c);
    Mokko::Vec3d static GetUVBarycenter(Mokko::Vec3d a,Mokko::Vec3d b, Mokko::Vec3d c,Mokko::Vec3d A,Mokko::Vec3d B, Mokko::Vec3d C, Mokko::Vec3d P);
    static Mokko::Vec3d GetBarycentricPosition(Mokko::Vec3d A,Mokko::Vec3d B, Mokko::Vec3d C, Mokko::Vec3d a, Mokko::Vec3d b, Mokko::Vec3d c, Mokko::Vec3d position);
	inline
        static double squaredDistToEdge(const Mokko::Vec3d& p,
                                 const Mokko::Vec3d& v0,
                                 const Mokko::Vec3d& edgeDirection,
								 double			  edgeLength)
		{
			// Project point onto the edge
            Mokko::Vec3d v0p(p);
			v0p -= v0;

			double d = v0p.x*edgeDirection.x + v0p.y*edgeDirection.y + v0p.z*edgeDirection.z;

			if (d<0.0) d = 0.0;
			if (d>edgeLength) d = edgeLength;

            Mokko::Vec3d projP(edgeDirection);
			projP *= d;

			// Compute the squared distance
            Mokko::Vec3d delta(projP);
			delta -= v0p;

			return delta.x*delta.x + delta.y*delta.y + delta.z*delta.z;
		}
    void static BoundingBox(Mokko::Vec3d a,Mokko::Vec3d b,Mokko::Vec3d c, Mokko::Vec3d &min,Mokko::Vec3d &max);
    void static BoundingBox2D(Mokko::Vec3d a,Mokko::Vec3d b,Mokko::Vec3d c, Mokko::Vec3d &min,Mokko::Vec3d &max);
};

}

#endif
