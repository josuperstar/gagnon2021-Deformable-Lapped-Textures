#ifndef __TBB_Atlas_TestingConcealed_h__
#define __TBB_Atlas_TestingConcealed_h__

#include "AtlasTestingConcealed.h"

using namespace TexturingFluids;


// PatchedSurface &surface,
struct TestingConcealed_executor
{
  TestingConcealed_executor(AtlasTestingConcealed &rasterizer, int w, int h,
           ParametersDeformablePatches params) : _rasterizer(rasterizer),
            _w(w), _h(h), _params(params)
  {
  }

  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    //cout << "TBB Atlas"<<endl;
    for (size_t i=r.begin();i!=r.end();++i)
    {
      _rasterizer.RasterizePrimitive(GA_Offset(i), _w, _h, _params);
    }
  }

  AtlasTestingConcealed& _rasterizer;
  //PatchedSurface& _surface;
  int _w;
  int _h;
  ParametersDeformablePatches _params;

};

#endif
