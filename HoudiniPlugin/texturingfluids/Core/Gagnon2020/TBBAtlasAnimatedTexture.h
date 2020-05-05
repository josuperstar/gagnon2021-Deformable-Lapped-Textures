#ifndef __TBB_Atlas_DeformablePatches_h__
#define __TBB_Atlas_DeformablePatches_h__

#include "AtlasDeformablePatches.h"

using namespace TexturingFluids;

struct DeformablePatches_executor
{
  DeformablePatches_executor(AtlasDeformablePatches &rasterizer, int w, int h,
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

  AtlasDeformablePatches& _rasterizer;
  int _w;
  int _h;
  ParametersDeformablePatches _params;

};

#endif
